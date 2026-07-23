<?php

namespace App\Controller;

use App\Entity\Badge;
use App\Entity\Creation;
use App\Entity\Formation;
use App\Entity\Event;
use App\Entity\Institution;
use App\Entity\LabPage;
use App\Entity\LabPageImage;
use App\Entity\Loan;
use App\Entity\LoanableItem;
use App\Entity\Machine;
use App\Entity\MaintenanceTask;
use App\Entity\Material;
use App\Entity\Place;
use App\Entity\MachineBadge;
use App\Entity\OpeningHour;
use App\Entity\Progression;
use App\Entity\RfidReader;
use App\Entity\Role;
use App\Entity\Utilisateur;
use App\Entity\UtilisateurRole;
use App\Form\BadgeAdminType;
use App\Form\CreationAdminType;
use App\Form\EventAdminType;
use App\Form\LoanableItemAdminType;
use App\Form\LoanAdminType;
use App\Form\MaintenanceBatchType;
use App\Form\MaintenanceTaskAdminType;
use App\Form\MaterialAdminType;
use App\Form\FormationAdminType;
use App\Form\InstitutionAdminType;
use App\Form\LabPageAdminType;
use App\Form\MachineAdminType;
use App\Form\PlaceAdminType;
use App\Form\RfidReaderAdminType;
use App\Form\UserAdminType;
use App\Repository\AccessRfidLogRepository;
use App\Repository\BadgeRepository;
use App\Repository\InstitutionRepository;
use App\Repository\CreationRepository;
use App\Repository\CreationVoteRepository;
use App\Repository\EventRepository;
use App\Repository\FormationRepository;
use App\Repository\LabPageRepository;
use App\Repository\LogUtilisationRepository;
use App\Repository\LoanableItemRepository;
use App\Repository\LoanRepository;
use App\Repository\MachineRepository;
use App\Repository\MaintenanceTaskRepository;
use App\Repository\MaterialRepository;
use App\Repository\OpeningHourRepository;
use App\Repository\PlaceRepository;
use App\Repository\ProgressionRepository;
use App\Repository\ReservationRepository;
use App\Reservation\ReservableResolver;
use App\Reservation\ReservableType;
use App\Repository\RfidReaderRepository;
use App\Repository\RoleRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Repository\UtilisateurRepository;
use App\Service\ModuleService;
use App\Service\SiteSettingService;
use App\Service\OpeningHoursProvider;
use App\Service\TrainingQualificationService;
use App\Entity\HomepageSectionVisibility;
use App\Repository\HomepageSectionVisibilityRepository;
use App\Service\HomepageVisibilityService;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\Form\FormInterface;
use Symfony\Component\Form\FormError;
use Symfony\Component\HttpFoundation\File\Exception\FileException;
use Symfony\Component\HttpFoundation\File\UploadedFile;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\String\Slugger\SluggerInterface;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;
use Symfony\Component\Security\Http\Attribute\IsGranted;

#[Route('/admin')]
#[IsGranted('ROLE_ADMIN')]
final class AdminController extends AbstractController
{
    #[Route('', name: 'app_admin_dashboard', methods: ['GET'])]
    #[Route('/dashboard', name: 'app_admin_dashboard_alt', methods: ['GET'])]
    #[Route('/dashboard.html', name: 'app_admin_dashboard_scoped_html', methods: ['GET'])]
    public function dashboard(
        UtilisateurRepository $users,
        MachineRepository $machines,
        FormationRepository $formations,
        ReservationRepository $reservations,
        AccessRfidLogRepository $rfidLogs,
        BadgeRepository $badges,
        ProgressionRepository $progressions,
        LogUtilisationRepository $usageLogs,
    ): Response {
        $recentActivities = $this->buildRecentActivities($rfidLogs, $reservations, $progressions, $usageLogs);

        return $this->render('site/admin-dashboard.html.twig', [
            'dashboardStats' => [
                'users' => $users->count([]),
                'machines' => $machines->count([]),
                'formations' => $formations->countVisible(),
                'reservations' => $reservations->count([]),
                'rfidLogs' => $rfidLogs->count([]),
                'badges' => $badges->count([]),
                'completedFormations' => $progressions->countCompletedVisible(),
            ],
            'recentActivities' => $recentActivities,
        ]);
    }

    #[Route('/homepage', name: 'app_admin_homepage', methods: ['GET', 'POST'])]
    public function homepage(
        Request $request,
        HomepageVisibilityService $homepageVisibility,
        HomepageSectionVisibilityRepository $homepageSections,
        RoleRepository $roles,
        EntityManagerInterface $entityManager,
    ): Response {
        $showStaffColumn = $roles->findOneBySecurityRole('ROLE_STAFF') !== null;

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_homepage', (string) $request->request->get('_token'))) {
                throw $this->createAccessDeniedException('Token CSRF invalide.');
            }

            $submittedSections = $request->request->all('sections');
            foreach ($homepageVisibility->getDefaultSections() as $sectionKey => $defaults) {
                $submitted = is_array($submittedSections[$sectionKey] ?? null) ? $submittedSections[$sectionKey] : [];
                $section = $homepageSections->findOneBySectionKey($sectionKey) ?? (new HomepageSectionVisibility())->setSectionKey($sectionKey);
                $section
                    ->setLabel($defaults['label'])
                    ->setVisibleAnonymous(isset($submitted['visibleAnonymous']))
                    ->setVisibleUser(isset($submitted['visibleUser']))
                    ->setVisibleStaff($showStaffColumn && isset($submitted['visibleStaff']))
                    ->setVisibleAdmin(isset($submitted['visibleAdmin']))
                    ->setSortOrder((int) ($submitted['sortOrder'] ?? $defaults['sortOrder']))
                    ->setUpdatedAt(new \DateTimeImmutable());

                $entityManager->persist($section);
            }

            $entityManager->flush();
            $this->addFlash('success', 'Configuration de l’accueil mise à jour.');

            return $this->redirectToRoute('app_admin_homepage');
        }

        return $this->render('site/admin-homepage.html.twig', [
            'sections' => $homepageVisibility->getAdminRows(),
            'showStaffColumn' => $showStaffColumn,
        ]);
    }

    #[Route('/machines', name: 'app_admin_machines', methods: ['GET'])]
    #[Route('/machines.html', name: 'app_admin_machines_scoped_html', methods: ['GET'])]
    #[Route('/admin-machines.html', name: 'app_admin_machines_double_legacy_html', methods: ['GET'])]
    public function machines(Request $request, MachineRepository $machines, BadgeRepository $badges): Response
    {
        $filters = $this->extractFilters($request, ['q', 'statut', 'niveau', 'badge']);

        return $this->render('site/admin-machines.html.twig', [
            'machines' => $machines->findForAdminFilters($filters),
            'filters' => $filters,
            'availableBadges' => $badges->findBy([], ['nom' => 'ASC']),
        ]);
    }

    #[Route('/machines/new', name: 'app_admin_machine_new', methods: ['GET', 'POST'])]
    public function newMachine(Request $request, EntityManagerInterface $entityManager, BadgeRepository $badges, MachineRepository $machines): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $machine = new Machine();
        $form = $this->createForm(MachineAdminType::class, $machine, [
            'category_label' => null,
            'level_value' => null,
            'icon_slug' => null,
            'materials' => [],
            'features' => [],
            'requirement_description' => null,
            'popularity' => null,
            'include_machine_token' => true,
        ]);
        $availableBadges = $badges->findBy([], ['nom' => 'ASC']);
        $submittedBadgeIds = $request->request->all('requiredBadges');

        $form->handleRequest($request);

        if ($form->isSubmitted()) {
            $existingMachine = $machines->findOneByMachineToken($machine->getMachineToken());
            if ($existingMachine !== null) {
                $form->get('machineToken')->addError(new FormError(sprintf('Le token machine "%s" est déjà utilisé.', $existingMachine->getMachineToken())));
            }
        }

        if ($form->isSubmitted() && $form->isValid()) {
            $this->applyMachineAdminFormData($machine, $form);
            $entityManager->persist($machine);
            $this->syncMachineBadges($machine, $availableBadges, $submittedBadgeIds, $entityManager);
            $entityManager->flush();

            $this->addFlash('success', sprintf('Machine "%s" créée.', $machine->getNom()));
            if ($submittedBadgeIds === []) {
                $this->addFlash('warning', 'Cette machine n’a aucun badge requis et pourra être considérée sans restriction selon la logique actuelle.');
            }

            return $this->redirectToRoute('app_admin_machines');
        }

        return $this->render('site/admin-machine-new.html.twig', [
            'machine' => $machine,
            'form' => $form,
            'availableBadges' => $availableBadges,
            'selectedBadgeIds' => array_map('intval', array_filter($submittedBadgeIds, static fn ($id): bool => is_scalar($id) && ctype_digit((string) $id))),
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/machines/{id}/edit', name: 'app_admin_machine_edit', requirements: ['id' => '\\d+'], methods: ['GET', 'POST'])]
    public function editMachine(Machine $machine, Request $request, EntityManagerInterface $entityManager, BadgeRepository $badges): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(MachineAdminType::class, $machine, [
            'category_label' => $machine->getCategoryLabel(),
            'level_value' => $this->extractMachineLevel($machine),
            'icon_slug' => $machine->getIconSlug(),
            'materials' => $machine->getMaterials(),
            'features' => $machine->getFeatures(),
            'requirement_description' => $machine->getRequirementDescription(),
            'popularity' => $machine->getPopularity(),
        ]);
        $availableBadges = $badges->findBy([], ['nom' => 'ASC']);
        $selectedBadgeIds = [];
        foreach ($machine->getMachineBadges() as $machineBadge) {
            $badge = $machineBadge->getBadge();
            if ($badge?->getId() !== null) {
                $selectedBadgeIds[] = $badge->getId();
            }
        }

        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $this->applyMachineAdminFormData($machine, $form);
            $this->syncMachineBadges($machine, $availableBadges, $request->request->all('requiredBadges'), $entityManager);
            $machine->setUpdated(new \DateTimeImmutable());

            $entityManager->flush();
            $this->addFlash('success', sprintf('Machine "%s" mise à jour.', $machine->getNom()));

            return $this->redirectToRoute('app_admin_machines');
        }

        return $this->render('site/admin-machine-edit.html.twig', [
            'machine' => $machine,
            'form' => $form,
            'availableBadges' => $availableBadges,
            'selectedBadgeIds' => $selectedBadgeIds,
        ]);
    }

    private function applyMachineAdminFormData(Machine $machine, $form): void
    {
        $category = $this->nullableString($form->get('categorie')->getData());
        $machine->setCategoryLabel($category);
        $machine->setCategorySlug($category ? $this->slugify($category) : null);

        $level = $form->get('niveau')->getData();
        if ($level === null || $level === '') {
            $machine->setLevelSlug(null);
            $machine->setLevelLabel(null);
        } else {
            $level = (int) $level;
            $machine->setLevelSlug('niveau-' . $level);
            $machine->setLevelLabel('Niveau ' . $level);
        }

        $machine->setIconSlug($this->nullableString($form->get('icone')->getData()));
        $machine->setMaterials($this->linesToArray($form->get('materiaux')->getData()));
        $machine->setFeatures($this->linesToArray($form->get('caracteristiques')->getData()));
        $machine->setRequirementDescription($this->nullableString($form->get('prerequis')->getData()));

        $popularity = $form->get('popularite')->getData();
        $machine->setPopularity($popularity === null || $popularity === '' ? null : (int) $popularity);
    }

    /**
     * @param Badge[] $availableBadges
     * @param mixed[] $submittedBadgeIds
     */
    private function syncMachineBadges(Machine $machine, array $availableBadges, array $submittedBadgeIds, EntityManagerInterface $entityManager): void
    {
        $badgesById = [];
        foreach ($availableBadges as $badge) {
            if ($badge->getId() !== null) {
                $badgesById[$badge->getId()] = $badge;
            }
        }

        $requestedIds = [];
        foreach ($submittedBadgeIds as $submittedBadgeId) {
            if (!is_scalar($submittedBadgeId) || !ctype_digit((string) $submittedBadgeId)) {
                continue;
            }

            $badgeId = (int) $submittedBadgeId;
            if (isset($badgesById[$badgeId])) {
                $requestedIds[$badgeId] = true;
            }
        }

        $existingByBadgeId = [];
        foreach ($machine->getMachineBadges() as $machineBadge) {
            $badgeId = $machineBadge->getBadge()?->getId();
            if ($badgeId === null) {
                continue;
            }

            $existingByBadgeId[$badgeId] = $machineBadge;
            if (!isset($requestedIds[$badgeId])) {
                $entityManager->remove($machineBadge);
            }
        }

        foreach (array_keys($requestedIds) as $badgeId) {
            if (isset($existingByBadgeId[$badgeId])) {
                continue;
            }

            $entityManager->persist((new MachineBadge())
                ->setMachine($machine)
                ->setBadge($badgesById[$badgeId])
                ->setRequiredForAccess(true));
        }
    }

    #[Route('/formations', name: 'app_admin_formations', methods: ['GET'])]
    #[Route('/formations.html', name: 'app_admin_formations_scoped_html', methods: ['GET'])]
    #[Route('/admin-formations.html', name: 'app_admin_formations_double_legacy_html', methods: ['GET'])]
    public function formations(Request $request, FormationRepository $formations, ProgressionRepository $progressions, BadgeRepository $badges): Response
    {
        $filters = $this->extractFilters($request, ['q', 'niveau', 'badge']);

        return $this->render('site/admin-formations.html.twig', [
            'formations' => $formations->findForAdminFilters($filters),
            'progressionStats' => $this->buildFormationProgressionStats($progressions),
            'filters' => $filters,
            'availableBadges' => $badges->findBy([], ['nom' => 'ASC']),
        ]);
    }


    #[Route('/formations/new', name: 'app_admin_formation_new', methods: ['GET', 'POST'])]
    public function newFormation(Request $request, EntityManagerInterface $entityManager, FormationRepository $formations): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $formation = new Formation();
        $form = $this->createForm(FormationAdminType::class, $formation);
        $form->handleRequest($request);

        if ($form->isSubmitted()) {
            $existingFormation = $formations->findOneByNormalizedTitle($formation->getTitre());
            if ($existingFormation !== null) {
                $form->get('titre')->addError(new FormError(sprintf('La formation "%s" existe déjà.', $existingFormation->getTitre())));
            }
        }

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->persist($formation);
            $entityManager->flush();
            $this->addFlash('success', sprintf('Formation "%s" créée.', $formation->getTitre()));
            if ($formation->getBadge() === null) {
                $this->addFlash('warning', 'Cette formation ne donnera aucun badge tant qu’un badge n’est pas associé.');
            }

            return $this->redirectToRoute('app_admin_formations');
        }

        return $this->render('site/admin-formation-new.html.twig', [
            'formation' => $formation,
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/formations/{id}/edit', name: 'app_admin_formation_edit', requirements: ['id' => '\\d+'], methods: ['GET', 'POST'])]
    public function editFormation(Formation $formation, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(FormationAdminType::class, $formation);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->flush();
            $this->addFlash('success', sprintf('Formation "%s" mise à jour.', $formation->getTitre()));

            return $this->redirectToRoute('app_admin_formations');
        }

        return $this->render('site/admin-formation-edit.html.twig', [
            'formation' => $formation,
            'form' => $form,
        ]);
    }

    #[Route('/reservations', name: 'app_admin_reservations', methods: ['GET'])]
    #[Route('/reservations.html', name: 'app_admin_reservations_scoped_html', methods: ['GET'])]
    #[Route('/admin-reservations.html', name: 'app_admin_reservations_double_legacy_html', methods: ['GET'])]
    public function reservations(Request $request, ReservationRepository $reservations, ReservableResolver $reservables): Response
    {
        $filters = $this->extractFilters($request, ['q', 'statut', 'dateFrom', 'dateTo', 'reservableType']);
        $rows = $reservations->findForAdminFilters($filters);
        $reservables->warm($rows);

        return $this->render('site/admin-reservations.html.twig', [
            'reservations' => $rows,
            'filters' => $filters,
        ]);
    }


    #[Route('/horaires', name: 'app_admin_opening_hours', methods: ['GET', 'POST'])]
    public function openingHours(Request $request, OpeningHourRepository $openingHours, OpeningHoursProvider $openingHoursProvider, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $rows = $this->ensureOpeningHourRows($openingHours, $openingHoursProvider, $entityManager);
        $errors = [];

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_opening_hours', (string) $request->request->get('_token'))) {
                $errors['_global'][] = 'Token CSRF invalide.';
            }

            foreach ($rows as $row) {
                $day = $row->getDayOfWeek();
                $isClosed = $request->request->getBoolean('closed_' . $day);
                $openInput = trim((string) $request->request->get('open_' . $day, ''));
                $closeInput = trim((string) $request->request->get('close_' . $day, ''));

                if ($isClosed) {
                    continue;
                }

                $openTime = $this->parseAdminTime($openInput);
                $closeTime = $this->parseAdminTime($closeInput);
                if (!$openTime) {
                    $errors[$day][] = 'Heure ouverture obligatoire au format HH:mm.';
                }
                if (!$closeTime) {
                    $errors[$day][] = 'Heure fermeture obligatoire au format HH:mm.';
                }
                if ($openTime && $closeTime && $closeTime <= $openTime) {
                    $errors[$day][] = 'Heure fermeture doit être après heure ouverture.';
                }
            }

            if ($errors === []) {
                foreach ($rows as $row) {
                    $day = $row->getDayOfWeek();
                    $isClosed = $request->request->getBoolean('closed_' . $day);
                    $row->setIsClosed($isClosed);
                    if ($isClosed) {
                        $row->setOpenTime(null)->setCloseTime(null);
                    } else {
                        $row->setOpenTime($this->parseAdminTime((string) $request->request->get('open_' . $day)));
                        $row->setCloseTime($this->parseAdminTime((string) $request->request->get('close_' . $day)));
                    }
                    $row->setUpdatedAt(new \DateTimeImmutable());
                }

                $entityManager->flush();
                $this->addFlash('success', 'Horaires d’ouverture mis à jour.');

                return $this->redirectToRoute('app_admin_opening_hours');
            }
        }

        return $this->render('site/admin-opening-hours.html.twig', [
            'openingHours' => $rows,
            'errors' => $errors,
        ], $request->isMethod('POST') ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/utilisateurs', name: 'app_admin_users', methods: ['GET'])]
    #[Route('/utilisateurs.html', name: 'app_admin_users_scoped_html', methods: ['GET'])]
    #[Route('/admin-utilisateurs.html', name: 'app_admin_users_double_legacy_html', methods: ['GET'])]
    public function users(Request $request, UtilisateurRepository $users, AccessRfidLogRepository $logs, ProgressionRepository $progressions, RoleRepository $roles): Response
    {
        $filters = $this->extractFilters($request, ['q', 'statut', 'role']);
        $logCounts = [];
        foreach ($logs->createQueryBuilder('log')
            ->select('IDENTITY(log.utilisateur) AS userId, COUNT(log.id) AS logCount')
            ->where('log.utilisateur IS NOT NULL')
            ->groupBy('log.utilisateur')
            ->getQuery()
            ->getArrayResult() as $row) {
            $logCounts[(int) $row['userId']] = (int) $row['logCount'];
        }

        return $this->render('site/admin-utilisateurs.html.twig', [
            'users' => $users->findForAdminFilters($filters),
            'logCounts' => $logCounts,
            'progressionCounts' => $this->buildUserProgressionStats($progressions),
            'filters' => $filters,
            'roleChoices' => $this->buildAdminRoleChoices($roles),
        ]);
    }

    #[Route('/utilisateurs/new', name: 'app_admin_user_new', methods: ['GET', 'POST'])]
    public function newUser(
        Request $request,
        EntityManagerInterface $entityManager,
        UtilisateurRepository $users,
        RoleRepository $roles,
        UserPasswordHasherInterface $passwordHasher,
    ): Response {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $user = (new Utilisateur())
            ->setStatut('actif')
            ->setNotificationEmail(true)
            ->setNotificationPush(false)
            ->setRappelReservation(true)
            ->setTheme('system')
            ->setLangue('fr')
            ->setIsVerified(true);

        $roleChoices = $this->buildAdminRoleChoices($roles);
        $form = $this->createForm(UserAdminType::class, $user, ['role_choices' => $roleChoices]);
        $form->handleRequest($request);

        if ($form->isSubmitted()) {
            $email = mb_strtolower(trim($user->getEmail()));
            $username = trim($user->getUsername());
            $rfid = $this->nullableString($user->getIdentifiantRfid());
            $user
                ->setEmail($email)
                ->setUsername($username)
                ->setIdentifiantRfid($rfid)
                ->setNumeroId($this->nullableString($user->getNumeroId()));

            if ($users->findOneBy(['email' => $email]) !== null) {
                $form->get('email')->addError(new FormError('Cet email est déjà utilisé.'));
            }
            if ($users->findOneBy(['username' => $username]) !== null) {
                $form->get('username')->addError(new FormError('Ce username est déjà utilisé.'));
            }
            if ($rfid !== null && $users->findOneBy(['identifiantRfid' => $rfid]) !== null) {
                $form->get('identifiantRfid')->addError(new FormError('Cet identifiant RFID est déjà utilisé.'));
            }

            $plainPassword = (string) $form->get('plainPassword')->getData();
            $confirmPassword = (string) $form->get('confirmPassword')->getData();
            if ($plainPassword !== $confirmPassword) {
                $form->get('confirmPassword')->addError(new FormError('La confirmation ne correspond pas au mot de passe.'));
            }

            $selectedRole = (string) $form->get('role')->getData();
            $role = $roles->findOneBySecurityRole($selectedRole);
            if ($role === null || !in_array($selectedRole, array_values($roleChoices), true)) {
                $form->get('role')->addError(new FormError('Rôle invalide.'));
            }
        }

        if ($form->isSubmitted() && $form->isValid()) {
            $selectedRole = (string) $form->get('role')->getData();
            $role = $roles->findOneBySecurityRole($selectedRole);
            if ($role === null) {
                throw new \LogicException('Rôle validé introuvable.');
            }

            $user->setPassword($passwordHasher->hashPassword($user, (string) $form->get('plainPassword')->getData()));
            $entityManager->persist($user);
            $entityManager->persist((new UtilisateurRole())->setUtilisateur($user)->setRole($role));
            $entityManager->flush();

            $this->addFlash('success', sprintf('Utilisateur "%s" créé.', $user->getEmail()));

            return $this->redirectToRoute('app_admin_users');
        }

        return $this->render('site/admin-utilisateur-new.html.twig', [
            'user' => $user,
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }


    #[Route('/utilisateurs/{id}', name: 'app_admin_user_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function userDetail(
        int $id,
        UtilisateurRepository $users,
        AccessRfidLogRepository $logs,
        ProgressionRepository $progressions,
        ReservationRepository $reservations,
        LogUtilisationRepository $usageLogs,
        FormationRepository $formations,
        TrainingQualificationService $qualification,
    ): Response {
        $user = $users->find($id);
        if (!$user) {
            throw $this->createNotFoundException('Utilisateur introuvable');
        }

        $physicalTrainingRows = [];
        foreach ($formations->findVisible(['titre' => 'ASC']) as $formation) {
            if ($formation->getBadge() === null) {
                continue;
            }

            $status = $qualification->getStatus($formation, $user);
            if ($status['physicalFormation'] === null) {
                continue;
            }

            $physicalTrainingRows[] = [
                'formation' => $formation,
                'status' => $status,
            ];
        }

        return $this->render('site/admin-utilisateur-detail.html.twig', [
            'user' => $user,
            'rfidLogs' => $logs->findBy(['utilisateur' => $user], ['createdAt' => 'DESC']),
            'progressions' => $progressions->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']),
            'reservations' => $reservations->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']),
            'usageLogs' => $usageLogs->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']),
            'physicalTrainingRows' => $physicalTrainingRows,
        ]);
    }

    #[Route('/utilisateurs/{id}/person-type', name: 'app_admin_user_person_type', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function updatePersonType(
        int $id,
        Request $request,
        UtilisateurRepository $users,
        RoleRepository $roles,
        ReservationRepository $reservations,
        EntityManagerInterface $entityManager,
    ): Response {
        $user = $users->find($id);
        if (!$user instanceof Utilisateur) {
            throw $this->createNotFoundException('Utilisateur introuvable.');
        }

        if (!$this->isCsrfTokenValid('person_type_' . $id, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Mise à jour refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_user_detail', ['id' => $id]);
        }

        // Person-type is stored as ROLE membership (roles are created on demand).
        $this->setPersonTypeRole($user, 'staff', $request->request->getBoolean('is_staff'), $roles, $entityManager);
        $this->setPersonTypeRole($user, 'trainer', $request->request->getBoolean('is_trainer'), $roles, $entityManager);

        // Being bookable is the admin's call; the person then owns their own
        // slots and durations. Turning it off cancels what was already booked —
        // leaving live appointments on a page nobody can reach would strand them.
        $wasBookable = $user->isBookable();
        $isBookable = $request->request->getBoolean('is_bookable');
        $user->setBookable($isBookable);
        if ($wasBookable && !$isBookable) {
            $reservations->cancelUpcomingForReservable(ReservableType::User, $id);
        }

        $entityManager->flush();
        $this->addFlash('success', 'Type de personne mis à jour.');

        return $this->redirectToRoute('app_admin_user_detail', ['id' => $id]);
    }

    private function setPersonTypeRole(
        Utilisateur $user,
        string $roleName,
        bool $shouldHave,
        RoleRepository $roles,
        EntityManagerInterface $entityManager,
    ): void {
        $existing = null;
        foreach ($user->getUtilisateurRoles() as $utilisateurRole) {
            if (strtolower((string) $utilisateurRole->getRole()?->getNom()) === $roleName) {
                $existing = $utilisateurRole;
                break;
            }
        }

        if ($shouldHave && $existing === null) {
            $role = $roles->findOneBy(['nom' => $roleName]);
            if ($role === null) {
                $role = (new Role())->setNom($roleName);
                $entityManager->persist($role);
            }
            $entityManager->persist((new UtilisateurRole())->setUtilisateur($user)->setRole($role));
        } elseif (!$shouldHave && $existing !== null) {
            $entityManager->remove($existing);
        }
    }

    #[Route('/utilisateurs/{userId}/formations/{formationId}/validation-physique', name: 'app_admin_validate_physical_training', requirements: ['userId' => '\d+', 'formationId' => '\d+'], methods: ['POST'])]
    public function validatePhysicalTraining(
        int $userId,
        int $formationId,
        Request $request,
        UtilisateurRepository $users,
        FormationRepository $formations,
        ProgressionRepository $progressions,
        TrainingQualificationService $qualification,
        EntityManagerInterface $entityManager,
    ): Response {
        $user = $users->find($userId);
        $formation = $formations->find($formationId);
        if (!$user instanceof Utilisateur || !$formation instanceof Formation) {
            throw $this->createNotFoundException('Utilisateur ou formation introuvable.');
        }

        if (!$this->isCsrfTokenValid(
            'validate_physical_training_' . $userId . '_' . $formationId,
            (string) $request->request->get('_token'),
        )) {
            $this->addFlash('error', 'La validation physique a été refusée : token CSRF invalide.');
            return $this->redirectToRoute('app_admin_user_detail', ['id' => $userId]);
        }

        $physicalFormation = $qualification->getPhysicalFormation($formation);
        if (!$physicalFormation instanceof Formation) {
            $this->addFlash('error', 'La validation physique n’est pas configurée pour cette formation. Importez le script de données fourni.');
            return $this->redirectToRoute('app_admin_user_detail', ['id' => $userId]);
        }

        $progression = $progressions->findOneBy([
            'utilisateur' => $user,
            'formation' => $physicalFormation,
        ]);

        if (!$progression instanceof Progression) {
            $progression = (new Progression())
                ->setUtilisateur($user)
                ->setFormation($physicalFormation);
            $entityManager->persist($progression);
        }

        $now = new \DateTimeImmutable();
        $minimumEnd = $progression->getDateDebut()->modify('+1 second');
        $progression
            ->setScore(100)
            ->setCompleted(true)
            ->setDateEnd($now > $minimumEnd ? $now : $minimumEnd);

        $entityManager->flush();
        $status = $qualification->getStatus($formation, $user);

        $this->addFlash(
            'success',
            $status['badgeUnlocked']
                ? sprintf('Formation physique validée. Le badge « %s » est maintenant attribué.', $formation->getBadge()?->getNom() ?? 'formation')
                : sprintf('Formation physique validée. Le badge sera attribué dès que la progression théorique atteindra 80 %% (actuellement %d %%).', $status['overallPercent']),
        );

        return $this->redirectToRoute('app_admin_user_detail', ['id' => $userId]);
    }

    #[Route('/creations', name: 'app_admin_creations', methods: ['GET'])]
    public function creations(CreationRepository $creations, CreationVoteRepository $votes): Response
    {
        $creationItems = $creations->findAllForModeration();
        $ratingStats = $votes->getStatsByCreation($creationItems);
        $creationRows = [];

        foreach ($creationItems as $creation) {
            $stats = $ratingStats[$creation->getId()] ?? ['average' => null, 'count' => 0];
            $creationRows[] = [
                'creation' => $creation,
                'averageRating' => $stats['average'],
                'ratingCount' => $stats['count'],
                'voteCount' => $stats['count'],
            ];
        }

        return $this->render('site/admin-creations.html.twig', [
            'creationRows' => $creationRows,
        ]);
    }

    #[Route('/creations/new', name: 'app_admin_creation_new', methods: ['GET', 'POST'])]
    public function newCreation(Request $request, EntityManagerInterface $entityManager, SluggerInterface $slugger): Response
    {
        $creation = new Creation();
        $form = $this->createForm(CreationAdminType::class, $creation);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $this->normalizeCreationData($creation);

            if (!$this->handleCreationUploads($creation, $form, $slugger)) {
                return $this->render('site/admin-creation-new.html.twig', [
                    'creation' => $creation,
                    'form' => $form,
                ], new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY));
            }

            $entityManager->persist($creation);
            $entityManager->flush();
            $this->addFlash('success', sprintf('Création "%s" ajoutée.', $creation->getTitle()));

            return $this->redirectToRoute('app_admin_creations');
        }

        return $this->render('site/admin-creation-new.html.twig', [
            'creation' => $creation,
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/creations/{id}/edit', name: 'app_admin_creation_edit', requirements: ['id' => '\\d+'], methods: ['GET', 'POST'])]
    public function editCreation(Creation $creation, Request $request, EntityManagerInterface $entityManager, SluggerInterface $slugger): Response
    {
        $form = $this->createForm(CreationAdminType::class, $creation);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $this->normalizeCreationData($creation);

            if (!$this->handleCreationUploads($creation, $form, $slugger)) {
                return $this->render('site/admin-creation-edit.html.twig', [
                    'creation' => $creation,
                    'form' => $form,
                ], new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY));
            }

            $creation->setUpdatedAt(new \DateTimeImmutable());
            $entityManager->flush();
            $this->addFlash('success', sprintf('Création "%s" mise à jour.', $creation->getTitle()));

            return $this->redirectToRoute('app_admin_creations');
        }

        return $this->render('site/admin-creation-edit.html.twig', [
            'creation' => $creation,
            'form' => $form,
        ]);
    }

    #[Route('/creations/{id}/toggle-published', name: 'app_admin_creation_toggle_published', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function toggleCreationPublished(Creation $creation, Request $request, EntityManagerInterface $entityManager): Response
    {
        if (!$this->isCsrfTokenValid('toggle_creation_' . $creation->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Modification refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_creations');
        }

        $creation
            ->setIsPublished(!$creation->isPublished())
            ->setUpdatedAt(new \DateTimeImmutable());
        $entityManager->flush();
        $this->addFlash('success', sprintf('Statut de "%s" mis à jour.', $creation->getTitle()));

        return $this->redirectToRoute('app_admin_creations');
    }

    #[Route('/creations/{id}/toggle-pinned', name: 'app_admin_creation_toggle_pinned', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function toggleCreationPinned(Creation $creation, Request $request, EntityManagerInterface $entityManager): Response
    {
        if (!$this->isCsrfTokenValid('toggle_pinned_creation_' . $creation->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Modification refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_creations');
        }

        $creation
            ->setIsPinned(!$creation->isPinned())
            ->setUpdatedAt(new \DateTimeImmutable());
        $entityManager->flush();
        $this->addFlash('success', sprintf('Épinglage de "%s" mis à jour.', $creation->getTitle()));

        return $this->redirectToRoute('app_admin_creations');
    }

    #[Route('/settings', name: 'app_admin_settings', methods: ['GET', 'POST'])]
    public function settings(Request $request, SiteSettingService $siteSettings): Response
    {
        $availableLocales = ['fr' => 'Français', 'en' => 'English', 'es' => 'Español', 'de' => 'Deutsch', 'it' => 'Italiano'];

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_settings', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_admin_settings');
            }

            $locale = (string) $request->request->get('default_locale');
            if (array_key_exists($locale, $availableLocales)) {
                $siteSettings->setDefaultLocale($locale);
                $siteSettings->setAlertBanner(
                    $request->request->getBoolean('alert_banner_enabled'),
                    (string) $request->request->get('alert_banner_text'),
                );
                $siteSettings->setLabPagesNavLabel((string) $request->request->get('lab_pages_nav_label'));
                $siteSettings->setLabRules(
                    (string) $request->request->get('lab_rules_html'),
                    (string) $request->request->get('lab_rules_pdf_url'),
                );
                if ($request->request->getBoolean('regenerate_ical_token')) {
                    $siteSettings->regenerateIcalFeedToken();
                    $this->addFlash('success', 'Jeton des flux iCal régénéré : les abonnements existants doivent être renouvelés.');
                }
                $this->addFlash('success', 'Réglages mis à jour.');
            } else {
                $this->addFlash('error', 'Langue invalide.');
            }

            return $this->redirectToRoute('app_admin_settings');
        }

        return $this->render('site/admin-settings.html.twig', [
            'availableLocales' => $availableLocales,
            'currentLocale' => $siteSettings->getDefaultLocale(),
            'alertBannerEnabled' => $siteSettings->isAlertBannerEnabled(),
            'alertBannerText' => $siteSettings->getAlertBannerText(),
            'labPagesNavLabel' => $siteSettings->getLabPagesNavLabel(),
            'labRulesHtml' => $siteSettings->getLabRulesHtml(),
            'labRulesPdfUrl' => $siteSettings->getLabRulesPdfUrl(),
            'icalFeedToken' => $siteSettings->getIcalFeedToken(),
        ]);
    }

    #[Route('/modules', name: 'app_admin_modules', methods: ['GET', 'POST'])]
    public function modules(Request $request, ModuleService $modules): Response
    {
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_modules', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_admin_modules');
            }

            $checked = (array) $request->request->all('modules');
            foreach (ModuleService::MODULES as $key) {
                $modules->setEnabled($key, in_array($key, $checked, true));
            }
            $this->addFlash('success', 'Modules mis à jour.');

            return $this->redirectToRoute('app_admin_modules');
        }

        return $this->render('site/admin-modules.html.twig', [
            'modules' => $modules->all(),
        ]);
    }

    #[Route('/creations/{id}/delete', name: 'app_admin_creation_delete', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function deleteCreation(Creation $creation, Request $request, EntityManagerInterface $entityManager): Response
    {
        if (!$this->isCsrfTokenValid('delete_creation_' . $creation->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Suppression refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_creations');
        }

        $title = $creation->getTitle();
        $imageFilename = $creation->getImageFilename();
        $fileFilename = $creation->getFileFilename();

        $entityManager->remove($creation);
        $entityManager->flush();

        $this->deleteCreationUploadIfSafe('public/uploads/creations/images', $imageFilename);
        $this->deleteCreationUploadIfSafe('public/uploads/creations/files', $fileFilename);
        $this->addFlash('success', sprintf('Création "%s" supprimée.', $title));

        return $this->redirectToRoute('app_admin_creations');
    }

    #[Route('/badges', name: 'app_admin_badges', methods: ['GET'])]
    #[Route('/badges.html', name: 'app_admin_badges_scoped_html', methods: ['GET'])]
    #[Route('/admin-badges.html', name: 'app_admin_badges_double_legacy_html', methods: ['GET'])]
    public function badges(Request $request, BadgeRepository $badges, UtilisateurBadgeRepository $userBadges, FormationRepository $formations): Response
    {
        $filters = $this->extractFilters($request, ['q']);
        $badgeRows = [];
        foreach ($badges->findForAdminFilters($filters) as $badge) {
            $badgeRows[] = [
                'badge' => $badge,
                'userCount' => $userBadges->count(['badge' => $badge]),
                'formationCount' => $formations->count(['badge' => $badge]),
            ];
        }

        return $this->render('site/admin-badges.html.twig', [
            'badgeRows' => $badgeRows,
            'filters' => $filters,
        ]);
    }


    #[Route('/badges/new', name: 'app_admin_badge_new', methods: ['GET', 'POST'])]
    public function newBadge(Request $request, EntityManagerInterface $entityManager, BadgeRepository $badges): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $badge = new Badge();
        $form = $this->createForm(BadgeAdminType::class, $badge);
        $form->handleRequest($request);

        if ($form->isSubmitted()) {
            $existingBadge = $badges->findOneByNormalizedName($badge->getNom());
            if ($existingBadge !== null) {
                $form->get('nom')->addError(new FormError(sprintf('Le badge "%s" existe déjà.', $existingBadge->getNom())));
            }
        }

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->persist($badge);
            $entityManager->flush();
            $this->addFlash('success', sprintf('Badge "%s" créé.', $badge->getNom()));

            return $this->redirectToRoute('app_admin_badges');
        }

        return $this->render('site/admin-badge-new.html.twig', [
            'badge' => $badge,
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }


    #[Route('/badges/{id}/edit', name: 'app_admin_badge_edit', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function editBadge(Badge $badge, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(BadgeAdminType::class, $badge);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->flush();
            $this->addFlash('success', sprintf('Badge "%s" mis à jour.', $badge->getNom()));

            return $this->redirectToRoute('app_admin_badges');
        }

        return $this->render('site/admin-badge-edit.html.twig', [
            'badge' => $badge,
            'form' => $form,
        ]);
    }

    #[Route('/institutions', name: 'app_admin_institutions', methods: ['GET'])]
    public function institutions(InstitutionRepository $institutions): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return $this->render('site/admin-institutions.html.twig', [
            'institutions' => $institutions->findBy([], ['nom' => 'ASC']),
        ]);
    }

    #[Route('/institutions/new', name: 'app_admin_institution_new', methods: ['GET', 'POST'])]
    public function newInstitution(Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $institution = new Institution();
        $form = $this->createForm(InstitutionAdminType::class, $institution);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->persist($institution);
            $entityManager->flush();
            $this->addFlash('success', sprintf('Institution "%s" créée.', $institution->getNom()));

            return $this->redirectToRoute('app_admin_institutions');
        }

        return $this->render('site/admin-institution-new.html.twig', [
            'institution' => $institution,
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/institutions/{id}/edit', name: 'app_admin_institution_edit', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function editInstitution(Institution $institution, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(InstitutionAdminType::class, $institution);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->flush();
            $this->addFlash('success', sprintf('Institution "%s" mise à jour.', $institution->getNom()));

            return $this->redirectToRoute('app_admin_institutions');
        }

        return $this->render('site/admin-institution-edit.html.twig', [
            'institution' => $institution,
            'form' => $form,
        ]);
    }

    #[Route('/institutions/{id}/delete', name: 'app_admin_institution_delete', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function deleteInstitution(Institution $institution, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('delete_institution_' . $institution->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Suppression refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_institutions');
        }

        $name = $institution->getNom();
        $entityManager->remove($institution);
        $entityManager->flush();
        $this->addFlash('success', sprintf('Institution "%s" supprimée.', $name));

        return $this->redirectToRoute('app_admin_institutions');
    }

    #[Route('/lab-pages', name: 'app_admin_lab_pages', methods: ['GET'])]
    public function labPages(LabPageRepository $labPages): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return $this->render('site/admin-lab-pages.html.twig', [
            'topLevelPages' => $labPages->findTopLevel(),
        ]);
    }

    #[Route('/lab-pages/new', name: 'app_admin_lab_page_new', methods: ['GET', 'POST'])]
    public function newLabPage(Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $page = new LabPage();
        $form = $this->createForm(LabPageAdminType::class, $page);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->persist($page);
            $entityManager->flush();
            $this->addFlash('success', sprintf('Page "%s" créée.', $page->getTitre()));

            return $this->redirectToRoute('app_admin_lab_pages');
        }

        return $this->render('site/admin-lab-page-new.html.twig', [
            'page' => $page,
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/lab-pages/{id}/edit', name: 'app_admin_lab_page_edit', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function editLabPage(LabPage $page, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $hadChildren = !$page->getChildren()->isEmpty();
        $form = $this->createForm(LabPageAdminType::class, $page);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            if ($hadChildren) {
                // Keep the hierarchy fixed at 2 levels: a page that already
                // has children must stay top-level.
                $page->setParentPage(null);
            }
            $page->setUpdatedAt(new \DateTimeImmutable());
            $entityManager->flush();
            $this->addFlash('success', sprintf('Page "%s" mise à jour.', $page->getTitre()));

            return $this->redirectToRoute('app_admin_lab_pages');
        }

        return $this->render('site/admin-lab-page-edit.html.twig', [
            'page' => $page,
            'form' => $form,
            'hadChildren' => $hadChildren,
        ]);
    }

    #[Route('/lab-pages/{id}/delete', name: 'app_admin_lab_page_delete', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function deleteLabPage(LabPage $page, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('delete_lab_page_' . $page->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Suppression refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_lab_pages');
        }

        $title = $page->getTitre();
        $imageFilenames = array_map(static fn (LabPageImage $image): string => $image->getImageFilename(), $page->getImages()->toArray());
        $entityManager->remove($page);
        $entityManager->flush();

        foreach ($imageFilenames as $imageFilename) {
            $this->deleteLabPageImageFileIfSafe($imageFilename);
        }
        $this->addFlash('success', sprintf('Page "%s" supprimée.', $title));

        return $this->redirectToRoute('app_admin_lab_pages');
    }

    #[Route('/lab-pages/{id}/photos', name: 'app_admin_lab_page_photo_add', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function addLabPagePhoto(LabPage $page, Request $request, EntityManagerInterface $entityManager, SluggerInterface $slugger): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('lab_page_photo_' . $page->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Ajout refusé : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
        }

        $uploadedFile = $request->files->get('photo');
        if (!$uploadedFile instanceof UploadedFile) {
            $this->addFlash('error', 'Choisissez une photo à ajouter.');

            return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
        }

        $extension = strtolower($uploadedFile->guessExtension() ?: $uploadedFile->getClientOriginalExtension() ?: 'bin');
        if ($extension === 'jpeg') {
            $extension = 'jpg';
        }

        if (!in_array($extension, ['png', 'jpg', 'webp'], true)) {
            $this->addFlash('error', 'Choisissez une image PNG, JPG, JPEG ou WEBP.');

            return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
        }

        $uploadDir = $this->getParameter('kernel.project_dir') . '/public/uploads/lab-pages';
        if (!is_dir($uploadDir) && !mkdir($uploadDir, 0775, true) && !is_dir($uploadDir)) {
            $this->addFlash('error', 'Impossible de créer le dossier des photos.');

            return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
        }

        $fileName = sprintf('lab-page-%d-%s.%s', $page->getId(), bin2hex(random_bytes(6)), $extension);

        try {
            $uploadedFile->move($uploadDir, $fileName);
        } catch (FileException) {
            $this->addFlash('error', 'Impossible de copier la photo.');

            return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
        }

        $image = (new LabPageImage())->setLabPage($page)->setImageFilename($fileName);
        $entityManager->persist($image);
        $entityManager->flush();
        $this->addFlash('success', 'Photo ajoutée.');

        return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
    }

    #[Route('/lab-pages/{id}/photos/{photoId}/delete', name: 'app_admin_lab_page_photo_delete', requirements: ['id' => '\d+', 'photoId' => '\d+'], methods: ['POST'])]
    public function deleteLabPagePhoto(LabPage $page, int $photoId, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('lab_page_photo_delete_' . $photoId, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Suppression refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
        }

        foreach ($page->getImages() as $image) {
            if ($image->getId() === $photoId) {
                $filename = $image->getImageFilename();
                $entityManager->remove($image);
                $entityManager->flush();
                $this->deleteLabPageImageFileIfSafe($filename);
                $this->addFlash('success', 'Photo supprimée.');
                break;
            }
        }

        return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
    }

    private function deleteLabPageImageFileIfSafe(string $filename): void
    {
        if ($filename === '' || !preg_match('/^[A-Za-z0-9._-]+$/', $filename)) {
            return;
        }

        $path = $this->getParameter('kernel.project_dir') . '/public/uploads/lab-pages/' . $filename;
        if (is_file($path)) {
            @unlink($path);
        }
    }

    #[Route('/places', name: 'app_admin_places', methods: ['GET'])]
    public function places(PlaceRepository $places): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return $this->render('site/admin-places.html.twig', [
            'places' => $places->findBy([], ['nom' => 'ASC']),
        ]);
    }

    #[Route('/places/new', name: 'app_admin_place_new', methods: ['GET', 'POST'])]
    public function newPlace(Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $place = new Place();
        $form = $this->createForm(PlaceAdminType::class, $place);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->persist($place);
            $entityManager->flush();
            $this->addFlash('success', sprintf('Espace "%s" créé.', $place->getNom()));

            return $this->redirectToRoute('app_admin_places');
        }

        return $this->render('site/admin-place-new.html.twig', [
            'place' => $place,
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/places/{id}/edit', name: 'app_admin_place_edit', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function editPlace(Place $place, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(PlaceAdminType::class, $place);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->flush();
            $this->addFlash('success', sprintf('Espace "%s" mis à jour.', $place->getNom()));

            return $this->redirectToRoute('app_admin_places');
        }

        return $this->render('site/admin-place-edit.html.twig', [
            'place' => $place,
            'form' => $form,
        ]);
    }

    #[Route('/places/{id}/delete', name: 'app_admin_place_delete', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function deletePlace(Place $place, Request $request, EntityManagerInterface $entityManager, ReservationRepository $reservations): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('delete_place_' . $place->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Suppression refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_places');
        }

        $name = $place->getNom();

        // Reservations no longer hold a FK to the place, so nothing cascades:
        // cancel the upcoming ones explicitly. Past bookings stay, carrying the
        // resource name snapshotted in reservableLabel.
        $cancelled = $reservations->cancelUpcomingForReservable(ReservableType::Place, $place->getId());

        $entityManager->remove($place);
        $entityManager->flush();
        $this->addFlash('success', $cancelled > 0
            ? sprintf('Espace "%s" supprimé (%d réservation(s) à venir annulée(s)).', $name, $cancelled)
            : sprintf('Espace "%s" supprimé.', $name));

        return $this->redirectToRoute('app_admin_places');
    }

    #[Route('/events', name: 'app_admin_events', methods: ['GET'])]
    public function events(EventRepository $events): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return $this->render('site/admin-events.html.twig', [
            'events' => $events->findBy([], ['dateDebut' => 'DESC']),
        ]);
    }

    #[Route('/events/new', name: 'app_admin_event_new', methods: ['GET', 'POST'])]
    public function newEvent(Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $event = new Event();
        $form = $this->createForm(EventAdminType::class, $event);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->persist($event);
            $entityManager->flush();
            $this->addFlash('success', sprintf('Événement "%s" créé.', $event->getTitre()));

            return $this->redirectToRoute('app_admin_events');
        }

        return $this->render('site/admin-event-new.html.twig', [
            'event' => $event,
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/events/{id}/edit', name: 'app_admin_event_edit', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function editEvent(Event $event, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(EventAdminType::class, $event);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->flush();
            $this->addFlash('success', sprintf('Événement "%s" mis à jour.', $event->getTitre()));

            return $this->redirectToRoute('app_admin_events');
        }

        return $this->render('site/admin-event-edit.html.twig', [
            'event' => $event,
            'form' => $form,
        ]);
    }

    #[Route('/events/{id}/delete', name: 'app_admin_event_delete', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function deleteEvent(Event $event, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('delete_event_' . $event->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Suppression refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_events');
        }

        $name = $event->getTitre();
        $entityManager->remove($event);
        $entityManager->flush();
        $this->addFlash('success', sprintf('Événement "%s" supprimé.', $name));

        return $this->redirectToRoute('app_admin_events');
    }

    #[Route('/materials', name: 'app_admin_materials', methods: ['GET'])]
    public function materials(MaterialRepository $materials): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return $this->render('site/admin-materials.html.twig', [
            'materials' => $materials->findBy([], ['category' => 'ASC', 'name' => 'ASC']),
        ]);
    }

    #[Route('/materials/new', name: 'app_admin_material_new', methods: ['GET', 'POST'])]
    public function newMaterial(Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $material = new Material();
        $form = $this->createForm(MaterialAdminType::class, $material);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->persist($material);
            $entityManager->flush();
            $this->addFlash('success', sprintf('Matériau "%s" créé.', $material->getName()));

            return $this->redirectToRoute('app_admin_materials');
        }

        return $this->render('site/admin-material-new.html.twig', [
            'material' => $material,
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/materials/{id}/edit', name: 'app_admin_material_edit', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function editMaterial(Material $material, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(MaterialAdminType::class, $material);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->flush();
            $this->addFlash('success', sprintf('Matériau "%s" mis à jour.', $material->getName()));

            return $this->redirectToRoute('app_admin_materials');
        }

        return $this->render('site/admin-material-edit.html.twig', [
            'material' => $material,
            'form' => $form,
        ]);
    }

    #[Route('/materials/{id}/delete', name: 'app_admin_material_delete', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function deleteMaterial(Material $material, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('delete_material_' . $material->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Suppression refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_materials');
        }

        $name = $material->getName();
        $entityManager->remove($material);
        $entityManager->flush();
        $this->addFlash('success', sprintf('Matériau "%s" supprimé.', $name));

        return $this->redirectToRoute('app_admin_materials');
    }

    #[Route('/loanable-items', name: 'app_admin_loanable_items', methods: ['GET'])]
    public function loanableItems(LoanableItemRepository $items, LoanRepository $loans): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return $this->render('site/admin-loanable-items.html.twig', [
            'items' => $items->findBy([], ['category' => 'ASC', 'name' => 'ASC']),
            'activeCounts' => $loans->activeCountsByItem(),
        ]);
    }

    #[Route('/loanable-items/new', name: 'app_admin_loanable_item_new', methods: ['GET', 'POST'])]
    public function newLoanableItem(Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $item = new LoanableItem();
        $form = $this->createForm(LoanableItemAdminType::class, $item);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->persist($item);
            $entityManager->flush();
            $this->addFlash('success', sprintf('Objet "%s" créé.', $item->getName()));

            return $this->redirectToRoute('app_admin_loanable_items');
        }

        return $this->render('site/admin-loanable-item-new.html.twig', [
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/loanable-items/{id}/edit', name: 'app_admin_loanable_item_edit', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function editLoanableItem(LoanableItem $item, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(LoanableItemAdminType::class, $item);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->flush();
            $this->addFlash('success', sprintf('Objet "%s" mis à jour.', $item->getName()));

            return $this->redirectToRoute('app_admin_loanable_items');
        }

        return $this->render('site/admin-loanable-item-edit.html.twig', [
            'item' => $item,
            'form' => $form,
        ]);
    }

    #[Route('/loanable-items/{id}/delete', name: 'app_admin_loanable_item_delete', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function deleteLoanableItem(LoanableItem $item, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('delete_loanable_item_' . $item->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Suppression refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_loanable_items');
        }

        $name = $item->getName();
        $entityManager->remove($item);
        $entityManager->flush();
        $this->addFlash('success', sprintf('Objet "%s" supprimé (et ses prêts).', $name));

        return $this->redirectToRoute('app_admin_loanable_items');
    }

    #[Route('/loans', name: 'app_admin_loans', methods: ['GET'])]
    public function loans(LoanRepository $loans): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return $this->render('site/admin-loans.html.twig', [
            'loans' => $loans->findAllSafe(),
        ]);
    }

    #[Route('/loans/new', name: 'app_admin_loan_new', methods: ['GET', 'POST'])]
    public function newLoan(Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $loan = new Loan();
        $form = $this->createForm(LoanAdminType::class, $loan);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            if ($this->getUser() instanceof Utilisateur) {
                $loan->setLentBy($this->getUser());
            }
            $loan->setStatus(Loan::STATUS_OUT);
            $entityManager->persist($loan);
            $entityManager->flush();
            $this->addFlash('success', sprintf('Prêt enregistré pour "%s".', $loan->getBorrowerDisplay()));

            return $this->redirectToRoute('app_admin_loans');
        }

        return $this->render('site/admin-loan-new.html.twig', [
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/loans/{id}/return', name: 'app_admin_loan_return', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function returnLoan(Loan $loan, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('return_loan_' . $loan->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Action refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_loans');
        }

        $loan
            ->setStatus(Loan::STATUS_RETURNED)
            ->setActualReturnDate(new \DateTimeImmutable('today'))
            ->setConditionReturn((string) $request->request->get('conditionReturn') ?: null);
        $entityManager->flush();
        $this->addFlash('success', 'Prêt marqué comme rendu.');

        return $this->redirectToRoute('app_admin_loans');
    }

    #[Route('/maintenance', name: 'app_admin_maintenance', methods: ['GET'])]
    public function maintenance(MaintenanceTaskRepository $tasks): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return $this->render('site/admin-maintenance.html.twig', [
            'tasks' => $tasks->findAllSafe(),
        ]);
    }

    #[Route('/maintenance/new', name: 'app_admin_maintenance_new', methods: ['GET', 'POST'])]
    public function newMaintenance(Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $task = new MaintenanceTask();
        $form = $this->createForm(MaintenanceTaskAdminType::class, $task);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->persist($task);
            $entityManager->flush();
            $this->addFlash('success', sprintf('Tâche de maintenance "%s" créée.', $task->getTitle()));

            return $this->redirectToRoute('app_admin_maintenance');
        }

        return $this->render('site/admin-maintenance-new.html.twig', [
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/maintenance/batch', name: 'app_admin_maintenance_batch', methods: ['GET', 'POST'])]
    public function batchMaintenance(Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(MaintenanceBatchType::class);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $data = $form->getData();
            $count = 0;
            foreach ($data['machines'] as $machine) {
                $task = (new MaintenanceTask())
                    ->setMachine($machine)
                    ->setTitle((string) $data['title'])
                    ->setType((string) $data['type'])
                    ->setDueDate($data['dueDate'] ?? null)
                    ->setRecurrenceDays($data['recurrenceDays'] ?? null)
                    ->setLink($data['link'] ?? null);
                $entityManager->persist($task);
                $count++;
            }
            $entityManager->flush();
            $this->addFlash('success', sprintf('%d tâche(s) de maintenance créée(s).', $count));

            return $this->redirectToRoute('app_admin_maintenance');
        }

        return $this->render('site/admin-maintenance-batch.html.twig', [
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/maintenance/{id}/done', name: 'app_admin_maintenance_done', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function doneMaintenance(MaintenanceTask $task, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('done_maintenance_' . $task->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Action refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_maintenance');
        }

        $today = new \DateTimeImmutable('today');
        $task
            ->setStatus(MaintenanceTask::STATUS_DONE)
            ->setDoneDate($today)
            ->setNotes(($request->request->get('notes') ? (string) $request->request->get('notes') : $task->getNotes()));
        if ($this->getUser() instanceof Utilisateur) {
            $task->setDoneBy($this->getUser());
        }

        // Recurring task: spawn the next occurrence.
        if ($task->getRecurrenceDays() !== null) {
            $next = (new MaintenanceTask())
                ->setMachine($task->getMachine())
                ->setTitle($task->getTitle())
                ->setType($task->getType())
                ->setLink($task->getLink())
                ->setRecurrenceDays($task->getRecurrenceDays())
                ->setDueDate($today->modify('+' . $task->getRecurrenceDays() . ' days'));
            $entityManager->persist($next);
        }

        $entityManager->flush();
        $this->addFlash('success', 'Tâche marquée comme faite.');

        return $this->redirectToRoute('app_admin_maintenance');
    }

    #[Route('/maintenance/{id}/delete', name: 'app_admin_maintenance_delete', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function deleteMaintenance(MaintenanceTask $task, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('delete_maintenance_' . $task->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Suppression refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_maintenance');
        }

        $entityManager->remove($task);
        $entityManager->flush();
        $this->addFlash('success', 'Tâche de maintenance supprimée.');

        return $this->redirectToRoute('app_admin_maintenance');
    }

    #[Route('/access-rfid-logs', name: 'app_admin_access_rfid_logs', methods: ['GET'])]
    public function accessRfidLogs(AccessRfidLogRepository $logs, EntityManagerInterface $entityManager): Response
    {
        $readerColumnsExist = $this->accessRfidLogReaderColumnsExist($entityManager);

        return $this->render('site/admin-access-rfid-logs.html.twig', [
            'logs' => $readerColumnsExist ? $logs->findBy([], ['createdAt' => 'DESC'], 100) : $this->findAccessRfidLogsWithoutReaderColumns($entityManager),
            'readerColumnsExist' => $readerColumnsExist,
        ]);
    }

    #[Route('/rfid-readers', name: 'app_admin_rfid_readers', methods: ['GET'])]
    public function rfidReaders(RfidReaderRepository $readers): Response
    {
        return $this->render('site/admin-rfid-readers.html.twig', [
            'readers' => $readers->findForAdmin(),
        ]);
    }

    #[Route('/rfid-readers/new', name: 'app_admin_rfid_reader_new', methods: ['GET', 'POST'])]
    public function newRfidReader(Request $request, EntityManagerInterface $entityManager, RfidReaderRepository $readers): Response
    {
        $reader = (new RfidReader())->setIsActive(true);
        $form = $this->createForm(RfidReaderAdminType::class, $reader);
        $form->handleRequest($request);

        if ($form->isSubmitted()) {
            $reader->setName(trim($reader->getName()));
            $reader->setReaderToken($this->normalizeReaderToken($reader->getReaderToken()));

            if ($reader->getReaderToken() === '') {
                $reader->setReaderToken($this->generateUniqueReaderToken($reader, $readers));
            }

            $existingReader = $readers->findOneByReaderToken($reader->getReaderToken());
            if ($existingReader !== null) {
                $form->get('readerToken')->addError(new FormError(sprintf('Le readerToken "%s" est déjà utilisé.', $reader->getReaderToken())));
            }
        }

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->persist($reader);
            $entityManager->flush();

            $this->addFlash('success', sprintf('Lecteur RFID "%s" créé. Vous pouvez maintenant copier la configuration exacte pour la Pi.', $reader->getName()));

            return $this->redirectToRoute('app_admin_rfid_reader_edit', ['id' => $reader->getId()]);
        }

        return $this->render('site/admin-rfid-reader-form.html.twig', [
            'reader' => $reader,
            'form' => $form,
            'mode' => 'new',
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/rfid-readers/{id}/edit', name: 'app_admin_rfid_reader_edit', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function editRfidReader(RfidReader $reader, Request $request, EntityManagerInterface $entityManager, RfidReaderRepository $readers): Response
    {
        $form = $this->createForm(RfidReaderAdminType::class, $reader);
        $form->handleRequest($request);

        if ($form->isSubmitted()) {
            $reader->setName(trim($reader->getName()));
            $reader->setReaderToken($this->normalizeReaderToken($reader->getReaderToken()));

            if ($reader->getReaderToken() === '') {
                $reader->setReaderToken($this->generateUniqueReaderToken($reader, $readers));
            }

            $existingReader = $readers->findOneByReaderToken($reader->getReaderToken());
            if ($existingReader !== null && $existingReader->getId() !== $reader->getId()) {
                $form->get('readerToken')->addError(new FormError(sprintf('Le readerToken "%s" est déjà utilisé.', $reader->getReaderToken())));
            }
        }

        if ($form->isSubmitted() && $form->isValid()) {
            $reader->setUpdatedAt(new \DateTimeImmutable());
            $entityManager->flush();

            $this->addFlash('success', sprintf('Lecteur RFID "%s" mis à jour.', $reader->getName()));

            return $this->redirectToRoute('app_admin_rfid_readers');
        }

        return $this->render('site/admin-rfid-reader-form.html.twig', [
            'reader' => $reader,
            'form' => $form,
            'mode' => 'edit',
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/rfid-readers/{id}/delete', name: 'app_admin_rfid_reader_delete', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function deleteRfidReader(RfidReader $reader, Request $request, EntityManagerInterface $entityManager): Response
    {
        if (!$this->isCsrfTokenValid('delete_rfid_reader_' . $reader->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Jeton CSRF invalide. Suppression annulée.');

            return $this->redirectToRoute('app_admin_rfid_readers');
        }

        $readerName = $reader->getName();

        try {
            $entityManager->remove($reader);
            $entityManager->flush();

            $this->addFlash('success', sprintf('Lecteur RFID "%s" supprimé.', $readerName));
        } catch (\Throwable $e) {
            $this->addFlash('error', sprintf('Impossible de supprimer le lecteur RFID "%s". Désactivez-le pour conserver l’historique et empêcher son utilisation.', $readerName));
        }

        return $this->redirectToRoute('app_admin_rfid_readers');
    }

    #[Route('/utilisations', name: 'app_admin_usage_logs', methods: ['GET'])]
    public function usageLogs(Request $request, LogUtilisationRepository $usageLogs): Response
    {
        $filters = $this->extractFilters($request, ['q', 'state', 'source', 'dateFrom', 'dateTo']);
        $filters['state'] = in_array($filters['state'], ['open', 'closed'], true) ? $filters['state'] : 'all';

        $sourceChoices = ['rfid'];
        foreach ($usageLogs->findAdminUsageSources() as $source) {
            $sourceKey = mb_strtolower($source);
            if (!in_array($sourceKey, array_map('mb_strtolower', $sourceChoices), true)) {
                $sourceChoices[] = $source;
            }
        }

        $sourceFilter = mb_strtolower($filters['source']);
        $availableSourceFilters = array_map('mb_strtolower', $sourceChoices);
        $filters['source'] = $sourceFilter !== '' && in_array($sourceFilter, $availableSourceFilters, true) ? $sourceFilter : 'all';

        return $this->render('site/admin-usage-logs.html.twig', [
            'usageLogs' => $usageLogs->findAdminUsageLogs($filters),
            'filters' => $filters,
            'sourceChoices' => $sourceChoices,
        ]);
    }

    #[Route('/admin-dashboard.html', name: 'app_admin_dashboard_legacy_html', methods: ['GET'])]
    public function legacyDashboard(): Response
    {
        return $this->redirectToRoute('app_admin_dashboard');
    }


    /** @return OpeningHour[] */
    private function ensureOpeningHourRows(OpeningHourRepository $openingHours, OpeningHoursProvider $openingHoursProvider, EntityManagerInterface $entityManager): array
    {
        $existingRows = $openingHours->findOrdered();
        $existingByDay = [];
        foreach ($existingRows as $row) {
            $existingByDay[$row->getDayOfWeek()] = $row;
        }

        foreach ($openingHoursProvider->getOpeningHours() as $fallbackRow) {
            if (isset($existingByDay[$fallbackRow->getDayOfWeek()])) {
                continue;
            }

            $row = (new OpeningHour())
                ->setDayOfWeek($fallbackRow->getDayOfWeek())
                ->setLabel($fallbackRow->getLabel())
                ->setIsClosed($fallbackRow->isClosed())
                ->setOpenTime($fallbackRow->getOpenTime())
                ->setCloseTime($fallbackRow->getCloseTime())
                ->setSortOrder($fallbackRow->getSortOrder())
                ->setUpdatedAt(new \DateTimeImmutable());
            $entityManager->persist($row);
            $existingByDay[$row->getDayOfWeek()] = $row;
        }

        ksort($existingByDay);

        return array_values($existingByDay);
    }

    private function parseAdminTime(string $value): ?\DateTime
    {
        $value = trim($value);
        if (!preg_match('/^\d{2}:\d{2}$/', $value)) {
            return null;
        }

        $time = \DateTime::createFromFormat('!H:i', $value);

        return $time instanceof \DateTime ? $time : null;
    }

    /** @param string[] $names */
    private function extractFilters(Request $request, array $names): array
    {
        $filters = [];
        foreach ($names as $name) {
            $filters[$name] = trim((string) $request->query->get($name, ''));
        }

        return $filters;
    }

    /** @return array<string, string> */
    private function buildAdminRoleChoices(RoleRepository $roles): array
    {
        $available = [];
        foreach ($roles->findBy([], ['nom' => 'ASC']) as $role) {
            $securityRole = $this->toSecurityRole($role->getNom());
            if (in_array($securityRole, ['ROLE_USER', 'ROLE_ADMIN', 'ROLE_STAFF'], true)) {
                $available[$securityRole] = $securityRole;
            }
        }

        foreach (['ROLE_USER', 'ROLE_ADMIN'] as $requiredRole) {
            if (!isset($available[$requiredRole]) && $roles->findOneBySecurityRole($requiredRole) !== null) {
                $available[$requiredRole] = $requiredRole;
            }
        }

        return $available;
    }

    private function normalizeCreationData(Creation $creation): void
    {
        $creation
            ->setTitle(trim($creation->getTitle()))
            ->setDescription($this->nullableString($creation->getDescription()))
            ->setAuthorName($this->nullableString($creation->getAuthorName()))
            ->setExternalUrl($this->nullableString($creation->getExternalUrl()))
            ->setPrintDurationMinutes($creation->getPrintDurationMinutes() !== null ? max(0, $creation->getPrintDurationMinutes()) : null);
    }

    /** @param FormInterface<Creation> $form */
    private function handleCreationUploads(Creation $creation, FormInterface $form, SluggerInterface $slugger): bool
    {
        return $this->handleCreationImageUpload($creation, $form, $slugger)
            && $this->handleCreationFileUpload($creation, $form, $slugger);
    }

    /** @param FormInterface<Creation> $form */
    private function handleCreationImageUpload(Creation $creation, FormInterface $form, SluggerInterface $slugger): bool
    {
        $uploadedFile = $form->get('imageUpload')->getData();
        if (!$uploadedFile instanceof UploadedFile) {
            return true;
        }

        $uploadDir = $this->getParameter('kernel.project_dir') . '/public/uploads/creations/images';
        if (!is_dir($uploadDir) && !mkdir($uploadDir, 0775, true) && !is_dir($uploadDir)) {
            $form->get('imageUpload')->addError(new FormError('Impossible de créer le dossier des images de créations.'));
            return false;
        }

        $extension = strtolower($uploadedFile->guessExtension() ?: $uploadedFile->getClientOriginalExtension() ?: 'bin');
        if ($extension === 'jpeg') {
            $extension = 'jpg';
        }

        if (!in_array($extension, ['png', 'jpg', 'webp'], true)) {
            $form->get('imageUpload')->addError(new FormError('Choisissez une image PNG, JPG, JPEG ou WEBP.'));
            return false;
        }

        $fileName = $this->buildUploadedCreationFileName($creation, $slugger, $extension);

        try {
            $uploadedFile->move($uploadDir, $fileName);
        } catch (FileException) {
            $form->get('imageUpload')->addError(new FormError('Impossible de copier l’image de la création.'));
            return false;
        }

        $creation->setImageFilename($fileName);

        return true;
    }

    /** @param FormInterface<Creation> $form */
    private function handleCreationFileUpload(Creation $creation, FormInterface $form, SluggerInterface $slugger): bool
    {
        $uploadedFile = $form->get('fileUpload')->getData();
        if (!$uploadedFile instanceof UploadedFile) {
            return true;
        }

        $extension = strtolower($uploadedFile->getClientOriginalExtension() ?: $uploadedFile->guessExtension() ?: '');
        $allowedExtensions = ['stl', '3mf', 'obj', 'step', 'pdf', 'zip', 'afdesign'];
        if (!in_array($extension, $allowedExtensions, true)) {
            $form->get('fileUpload')->addError(new FormError('Choisissez un fichier STL, 3MF, OBJ, STEP, PDF, ZIP ou AFDESIGN.'));
            return false;
        }

        $uploadDir = $this->getParameter('kernel.project_dir') . '/public/uploads/creations/files';
        if (!is_dir($uploadDir) && !mkdir($uploadDir, 0775, true) && !is_dir($uploadDir)) {
            $form->get('fileUpload')->addError(new FormError('Impossible de créer le dossier des fichiers de créations.'));
            return false;
        }

        $fileName = $this->buildUploadedCreationFileName($creation, $slugger, $extension);

        try {
            $uploadedFile->move($uploadDir, $fileName);
        } catch (FileException) {
            $form->get('fileUpload')->addError(new FormError('Impossible de copier le fichier projet.'));
            return false;
        }

        $creation->setFileFilename($fileName);

        return true;
    }

    private function buildUploadedCreationFileName(Creation $creation, SluggerInterface $slugger, string $extension): string
    {
        $baseName = strtolower($slugger->slug($creation->getTitle())->toString());
        if ($baseName === '') {
            $baseName = 'creation';
        }

        return sprintf('%s-%s.%s', $baseName, bin2hex(random_bytes(3)), $extension);
    }

    private function deleteCreationUploadIfSafe(string $relativeDirectory, ?string $filename): void
    {
        if ($filename === null || $filename === '' || basename($filename) !== $filename || !preg_match('/^[A-Za-z0-9._-]+$/', $filename)) {
            return;
        }

        $projectDir = (string) $this->getParameter('kernel.project_dir');
        $directory = $projectDir . '/' . $relativeDirectory;
        $directoryRealPath = realpath($directory);
        if ($directoryRealPath === false) {
            return;
        }

        $filePath = $directoryRealPath . DIRECTORY_SEPARATOR . $filename;
        $fileRealPath = realpath($filePath);
        if ($fileRealPath === false || !is_file($fileRealPath)) {
            return;
        }

        $directoryPrefix = rtrim($directoryRealPath, DIRECTORY_SEPARATOR) . DIRECTORY_SEPARATOR;
        if (!str_starts_with($fileRealPath, $directoryPrefix)) {
            return;
        }

        @unlink($fileRealPath);
    }

    private function toSecurityRole(string $role): string
    {
        $role = strtoupper(trim($role));

        return str_starts_with($role, 'ROLE_') ? $role : 'ROLE_' . $role;
    }
    /** @return string[] */
    private function linesToArray(mixed $value): array
    {
        $text = trim((string) $value);
        if ($text === '') {
            return [];
        }

        $parts = preg_split('/[\r\n,]+/', $text) ?: [];

        return array_values(array_filter(array_map(static fn (string $item): string => trim($item), $parts), static fn (string $item): bool => $item !== ''));
    }

    private function nullableString(mixed $value): ?string
    {
        $value = trim((string) $value);

        return $value === '' ? null : $value;
    }

    private function slugify(string $value): string
    {
        $value = iconv('UTF-8', 'ASCII//TRANSLIT//IGNORE', $value) ?: $value;
        $value = strtolower($value);
        $value = preg_replace('/[^a-z0-9]+/', '-', $value) ?: '';
        $value = trim($value, '-');

        return $value !== '' ? $value : 'machine';
    }


    private function accessRfidLogReaderColumnsExist(EntityManagerInterface $entityManager): bool
    {
        try {
            $connection = $entityManager->getConnection();
            $database = (string) $connection->fetchOne('SELECT DATABASE()');

            $count = (int) $connection->fetchOne(
                "SELECT COUNT(*) FROM information_schema.COLUMNS WHERE TABLE_SCHEMA = :schema AND TABLE_NAME = 'ACCESS_RFID_LOG' AND COLUMN_NAME IN ('readerId', 'readerToken')",
                ['schema' => $database]
            );

            return $count === 2;
        } catch (\Throwable $e) {
            return false;
        }
    }

    /** @return array<int, array<string, mixed>> */
    private function findAccessRfidLogsWithoutReaderColumns(EntityManagerInterface $entityManager): array
    {
        try {
            return $entityManager->getConnection()->fetchAllAssociative(<<<'SQL'
                SELECT
                    l.id,
                    l.badgeUid,
                    l.authorized,
                    l.status,
                    l.reason,
                    l.message,
                    l.color,
                    l.createdAt,
                    u.id AS user_id,
                    u.firstName AS user_firstName,
                    u.lastName AS user_lastName,
                    u.username AS user_username,
                    m.id AS machine_id,
                    m.nom AS machine_nom
                FROM ACCESS_RFID_LOG l
                LEFT JOIN UTILISATEUR u ON u.id = l.userId
                LEFT JOIN MACHINE m ON m.id = l.machineId
                ORDER BY l.createdAt DESC
                LIMIT 100
            SQL);
        } catch (\Throwable $e) {
            return [];
        }
    }

    private function normalizeReaderToken(string $readerToken): string
    {
        $readerToken = strtolower(trim($readerToken));
        $readerToken = preg_replace('/[^a-z0-9_-]+/', '-', $readerToken) ?: '';
        $readerToken = trim($readerToken, '-_');

        return $readerToken;
    }

    private function generateUniqueReaderToken(RfidReader $reader, RfidReaderRepository $readers): string
    {
        $baseSource = $reader->getName();
        if ($baseSource === '' && $reader->getMachine() !== null) {
            $baseSource = $reader->getMachine()->getMachineToken();
        }

        $base = $this->slugify($baseSource);
        if (!str_starts_with($base, 'reader-')) {
            $base = 'reader-' . $base;
        }

        $token = substr($base, 0, 120);
        $suffix = 2;

        while (($existing = $readers->findOneByReaderToken($token)) !== null && $existing->getId() !== $reader->getId()) {
            $shortSuffix = '-' . $suffix;
            $token = substr($base, 0, 120 - strlen($shortSuffix)) . $shortSuffix;
            $suffix++;
        }

        return $token;
    }

    private function extractMachineLevel(Machine $machine): ?int
    {
        if (preg_match('/(\d+)/', $machine->getLevelSlug(), $matches) === 1) {
            $level = (int) $matches[1];

            return $level >= 1 && $level <= 3 ? $level : null;
        }

        if (preg_match('/(\d+)/', $machine->getLevelLabel(), $matches) === 1) {
            $level = (int) $matches[1];

            return $level >= 1 && $level <= 3 ? $level : null;
        }

        return null;
    }

    private function buildRecentActivities(
        AccessRfidLogRepository $rfidLogs,
        ReservationRepository $reservations,
        ProgressionRepository $progressions,
        LogUtilisationRepository $usageLogs,
    ): array {
        $activities = [];

        try {
            foreach ($rfidLogs->findBy([], ['createdAt' => 'DESC'], 5) as $log) {
                $user = $log->getUtilisateur();
                $machine = $log->getMachine();
                $activities[] = [
                    'type' => 'rfid',
                    'title' => $log->isAuthorized() ? 'Accès RFID autorisé' : 'Accès RFID refusé',
                    'message' => sprintf('%s · %s · badge %s', $user?->getDisplayName() ?? 'Utilisateur inconnu', $machine?->getNom() ?? 'Machine inconnue', $log->getBadgeUid()),
                    'date' => $log->getCreatedAt(),
                ];
            }
        } catch (\Throwable $e) {
            // La migration readerId/readerToken peut ne pas encore être appliquée.
        }

        foreach ($reservations->findBy([], ['created' => 'DESC'], 5) as $reservation) {
            $activities[] = [
                'type' => 'reservation',
                'title' => 'Réservation créée',
                'message' => sprintf('%s a réservé %s', $reservation->getUtilisateur()?->getDisplayName() ?? 'Utilisateur inconnu', $reservation->getReservableLabel() ?: 'une ressource'),
                'date' => $reservation->getCreated(),
            ];
        }

        foreach ($progressions->findBy([], ['dateDebut' => 'DESC'], 5) as $progression) {
            $activities[] = [
                'type' => 'formation',
                'title' => $progression->isCompleted() ? 'Formation terminée' : 'Formation commencée',
                'message' => sprintf('%s · %s · score %d', $progression->getUtilisateur()?->getDisplayName() ?? 'Utilisateur inconnu', $progression->getFormation()?->getTitre() ?? 'Formation inconnue', $progression->getScore()),
                'date' => $progression->getDateEnd() ?? $progression->getDateDebut(),
            ];
        }

        foreach ($usageLogs->findBy([], ['createdAt' => 'DESC'], 5) as $usageLog) {
            $activities[] = [
                'type' => 'usage',
                'title' => 'Utilisation machine',
                'message' => sprintf('%s · %s · %s', $usageLog->getUtilisateur()?->getDisplayName() ?? 'Utilisateur inconnu', $usageLog->getMachine()?->getNom() ?? 'Machine inconnue', $usageLog->getSource() ?? 'source non renseignée'),
                'date' => $usageLog->getCreatedAt(),
            ];
        }

        usort($activities, static fn (array $a, array $b): int => $b['date'] <=> $a['date']);

        return array_slice($activities, 0, 8);
    }

    private function buildFormationProgressionStats(ProgressionRepository $progressions): array
    {
        $stats = [];
        foreach ($progressions->findAll() as $progression) {
            $formation = $progression->getFormation();
            if (!$formation || !$formation->getId() || TrainingQualificationService::isInternalCategory($formation->getCategorie())) {
                continue;
            }

            $id = $formation->getId();
            $stats[$id] ??= ['total' => 0, 'completed' => 0, 'incomplete' => 0];
            $stats[$id]['total']++;
            $progression->isCompleted() ? $stats[$id]['completed']++ : $stats[$id]['incomplete']++;
        }

        return $stats;
    }

    private function buildUserProgressionStats(ProgressionRepository $progressions): array
    {
        $stats = [];
        foreach ($progressions->findAll() as $progression) {
            $formation = $progression->getFormation();
            if ($formation !== null && TrainingQualificationService::isInternalCategory($formation->getCategorie())) {
                continue;
            }

            $user = $progression->getUtilisateur();
            if ($user && $user->getId()) {
                $stats[$user->getId()] = ($stats[$user->getId()] ?? 0) + 1;
            }
        }

        return $stats;
    }
}
