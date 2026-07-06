<?php

namespace App\Controller;

use App\Entity\Badge;
use App\Entity\Formation;
use App\Entity\Machine;
use App\Entity\MachineBadge;
use App\Entity\OpeningHour;
use App\Entity\Utilisateur;
use App\Entity\UtilisateurRole;
use App\Form\BadgeAdminType;
use App\Form\FormationAdminType;
use App\Form\MachineAdminType;
use App\Form\UserAdminType;
use App\Repository\AccessRfidLogRepository;
use App\Repository\BadgeRepository;
use App\Repository\FormationRepository;
use App\Repository\LogUtilisationRepository;
use App\Repository\MachineRepository;
use App\Repository\OpeningHourRepository;
use App\Repository\ProgressionRepository;
use App\Repository\ReservationRepository;
use App\Repository\RoleRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Repository\UtilisateurRepository;
use App\Service\OpeningHoursProvider;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\Form\FormError;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
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
                'formations' => $formations->count([]),
                'reservations' => $reservations->count([]),
                'rfidLogs' => $rfidLogs->count([]),
                'badges' => $badges->count([]),
                'completedFormations' => $progressions->count(['completed' => true]),
            ],
            'recentActivities' => $recentActivities,
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
    public function reservations(Request $request, ReservationRepository $reservations): Response
    {
        $filters = $this->extractFilters($request, ['q', 'statut', 'dateFrom', 'dateTo']);

        return $this->render('site/admin-reservations.html.twig', [
            'reservations' => $reservations->findForAdminFilters($filters),
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
    ): Response {
        $user = $users->find($id);
        if (!$user) {
            throw $this->createNotFoundException('Utilisateur introuvable');
        }

        return $this->render('site/admin-utilisateur-detail.html.twig', [
            'user' => $user,
            'rfidLogs' => $logs->findBy(['utilisateur' => $user], ['createdAt' => 'DESC']),
            'progressions' => $progressions->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']),
            'reservations' => $reservations->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']),
            'usageLogs' => $usageLogs->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']),
        ]);
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

    #[Route('/access-rfid-logs', name: 'app_admin_access_rfid_logs', methods: ['GET'])]
    public function accessRfidLogs(AccessRfidLogRepository $logs): Response
    {
        return $this->render('site/admin-access-rfid-logs.html.twig', [
            'logs' => $logs->findBy([], ['createdAt' => 'DESC'], 100),
        ]);
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

        foreach ($reservations->findBy([], ['created' => 'DESC'], 5) as $reservation) {
            $activities[] = [
                'type' => 'reservation',
                'title' => 'Réservation créée',
                'message' => sprintf('%s a réservé %s', $reservation->getUtilisateur()?->getDisplayName() ?? 'Utilisateur inconnu', $reservation->getMachine()?->getNom() ?? 'Machine inconnue'),
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
            if (!$formation || !$formation->getId()) {
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
            $user = $progression->getUtilisateur();
            if ($user && $user->getId()) {
                $stats[$user->getId()] = ($stats[$user->getId()] ?? 0) + 1;
            }
        }

        return $stats;
    }
}
