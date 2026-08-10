<?php

namespace App\Controller;

use App\Feature\FeatureAdvice;
use App\Feature\FirstRun;
use App\Feature\SetupHealth;
use App\Feature\SiteFeatureRegistry;
use App\Feature\FeatureWorkspaceRegistry;
use App\Http\MissingPageLog;
use App\Image\ImageNormalizer;
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
use App\Event\EventRegistrationService;
use App\Event\EventShareQr;
use App\Event\TicketLinker;
use App\Mail\MailLog;
use App\Mail\Mailer;
use App\Mail\MailSettings;
use App\Mail\ReminderLog;
use App\Mail\ReminderSettings;
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
use App\Repository\EventRegistrationRepository;
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
use App\Reservation\Policy\BookingPolicy;
use App\Reservation\Policy\BookingPolicyRepository;
use App\Reservation\Policy\BookingTier;
use App\Reservation\ReservableResolver;
use App\Reservation\ReservableType;
use App\Reservation\ReservationMailer;
use App\Reporting\ReportScope;
use App\Reporting\ReportingRegistry;
use App\Repository\RfidReaderRepository;
use App\Repository\RoleRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Repository\UtilisateurRepository;
use App\Repository\VenueRepository;
use App\Feature\SiteFeatureService;
use App\Service\SiteSettingService;
use App\Service\OpeningHoursProvider;
use App\Service\TrainingQualificationService;
use App\Service\ThemeManager;
use App\Entity\HomepageSectionVisibility;
use App\Repository\HomepageSectionVisibilityRepository;
use App\Service\HomepageVisibilityService;
use App\UsageRights\UsageRightsService;
use App\UsageRights\UsageCapabilityRegistry;
use App\UsageRights\UsagePackageRepository;
use App\Reservation\LabClock;
use App\Venue\VenueContext;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\Form\FormInterface;
use Symfony\Component\Form\FormError;
use Symfony\Component\HttpFoundation\File\Exception\FileException;
use Symfony\Component\HttpFoundation\File\UploadedFile;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\HttpFoundation\StreamedResponse;
use Symfony\Component\String\Slugger\SluggerInterface;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Contracts\Translation\TranslatorInterface;
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
        FirstRun $firstRun,
    ): Response {
        $recentActivities = $this->buildRecentActivities($rfidLogs, $reservations, $progressions, $usageLogs);

        return $this->render('site/admin-dashboard.html.twig', [
            // The only thing that ever points at the wizard. Nothing redirects.
            'showFirstRun' => $firstRun->isFresh(),
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

    #[Route('/themes', name: 'app_admin_themes', methods: ['GET', 'POST'])]
    public function themes(Request $request, ThemeManager $themes): Response
    {
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_themes', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_admin_themes');
            }

            $action = $request->request->getString('action');
            try {
                if (in_array($action, ['save', 'preview'], true)) {
                    $themes->saveDraft($request->request->all('theme'));
                } elseif ($action === 'publish') {
                    $themes->saveDraft($request->request->all('theme'));
                    $themes->publish();
                } elseif ($action === 'discard') {
                    $themes->discardDraft();
                }
                $this->addFlash('success', $action === 'publish' ? 'Thème publié.' : ($action === 'discard' ? 'Brouillon remplacé par la version publiée.' : 'Brouillon enregistré.'));
            } catch (\InvalidArgumentException $exception) {
                $this->addFlash('error', $exception->getMessage());
            }

            return $this->redirectToRoute('app_admin_themes', $action === 'preview' ? ['preview' => 1] : []);
        }

        return $this->render('site/admin-themes.html.twig', [
            'draft' => $themes->draft(),
            'published' => $themes->published(),
            'preview' => $request->query->getBoolean('preview'),
        ]);
    }

    #[Route('/machines', name: 'app_admin_machines', methods: ['GET'])]
    #[Route('/machines.html', name: 'app_admin_machines_scoped_html', methods: ['GET'])]
    #[Route('/admin-machines.html', name: 'app_admin_machines_double_legacy_html', methods: ['GET'])]
    public function machines(Request $request, MachineRepository $machines, BadgeRepository $badges, VenueContext $venueContext): Response
    {
        $filters = $this->extractFilters($request, ['q', 'statut', 'niveau', 'badge', 'category']);
        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);
        $allMachines = $machines->findBy($context['selected'] === null ? [] : ['venue' => $context['selected']], ['nom' => 'ASC']);

        return $this->render('site/admin-machines.html.twig', [
            'machines' => $machines->findForAdminFilters($filters, $context['selected']),
            'machineCategoryTiles' => $this->machineCategoryTiles($allMachines, $filters),
            'machineStatusTiles' => $this->machineStatusTiles($allMachines, $filters),
            'machineListQuery' => array_filter(['statut' => $filters['statut'], 'category' => $filters['category']], static fn (string $value): bool => $value !== ''),
            'machineCount' => count($allMachines),
            'filters' => $filters,
            'availableBadges' => $badges->findBy([], ['nom' => 'ASC']),
            'venueContext' => $context,
        ]);
    }

    #[Route('/machines/categories', name: 'app_admin_machine_categories', methods: ['GET'])]
    public function machineCategories(Request $request, MachineRepository $machines, VenueContext $venueContext): Response
    {
        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);
        $rows = $machines->findBy($context['selected'] === null ? [] : ['venue' => $context['selected']], ['categoryLabel' => 'ASC']);

        return $this->render('site/admin-machine-taxonomy.html.twig', [
            'title' => 'Catégories de machines',
            'description' => 'Les catégories sont gérées depuis chaque machine et regroupées ici sans copie de catalogue.',
            'rows' => $this->machineTaxonomyRows($rows, static fn (Machine $machine): string => $machine->getCategoryLabel()),
            'venueContext' => $context,
            'routeName' => 'app_admin_machine_categories',
            'filterKey' => 'category',
        ]);
    }

    #[Route('/machines/models', name: 'app_admin_machine_models', methods: ['GET'])]
    public function machineModels(Request $request, MachineRepository $machines, VenueContext $venueContext): Response
    {
        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);
        $rows = $machines->findBy($context['selected'] === null ? [] : ['venue' => $context['selected']], ['manufacturer' => 'ASC', 'model' => 'ASC']);

        return $this->render('site/admin-machine-taxonomy.html.twig', [
            'title' => 'Modèles et marques',
            'description' => 'Les références sont gérées depuis chaque machine et regroupées ici.',
            'rows' => $this->machineTaxonomyRows($rows, static fn (Machine $machine): string => trim(($machine->getManufacturer() ?? '') . ' ' . ($machine->getModel() ?? '')) ?: 'Non renseigné'),
            'venueContext' => $context,
            'routeName' => 'app_admin_machine_models',
            'filterKey' => 'model',
        ]);
    }

    #[Route('/machines/new', name: 'app_admin_machine_new', methods: ['GET', 'POST'])]
    public function newMachine(Request $request, EntityManagerInterface $entityManager, BadgeRepository $badges, MachineRepository $machines, VenueRepository $venues): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $machine = new Machine();
        $machine->setVenue($this->requireDefaultVenue($venues));
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
        $type = ReservableType::tryParse($request->query->getString('reservableType'));
        if ($type === null) {
            return $this->redirectToRoute('app_admin_reservations', array_filter([
                'reservableType' => ReservableType::Machine->value,
                'q' => $request->query->getString('q'),
                'statut' => $request->query->getString('statut'),
                'dateFrom' => $request->query->getString('dateFrom'),
                'dateTo' => $request->query->getString('dateTo'),
            ], static fn (string $value): bool => $value !== ''));
        }
        $filters = $this->extractFilters($request, ['q', 'statut', 'dateFrom', 'dateTo', 'reservableType']);
        $rows = $reservations->findForAdminFilters($filters);
        $reservables->warm($rows);

        return $this->render('site/admin-reservations.html.twig', [
            'reservations' => $rows,
            'filters' => $filters,
        ]);
    }


    #[Route('/horaires', name: 'app_admin_opening_hours', methods: ['GET', 'POST'])]
    public function openingHours(Request $request, OpeningHourRepository $openingHours, OpeningHoursProvider $openingHoursProvider, VenueContext $venueContextService, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        // ⚠️ S131. This used to be `$venues->findDefault()`, so the screen could only
        // ever edit the default venue's week — while the table has been keyed
        // `(venueId, dayOfWeek)` since S106. A second sub-venue was creatable from
        // S129 and could never be given opening hours. `single()` is the shared
        // resolution for venue-scoped *editing*; it never answers "all", because
        // there is no row to write for "all".
        $venueContext = $venueContextService->single($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);
        $venue = $venueContext['selected'];
        $rows = $this->ensureOpeningHourRows($openingHours, $openingHoursProvider, $venue, $entityManager);
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

                // Keep the venue in the URL: redirecting bare would bounce the operator
                // back to their default venue after editing another one's week.
                return $this->redirectToRoute('app_admin_opening_hours', ['location' => $venueContext['location']]);
            }
        }

        return $this->render('site/admin-opening-hours.html.twig', [
            'openingHours' => $rows,
            'errors' => $errors,
            'venueContext' => $venueContext,
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
        UsageRightsService $usageRights,
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
            'usageRightsSummary' => $usageRights->overview($user),
        ]);
    }

    #[Route('/utilisateurs/{id}/person-type', name: 'app_admin_user_person_type', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function updatePersonType(
        int $id,
        Request $request,
        UtilisateurRepository $users,
        RoleRepository $roles,
        ReservationRepository $reservations,
        ReservationMailer $reservationMails,
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
            $stranded = $reservations->findUpcomingActiveForReservable(ReservableType::User, $id);
            $reservations->cancelUpcomingForReservable(ReservableType::User, $id);
            $reservationMails->cancelledBatch($stranded);
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
    public function newCreation(Request $request, EntityManagerInterface $entityManager, SluggerInterface $slugger, ImageNormalizer $images): Response
    {
        $creation = new Creation();
        $form = $this->createForm(CreationAdminType::class, $creation);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $this->normalizeCreationData($creation);

            if (!$this->handleCreationUploads($creation, $form, $slugger, $images)) {
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
    public function editCreation(Creation $creation, Request $request, EntityManagerInterface $entityManager, SluggerInterface $slugger, ImageNormalizer $images): Response
    {
        $form = $this->createForm(CreationAdminType::class, $creation);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $this->normalizeCreationData($creation);

            if (!$this->handleCreationUploads($creation, $form, $slugger, $images)) {
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
    public function settings(
        Request $request,
        SiteSettingService $siteSettings,
        RoleRepository $roles,
        UsagePackageRepository $usagePackages,
        UsageCapabilityRegistry $usageCapabilities,
        TranslatorInterface $translator,
    ): Response
    {
        $availableLocales = ['fr' => 'Français', 'en' => 'English', 'es' => 'Español', 'de' => 'Deutsch', 'it' => 'Italiano'];
        $capabilityKeys = array_map(static fn ($capability): string => $capability->key, $usageCapabilities->all());
        $rightsReadiness = $usagePackages->readiness($capabilityKeys, new \DateTimeImmutable('now', new \DateTimeZone($siteSettings->getTimezone())));

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
                $siteSettings->setVocabulary(
                    (string) $request->request->get('org_name'),
                    (string) $request->request->get('venue_label'),
                );
                $siteSettings->setDevelopmentMode($request->request->getBoolean('development_mode'));
                $enableRights = $request->request->getBoolean('usage_rights_enforced');
                if ($enableRights && !$siteSettings->isUsageRightsEnforced()) {
                    if ($rightsReadiness['packages'] < 1 || $rightsReadiness['members'] < 1) {
                        $enableRights = false;
                        $this->addFlash('error', $translator->trans('usage_rights.settings_not_ready'));
                    } elseif (!$request->request->getBoolean('usage_rights_confirm_enable')) {
                        $enableRights = false;
                        $this->addFlash('error', $translator->trans('usage_rights.settings_confirmation_required'));
                    }
                }
                $siteSettings->setUsageRightsEnforced($enableRights);
                $siteSettings->setLabRules(
                    (string) $request->request->get('lab_rules_html'),
                    (string) $request->request->get('lab_rules_pdf_url'),
                );
                // Rejected rather than silently ignored: a typo here would send every
                // displayed time back to the fallback zone with nothing on screen
                // saying so.
                $timezone = trim((string) $request->request->get('timezone'));
                if (SiteSettingService::isValidTimezone($timezone)) {
                    $siteSettings->setTimezone($timezone);
                } elseif ($timezone !== '') {
                    $this->addFlash('error', sprintf('Fuseau horaire inconnu : « %s ». Les autres réglages ont été enregistrés.', $timezone));
                }
                // ⚠️ Read as "the roles that were ticked", so unticking every box
                // stores "nobody but the booking's owner" rather than falling back to
                // the default — that is a choice an operator is entitled to make.
                $siteSettings->setBookingIdentityRoles(
                    array_map('strval', (array) $request->request->all('booking_identity_roles')),
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
            'orgName' => $siteSettings->getOrgName(),
            'venueLabel' => $siteSettings->getVenueLabel(),
            'labRulesHtml' => $siteSettings->getLabRulesHtml(),
            'labRulesPdfUrl' => $siteSettings->getLabRulesPdfUrl(),
            'icalFeedToken' => $siteSettings->getIcalFeedToken(),
            'bookingIdentityRoles' => $siteSettings->getBookingIdentityRoles(),
            'timezone' => $siteSettings->getTimezone(),
            'availableTimezones' => \DateTimeZone::listIdentifiers(),
            'developmentMode' => $siteSettings->isDevelopmentMode(),
            'usageRightsEnforced' => $siteSettings->isUsageRightsEnforced(),
            'usageRightsReadiness' => $rightsReadiness,
            'usageRightsCapabilities' => $usageCapabilities->all(),
            // The operator's own role list, not a hardcoded set: a deployment that
            // added "formateur" must be able to tick it here. Mapped through the same
            // helper the firewall will later be asked about, so the two cannot drift.
            'assignableRoles' => array_map(
                static fn (Role $role): array => [
                    'label' => $role->getNom(),
                    'securityRole' => Utilisateur::securityRoleFor($role->getNom()),
                ],
                $roles->findBy([], ['nom' => 'ASC']),
            ),
        ]);
    }

    /**
     * The screen an operator uses to decide what this deployment *is*.
     *
     * One list. There used to be two — "capabilities" over "modules", with a
     * registry, a derivation, a deviation model and an Advanced disclosure
     * joining them — but the catalogue was one-to-one, so all of that bought a
     * second vocabulary for the same set of choices. Collapsed into site
     * features: the thing the operator reads and the thing that gates routes are
     * now the same thing.
     */
    #[Route('/features', name: 'app_admin_features', methods: ['GET', 'POST'])]
    public function features(Request $request, SiteFeatureService $features, SiteFeatureRegistry $registry, FeatureAdvice $advice): Response
    {
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_features', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_admin_features');
            }

            $checked = (array) $request->request->all('features');
            foreach ($registry->keys() as $key) {
                $features->setEnabled($key, in_array($key, $checked, true));
            }
            $this->addFlash('success', 'Fonctionnalités mises à jour.');

            return $this->redirectToRoute('app_admin_features');
        }

        return $this->render('site/admin-features.html.twig', [
            'registry' => $registry,
            'grouped' => $features->groupedState(),
            'state' => $features->all(),
            'advice' => $advice->byFeature(),
        ]);
    }

    /**
     * The design tokens, and what they actually measure.
     *
     * A design system that documents itself in prose drifts from the CSS the
     * first time somebody edits one and not the other. This page reads the
     * *computed* values off `:root` and measures the contrast in the browser, so
     * it cannot claim a ratio the stylesheet does not deliver — and it picks up a
     * portal's own accent, which is the case a static swatch sheet would miss.
     */
    #[Route('/design', name: 'app_admin_design', methods: ['GET'])]
    public function design(): Response
    {
        return $this->render('site/admin-design.html.twig');
    }

    /**
     * Read-only S103 matrix. It exposes the target metadata contract without
     * changing live routes, grants or enforcement.
     */
    #[Route('/design/workspaces', name: 'app_admin_workspace_vision', methods: ['GET'])]
    public function workspaceVision(FeatureWorkspaceRegistry $registry): Response
    {
        return $this->render('site/admin-workspace-vision.html.twig', [
            'workspaces' => $registry->all(),
            'themeContract' => $registry->themeContract(),
        ]);
    }

    /**
     * Read-only product prototype for the next access/responsibility phases. It
     * reads the real role/category catalogues, but deliberately persists nothing: the
     * target model must be reviewed before a migration changes live access.
     */
    #[Route('/design/droits-quotas', name: 'app_admin_usage_rights_vision', methods: ['GET'])]
    public function usageRightsVision(
        RoleRepository $roles,
        MachineRepository $machines,
    ): Response {
        $categories = [];
        foreach ($machines->findBy([], ['categoryLabel' => 'ASC']) as $machine) {
            $slug = $machine->getStoredCategorySlug() ?: 'uncategorized';
            $categories[$slug] ??= [
                'slug' => $slug,
                'label' => $machine->getStoredCategoryLabel(),
                'count' => 0,
            ];
            ++$categories[$slug]['count'];
        }

        return $this->render('site/admin-usage-rights-vision.html.twig', [
            'roleRows' => $roles->findBy([], ['nom' => 'ASC']),
            'categories' => array_values($categories),
        ]);
    }

    /** Current-domain map: a venue is the physical booking scope. */
    #[Route('/design/structure', name: 'app_admin_structure_vision', methods: ['GET'])]
    public function structureVision(
        MachineRepository $machines,
        PlaceRepository $places,
        SiteSettingService $settings,
    ): Response {
        $machineRows = $machines->findAll();
        $categories = [];
        foreach ($machineRows as $machine) {
            $slug = $machine->getStoredCategorySlug() ?: 'uncategorized';
            $categories[$slug] ??= [
                'slug' => $slug,
                'label' => $machine->getStoredCategoryLabel(),
                'count' => 0,
            ];
            ++$categories[$slug]['count'];
        }

        return $this->render('site/admin-structure-vision.html.twig', [
            'categories' => array_values($categories),
            'machineCount' => count($machineRows),
            'placeCount' => $places->count([]),
            'orgName' => $settings->getOrgName(),
            'venueLabel' => $settings->getVenueLabel(),
            'address' => $settings->getLabAddress(),
            'timezone' => $settings->getTimezone(),
        ]);
    }

    /**
     * The questions a new install needs answered, in one place.
     *
     * ⚠️ **Nothing redirects here.** S25 flagged a global redirect-to-wizard
     * interceptor as a real hazard on a live install, and it is: every route that
     * ever forwards somewhere is a route that can strand somebody. This is
     * reachable from a card on the dashboard and from the sidebar, and that is
     * all — so the worst a bug here can do is render a page badly, never take a
     * working site hostage.
     *
     * It asks **only for settings that already have readers**, which is why there
     * is no "organisation logo" or "opening hours" step: a wizard that collects
     * something nothing reads is the same broken promise as a settings screen
     * that does. Features get a link rather than a copy of their screen — one
     * place to edit them stays one place.
     */
    #[Route('/wizard', name: 'app_admin_wizard', methods: ['GET', 'POST'])]
    public function wizard(Request $request, SiteSettingService $siteSettings, FirstRun $firstRun): Response
    {
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_wizard', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_admin_wizard');
            }

            // Skipping is a real answer, not a failure: an operator who already
            // knows the settings screens should be able to say so once.
            if ($request->request->get('action') !== 'skip') {
                $siteSettings->setVocabulary(
                    (string) $request->request->get('org_name'),
                    (string) $request->request->get('venue_label'),
                );
                $siteSettings->setPublicBaseUrl((string) $request->request->get('public_base_url'));
                $siteSettings->setLabAddress((string) $request->request->get('lab_address'));

                $locale = (string) $request->request->get('default_locale');
                if (in_array($locale, ['fr', 'en', 'de', 'es', 'it'], true)) {
                    $siteSettings->setDefaultLocale($locale);
                }

                // Asked here because the box runs UTC and the fallback is Europe/Paris:
                // an install anywhere else would otherwise show every recorded
                // timestamp in the wrong zone until somebody thought to look.
                $timezone = trim((string) $request->request->get('timezone'));
                if (SiteSettingService::isValidTimezone($timezone)) {
                    $siteSettings->setTimezone($timezone);
                }
            }

            $firstRun->markCompleted();
            $this->addFlash('success', $request->request->get('action') === 'skip'
                ? 'Configuration initiale passée. Tout reste modifiable dans les réglages.'
                : 'Configuration initiale enregistrée.');

            return $this->redirectToRoute('app_admin_setup');
        }

        return $this->render('site/admin-wizard.html.twig', [
            'orgName' => $siteSettings->getOrgName(),
            'venueLabel' => $siteSettings->getVenueLabel(),
            'publicBaseUrl' => $siteSettings->getPublicBaseUrl(),
            'labAddress' => $siteSettings->getLabAddress(),
            'currentLocale' => $siteSettings->getDefaultLocale(),
            'timezone' => $siteSettings->getTimezone(),
            'availableTimezones' => \DateTimeZone::listIdentifiers(),
            'completedAt' => $firstRun->completedAt(),
        ]);
    }

    /**
     * What is not yet configured, and what each gap actually costs.
     *
     * Read-only on purpose: every fix belongs on the screen that owns the
     * setting, so this links there rather than growing a second place to edit
     * the same thing.
     */
    #[Route('/setup', name: 'app_admin_setup', methods: ['GET'])]
    public function setup(SetupHealth $health): Response
    {
        return $this->render('site/admin-setup.html.twig', [
            'checks' => $health->checks(),
            'counts' => $health->counts(),
            'healthy' => $health->isHealthy(),
        ]);
    }

    /**
     * The URLs people ask for and do not get.
     *
     * The screen exists to separate two things a server log shows identically: a
     * **broken link** in the operator's own content, which is a mistake, and
     * **somebody reaching for a feature that is switched off**, which is the
     * gating model working exactly as designed — and is demand, answered on the
     * feature screen rather than in the templates.
     *
     * Pruning and clearing are POSTs behind a CSRF token: the log is the evidence
     * you fix links from, so wiping it must be a deliberate click.
     */
    #[Route('/missing-pages', name: 'app_admin_missing_pages', methods: ['GET', 'POST'])]
    public function missingPages(Request $request, MissingPageLog $log): Response
    {
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_missing_pages', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_admin_missing_pages');
            }

            if ($request->request->get('action') === 'clear') {
                $this->addFlash('success', sprintf('Journal vidé (%d adresse(s) oubliée(s)).', $log->clear()));
            } else {
                $this->addFlash('success', sprintf('%d adresse(s) inactive(s) depuis 90 jours oubliée(s).', $log->prune()));
            }

            return $this->redirectToRoute('app_admin_missing_pages');
        }

        return $this->render('site/admin-missing-pages.html.twig', [
            'misses' => $log->top(),
            'summary' => $log->summary(),
        ]);
    }

    /**
     * Portals: the list, and creating one.
     *
     * ⚠️ **The default portal is not a tenant, it is the global scope.** It owns
     * no rows of its own — `portalId = 0` means "the default portal's value" —
     * so it is shown here but has no override editor, and offering one would
     * show an admin a set of empty fields that silently do nothing.
     */
    #[Route('/portals', name: 'app_admin_portals', methods: ['GET'])]
    public function portals(): Response
    {
        $this->addFlash('info', 'Les portails ont été retirés : cette installation a un seul site.');
        return $this->redirectToRoute('app_admin_structure_vision', [], Response::HTTP_MOVED_PERMANENTLY);

        /*
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_portals', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_admin_portals');
            }

            try {
                if ($request->request->get('action') === 'delete') {
                    $id = $request->request->getInt('id');
                    $name = $portals->find($id)?->name ?? '';
                    $removed = $portals->delete($id);
                    $this->addFlash('success', sprintf('Portail « %s » supprimé, avec %d réglage(s) qui lui appartenaient.', $name, $removed));
                } else {
                    $id = $portals->create(
                        trim((string) $request->request->get('name')),
                        trim((string) $request->request->get('slug')),
                        (string) $request->request->get('hostname'),
                    );
                    $this->addFlash('success', 'Portail créé. Choisissez maintenant ce qu\'il propose.');

                    return $this->redirectToRoute('app_admin_portal_edit', ['id' => $id]);
                }
            } catch (\Throwable $e) {
                $this->addFlash('error', $e->getMessage());
            }

            return $this->redirectToRoute('app_admin_portals');
        }

        return $this->render('site/admin-portals.html.twig', [
            'portals' => $portals->all(),
        ]);
        */
    }

    #[Route('/portals/consolidation', name: 'app_admin_portal_consolidation', methods: ['GET'])]
    public function portalConsolidation(): Response
    {
        return $this->redirectToRoute('app_admin_structure_vision', [], Response::HTTP_MOVED_PERMANENTLY);
    }

    /**
     * One portal: its identity, what it offers, and what it overrides.
     *
     * Features and settings are **tri-state** here, and that is the point of the
     * screen: a portal says "on", "off" or *nothing at all*, and saying nothing
     * is the default. Plain checkboxes would write an explicit row for every
     * feature the first time anyone pressed Save, quietly cutting the portal off
     * from every later change to the site-wide switches.
     */
    #[Route('/portals/{id<\d+>}', name: 'app_admin_portal_edit', methods: ['GET'])]
    public function portalEdit(int $id): Response
    {
        return $this->redirectToRoute('app_admin_structure_vision', [], Response::HTTP_MOVED_PERMANENTLY);

        /*
        $portal = $portals->find($id);
        if ($portal === null) {
            throw $this->createNotFoundException();
        }

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_portal_edit', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_admin_portal_edit', ['id' => $id]);
            }

            try {
                if ($request->request->get('action') === 'delete') {
                    if ($portal->isDefault) {
                        throw new \LogicException('Le portail par défaut ne peut pas être supprimé.');
                    }
                    $name = $portal->name;
                    $removed = $portals->delete($id);
                    $this->addFlash('success', sprintf('Portail « %s » supprimé, avec %d réglage(s) qui lui appartenaient.', $name, $removed));

                    return $this->redirectToRoute('app_admin_portals');
                }
                // Overrides first, because they are where a typo actually happens —
                // a mistyped colour must not leave the portal renamed and then
                // report an error, which reads as "it half worked".
                if (!$portal->isDefault) {
                    // Submitted settings array.
                    $settings = (array) $request->request->all('settings');
                    $overrides->save($id, $settings);
                }

                $portals->update(
                    $id,
                    trim((string) $request->request->get('name')),
                    trim((string) $request->request->get('slug')),
                    (string) $request->request->get('hostname'),
                );

                if (!$portal->isDefault) {
                    $submitted = (array) $request->request->all('features');
                    // Root features only — the ones the screen actually shows. Walking
                    // every key would silently delete an add-on's row on a save that
                    // never offered it, and add-ons already follow their parent.
                    foreach (array_keys($registry->roots()) as $key) {
                        $choice = (string) ($submitted[$key] ?? 'inherit');
                        $features->setEnabledForScope($id, $key, match ($choice) {
                            'on' => true,
                            'off' => false,
                            default => null,
                        });
                    }
                }

                $this->addFlash('success', 'Portail enregistré.');
            } catch (\Throwable $e) {
                $this->addFlash('error', $e->getMessage());
            }

            return $this->redirectToRoute('app_admin_portal_edit', ['id' => $id]);
        }

        return $this->render('site/admin-portal-edit.html.twig', [
            'portal' => $portal,
            'registry' => $registry,
            // What the site says, so "inherit" can show what it actually means.
            'global' => $features->all(),
            'scoped' => $portal->isDefault ? [] : $features->stateForScope($id),
            'overrides' => $portal->isDefault ? [] : $overrides->forPortal($id),
        ]);
        */
    }

    /** `/admin/modules` and `/admin/capabilities` were both this screen. */
    #[Route('/modules', name: 'app_admin_modules', methods: ['GET'])]
    #[Route('/capabilities', name: 'app_admin_capabilities', methods: ['GET'])]
    public function featuresLegacy(): Response
    {
        return $this->redirectToRoute('app_admin_features', [], Response::HTTP_MOVED_PERMANENTLY);
    }


    /**
     * Sender account for outgoing mail, plus the audit log of what went out.
     * Nothing here sends anything on its own — it's the prerequisite screen the
     * rest of the notification work plugs into.
     */
    #[Route('/emails', name: 'app_admin_emails', methods: ['GET', 'POST'])]
    public function emails(Request $request, MailSettings $mailSettings, MailLog $mailLog, Mailer $mailer, SiteFeatureService $modules, ReminderSettings $reminderSettings, ReminderLog $reminderLog, SiteSettingService $siteSettings): Response
    {
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_emails', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_admin_emails');
            }

            if ($request->request->get('action') === 'reminders') {
                $enabled = [];
                foreach (ReminderSettings::KINDS as $kind) {
                    $enabled[$kind] = $request->request->getBoolean('reminder_' . $kind);
                }

                $reminderSettings->save(
                    $enabled,
                    $request->request->getInt('reminder_booking_lead_hours', ReminderSettings::DEFAULT_BOOKING_LEAD_HOURS),
                    $request->request->getInt('reminder_loan_lead_days', ReminderSettings::DEFAULT_LOAN_LEAD_DAYS),
                    $request->request->getInt('reminder_event_lead_hours', ReminderSettings::DEFAULT_EVENT_LEAD_HOURS),
                );
                $this->addFlash('success', 'Rappels programmés enregistrés.');

                return $this->redirectToRoute('app_admin_emails');
            }

            if ($request->request->get('action') === 'test') {
                $recipient = trim((string) $request->request->get('test_recipient'));
                if ($recipient === '') {
                    $recipient = $this->getUser() instanceof Utilisateur ? $this->getUser()->getEmail() : '';
                }

                $error = $mailer->sendNow($recipient, null, 'test', ['sent_at' => (new \DateTimeImmutable())->format('d/m/Y H:i')]);
                if ($error === null) {
                    $this->addFlash('success', sprintf('E-mail de test envoyé à %s.', $recipient));
                } else {
                    $this->addFlash('error', 'Échec de l\'envoi : ' . $error);
                }

                return $this->redirectToRoute('app_admin_emails');
            }

            // The form shows the DSN with its password masked; posting it back unchanged
            // must not overwrite the stored password with dots.
            $dsn = (string) $request->request->get('mail_transport_dsn');
            if (MailSettings::isMasked($dsn)) {
                $dsn = $mailSettings->getTransportDsn();
            }

            $mailSettings->save(
                $dsn,
                (string) $request->request->get('mail_from_address'),
                (string) $request->request->get('mail_from_name'),
                (string) $request->request->get('mail_reply_to'),
            );
            $siteSettings->setPublicBaseUrl((string) $request->request->get('public_base_url'));
            // A pause is a deliberate, visible state — not a side effect of
            // saving the form, so it is its own checkbox.
            $mailSettings->setPaused($request->request->getBoolean('mail_paused'));
            $this->addFlash('success', 'Compte d\'envoi enregistré.');

            return $this->redirectToRoute('app_admin_emails');
        }

        return $this->render('site/admin-emails.html.twig', [
            'mailPaused' => $mailSettings->isPaused(),
            'configured' => $mailSettings->isConfigured(),
            'transportDsn' => $mailSettings->getMaskedTransportDsn(),
            'fromAddress' => $mailSettings->getFromAddress(),
            'fromName' => $mailSettings->getFromName(),
            'replyTo' => $mailSettings->getReplyTo(),
            'logs' => $mailLog->recent(50),
            'counts' => $mailLog->statusCounts(),
            'queueSize' => $mailLog->pendingQueueSize(),
            'reminders' => $reminderSettings->all(),
            'reminderBookingLeadHours' => $reminderSettings->getBookingLeadHours(),
            'reminderLoanLeadDays' => $reminderSettings->getLoanLeadDays(),
            'reminderEventLeadHours' => $reminderSettings->getEventLeadHours(),
            'reminderCounts' => $reminderLog->countsByKind(),
            'publicBaseUrl' => $siteSettings->getPublicBaseUrl(),
        ]);
    }

    /**
     * Booking quotas, as a grid of resource kind × tier.
     *
     * Every cell is optional and blank means "no limit", so an untouched screen
     * is a lab with no quotas — which is exactly the state it ships in. Saving a
     * scope with every box empty deletes its row rather than storing nine nulls,
     * so "has a policy" and "is actually limited" never drift apart.
     */
    #[Route('/quotas-reservation', name: 'app_admin_booking_policies', methods: ['GET', 'POST'])]
    public function bookingPolicies(Request $request, BookingPolicyRepository $policies): Response
    {
        $selectedType = ReservableType::tryParse($request->query->getString('reservableType')) ?? ReservableType::Machine;
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_booking_policies', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_admin_booking_policies', ['reservableType' => $selectedType->value]);
            }

            foreach (BookingTier::ordered() as $tier) {
                    $values = [];
                    foreach (BookingPolicy::FIELDS as $field) {
                        $raw = trim((string) $request->request->get(sprintf('%s_%s_%s', $selectedType->value, $tier->value, $field), ''));
                        $values[$field] = ($raw === '' || !is_numeric($raw)) ? null : max(0, (int) $raw);
                    }

                    $policies->save(new BookingPolicy(
                        $selectedType,
                        $tier,
                        $values['minNoticeMinutes'],
                        $values['maxHorizonDays'],
                        $values['slotIncrementMinutes'],
                        $values['minDurationMinutes'],
                        $values['maxDurationMinutes'],
                        $values['maxActiveReservations'],
                        $values['maxPerDay'],
                        $values['maxPerWeek'],
                        $values['bufferMinutes'],
                        $values['cancellationNoticeMinutes'],
                    ));
            }

            $this->addFlash('success', 'Quotas de réservation enregistrés.');

            return $this->redirectToRoute('app_admin_booking_policies', ['reservableType' => $selectedType->value]);
        }

        $configured = $policies->allByScope();
        $grid = [];
        foreach (BookingTier::ordered() as $tier) {
                $policy = $configured[$selectedType->value . ':' . $tier->value] ?? BookingPolicy::unrestricted($selectedType, $tier);
                $grid[$tier->value] = [
                    'tierLabel' => $tier->label(),
                    'values' => $policy->toFormValues(),
                    'restricted' => !$policy->isUnrestricted(),
                ];
        }

        return $this->render('site/admin-booking-policies.html.twig', [
            'grid' => $grid,
            'selectedType' => $selectedType,
            'workspaceKey' => match ($selectedType) { ReservableType::Machine => 'equipment', ReservableType::Place => 'spaces', ReservableType::User => 'users' },
            'fields' => BookingPolicy::FIELDS,
        ]);
    }

    #[Route('/reporting/{workspace}', name: 'app_admin_reporting', requirements: ['workspace' => 'equipment|spaces'], methods: ['GET'])]
    public function reporting(string $workspace, Request $request, ReportingRegistry $reporting, VenueContext $venueContext): Response
    {
        [$scope, $from, $to, $context] = $this->reportScope($workspace, $request, $venueContext);

        return $this->render('site/admin-reporting.html.twig', [
            'report' => $reporting->forWorkspace($workspace)->report($scope),
            'workspaceKey' => $workspace,
            'venueContext' => $context,
            'from' => $from,
            'to' => $to,
        ]);
    }

    #[Route('/reporting/{workspace}/export.csv', name: 'app_admin_reporting_export', requirements: ['workspace' => 'equipment|spaces'], methods: ['GET'])]
    public function reportingExport(string $workspace, Request $request, ReportingRegistry $reporting, VenueContext $venueContext): Response
    {
        [$scope] = $this->reportScope($workspace, $request, $venueContext);
        $adapter = $reporting->forWorkspace($workspace);

        $response = new StreamedResponse(static function () use ($adapter, $scope): void {
            $output = fopen('php://output', 'wb');
            if ($output === false) { return; }
            fputcsv($output, ['date', 'reservations', 'minutes']);
            foreach ($adapter->export($scope) as $row) { fputcsv($output, $row); }
            fclose($output);
        });
        $response->headers->set('Content-Type', 'text/csv; charset=UTF-8');
        $response->headers->set('Content-Disposition', sprintf('attachment; filename="fabos-%s-report.csv"', $workspace));

        return $response;
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
        $this->deleteCreationUploadIfSafe('public/uploads/creations/thumbs', $imageFilename);
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

    #[Route('/badges/{id}/archive', name: 'app_admin_badge_archive', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function archiveBadge(Badge $badge, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('archive_badge_' . $badge->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Archivage refusé : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_badge_edit', ['id' => $badge->getId()]);
        }

        $badge->archive();
        $entityManager->flush();
        $this->addFlash('success', sprintf('Badge « %s » archivé ; ses attributions restent dans le journal.', $badge->getNom()));

        return $this->redirectToRoute('app_admin_badges');
    }

    #[Route('/badges/{id}/restore', name: 'app_admin_badge_restore', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function restoreBadge(Badge $badge, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');
        if (!$this->isCsrfTokenValid('restore_badge_' . $badge->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Restauration refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_admin_badge_edit', ['id' => $badge->getId()]);
        }

        $badge->restore();
        $entityManager->flush();
        $this->addFlash('success', sprintf('Badge « %s » restauré.', $badge->getNom()));

        return $this->redirectToRoute('app_admin_badges');
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
    public function addLabPagePhoto(LabPage $page, Request $request, EntityManagerInterface $entityManager, SluggerInterface $slugger, ImageNormalizer $images): Response
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


        // ⚠️ **Capped at the door (S80).** What a camera hands over is not what a
        // web page needs: the two posters this rule was written for were 23 MB
        // each. `capUploaded` also uprights the picture and can change the
        // container — a PNG with no alpha is a photograph in a format that
        // cannot compress photographs — so the filename is built from what it
        // RETURNS, never from what the browser sent.
        $extension = $images->capUploaded($uploadedFile->getPathname(), $extension);

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
    public function places(Request $request, PlaceRepository $places, VenueContext $venueContext): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);

        $filters = $this->extractFilters($request, ['q', 'category', 'manager', 'department']);
        $allRows = $places->findBy($context['selected'] === null ? [] : ['venue' => $context['selected']], ['nom' => 'ASC']);
        $rows = array_values(array_filter($allRows, static function (Place $place) use ($filters): bool {
            $haystack = mb_strtolower(implode(' ', array_filter([$place->getNom(), $place->getLocalisation(), $place->getCategory(), $place->getManager(), $place->getDepartment()])));
            return ($filters['q'] === '' || str_contains($haystack, mb_strtolower($filters['q'])))
                && ($filters['category'] === '' || $place->getCategory() === $filters['category'])
                && ($filters['manager'] === '' || $place->getManager() === $filters['manager'])
                && ($filters['department'] === '' || $place->getDepartment() === $filters['department']);
        }));

        return $this->render('site/admin-places.html.twig', [
            'places' => $rows,
            'venueContext' => $context,
            'filters' => $filters,
            'placeCategoryTiles' => $this->categoryTiles($allRows, static fn (Place $place): ?string => $place->getCategory()),
            'placeManagers' => array_values(array_unique(array_filter(array_map(static fn (Place $place): ?string => $place->getManager(), $allRows)))),
            'placeDepartments' => array_values(array_unique(array_filter(array_map(static fn (Place $place): ?string => $place->getDepartment(), $allRows)))),
        ]);
    }

    #[Route('/places/new', name: 'app_admin_place_new', methods: ['GET', 'POST'])]
    public function newPlace(Request $request, EntityManagerInterface $entityManager, VenueRepository $venues): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $place = new Place();
        $place->setVenue($this->requireDefaultVenue($venues));
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
    public function deletePlace(Place $place, Request $request, EntityManagerInterface $entityManager, ReservationRepository $reservations, ReservationMailer $reservationMails): Response
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
        $stranded = $reservations->findUpcomingActiveForReservable(ReservableType::Place, $place->getId());
        $cancelled = $reservations->cancelUpcomingForReservable(ReservableType::Place, $place->getId());
        $reservationMails->cancelledBatch($stranded);

        $entityManager->remove($place);
        $entityManager->flush();
        $this->addFlash('success', $cancelled > 0
            ? sprintf('Espace "%s" supprimé (%d réservation(s) à venir annulée(s)).', $name, $cancelled)
            : sprintf('Espace "%s" supprimé.', $name));

        return $this->redirectToRoute('app_admin_places');
    }

    #[Route('/events', name: 'app_admin_events', methods: ['GET'])]
    public function events(Request $request, EventRepository $events, VenueContext $venueContext, LabClock $labClock): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);

        $query = mb_strtolower(mb_substr(trim($request->query->getString('q')), 0, 80));
        $period = in_array($request->query->getString('type'), ['upcoming', 'past'], true) ? $request->query->getString('type') : '';
        // ⚠️ Convention B: `Event` dates are human-entered wall-clock, stored as the
        // digits the operator typed. `new DateTimeImmutable()` is the server instant in
        // UTC, so comparing the two was off by the lab's offset and an event changed tab
        // up to two hours early or late around midnight. `storedFormOf()` gives "now" in
        // the same digits the column holds, which is what these rows actually mean.
        $now = $labClock->storedFormOf($labClock->now());
        $rows = array_values(array_filter(
            $events->findBy($context['selected'] === null ? [] : ['venue' => $context['selected']], ['dateDebut' => 'DESC']),
            static function (Event $event) use ($query, $period, $now): bool {
                $end = $event->getDateFin() ?? $event->getDateDebut();
                return ($query === '' || str_contains(mb_strtolower($event->getTitre() . ' ' . ($event->getLieu() ?? '')), $query))
                    && ($period === '' || ($period === 'past' ? $end < $now : $end >= $now));
            },
        ));

        return $this->render('site/admin-events.html.twig', [
            'events' => $rows,
            'venueContext' => $context,
            'filters' => ['q' => $query, 'type' => $period],
            // The template compares against this, never Twig's `date()`, which would
            // reintroduce the server-UTC bug in the tile counts.
            'now' => $now,
        ]);
    }

    #[Route('/events/new', name: 'app_admin_event_new', methods: ['GET', 'POST'])]
    public function newEvent(Request $request, EntityManagerInterface $entityManager, VenueRepository $venues): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $event = new Event();
        $form = $this->createForm(EventAdminType::class, $event);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            // ⚠️ This forced `requireDefaultVenue()` on every onsite event AFTER the form
            // had bound, silently discarding the sub-venue the operator picked. Offsite
            // events genuinely have none; onsite ones keep the choice, falling back to the
            // default only when none was made (S133).
            if (!$event->isOnsite()) {
                $event->setVenue(null);
            } elseif ($event->getVenue() === null) {
                $event->setVenue($this->requireDefaultVenue($venues));
            }
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
    public function editEvent(Event $event, Request $request, EntityManagerInterface $entityManager, EventShareQr $qr, VenueRepository $venues): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(EventAdminType::class, $event);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            // ⚠️ This forced `requireDefaultVenue()` on every onsite event AFTER the form
            // had bound, silently discarding the sub-venue the operator picked. Offsite
            // events genuinely have none; onsite ones keep the choice, falling back to the
            // default only when none was made (S133).
            if (!$event->isOnsite()) {
                $event->setVenue(null);
            } elseif ($event->getVenue() === null) {
                $event->setVenue($this->requireDefaultVenue($venues));
            }
            $entityManager->flush();
            $this->addFlash('success', sprintf('Événement "%s" mis à jour.', $event->getTitre()));

            return $this->redirectToRoute('app_admin_events');
        }

        return $this->render('site/admin-event-edit.html.twig', [
            'event' => $event,
            'form' => $form,
            'shareUrl' => $qr->publicUrl($event),
            'shareQr' => $qr->svgDataUri($event),
        ]);
    }

    /**
     * Who is coming, and the quick way to call the whole thing off.
     *
     * Cancelling lives here rather than behind the delete button on purpose:
     * deleting an event throws away the list of people who were counting on it,
     * which is exactly who needs telling. Calling off keeps them and mails them.
     */
    #[Route('/events/{id}/inscriptions', name: 'app_admin_event_registrations', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function eventRegistrations(
        Event $event,
        Request $request,
        EventRegistrationRepository $registrations,
        EventRegistrationService $registrationService,
        EntityManagerInterface $entityManager,
        TicketLinker $tickets,
    ): Response {
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_event_calloff_' . $event->getId(), (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_admin_event_registrations', ['id' => $event->getId()]);
            }

            $action = (string) $request->request->get('action');

            // Door desk: toggle attendance. Deliberately not a one-way switch —
            // the commonest thing that happens at a door is tapping the wrong row.
            if ($action === 'checkin' || $action === 'undo_checkin') {
                $row = $registrations->find($request->request->getInt('registration'));

                if ($row === null || $row->getEvent()?->getId() !== $event->getId()) {
                    $this->addFlash('error', 'Inscription introuvable pour cet événement.');
                } elseif ($action === 'checkin' && !$row->isCheckInEligible()) {
                    $this->addFlash('error', 'Seules les personnes inscrites (hors liste d\'attente) peuvent être pointées.');
                } else {
                    $staff = $this->getUser() instanceof Utilisateur ? $this->getUser() : null;
                    $action === 'checkin' ? $row->checkIn($staff) : $row->undoCheckIn();
                    $entityManager->flush();
                }

                return $this->redirectToRoute('app_admin_event_registrations', ['id' => $event->getId()]);
            }

            // "They phoned to cancel." Goes through the same service as a
            // self-cancellation, so the seat is freed, the next person on the
            // waitlist is promoted, and both of them are mailed — none of which
            // would happen if this just flipped a status.
            if ($action === 'unregister') {
                $row = $registrations->find($request->request->getInt('registration'));

                if ($row === null || $row->getEvent()?->getId() !== $event->getId()) {
                    $this->addFlash('error', 'Inscription introuvable pour cet événement.');
                } else {
                    $result = $registrationService->cancel($row);
                    $this->addFlash(
                        $result->ok ? 'success' : 'error',
                        $result->ok
                            ? sprintf('Inscription de %s annulée. La personne a été prévenue par e-mail.', $row->getDisplayName())
                            : (string) $result->message,
                    );
                }

                return $this->redirectToRoute('app_admin_event_registrations', ['id' => $event->getId()]);
            }

            $notified = $registrationService->callOff($event, (string) $request->request->get('reason'));
            $this->addFlash('success', sprintf(
                'Événement annulé. %d personne(s) prévenue(s) par e-mail.',
                $notified,
            ));

            return $this->redirectToRoute('app_admin_event_registrations', ['id' => $event->getId()]);
        }

        $rows = $registrations->findForEvent($event);

        return $this->render('site/admin-event-registrations.html.twig', [
            'event' => $event,
            'registrations' => $rows,
            'seatsTaken' => $registrations->countSeatsTaken($event),
            'waitlistCount' => $registrations->countWaitlisted($event),
            'seatsRemaining' => $registrationService->seatsRemaining($event),
            'checkedInCount' => $registrations->countCheckedIn($event),
            // Signed per registration: the ticket route rejects an unsigned path,
            // so these cannot be built with path() in the template.
            'ticketUrls' => array_reduce(
                $rows,
                static function (array $carry, $reg) use ($tickets): array {
                    if ($reg->getId() !== null && $reg->isCheckInEligible()) {
                        $carry[$reg->getId()] = $tickets->ticketUrl($reg);
                    }

                    return $carry;
                },
                [],
            ),
        ]);
    }

    /**
     * The event's poster QR as a print-resolution PNG download.
     *
     * Streamed rather than written to disk: it is fully derived from the event
     * id and the public URL, so storing it would only create a file to keep in
     * step with a URL that can change.
     */
    #[Route('/events/{id}/qr.png', name: 'app_admin_event_qr', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function eventQr(Event $event, EventShareQr $qr): Response
    {
        $png = $qr->pngBytes($event);

        if ($png === null) {
            throw $this->createNotFoundException("Aucune adresse publique n'est configurée : impossible de générer le QR code.");
        }

        $slug = preg_replace('/[^a-z0-9]+/', '-', mb_strtolower($event->getTitre())) ?: 'evenement';

        return new Response($png, Response::HTTP_OK, [
            'Content-Type' => 'image/png',
            'Content-Disposition' => sprintf('attachment; filename="qr-%s.png"', trim($slug, '-')),
        ]);
    }

    /**
     * Poster artwork for an event. Same shape as the lab-page photo upload:
     * extension allow-list, random filename, and the old file removed on replace
     * so posters don't silently accumulate on disk.
     */
    #[Route('/events/{id}/poster', name: 'app_admin_event_poster', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function eventPoster(Event $event, Request $request, EntityManagerInterface $entityManager, ImageNormalizer $images): Response
    {
        $back = fn (): Response => $this->redirectToRoute('app_admin_event_edit', ['id' => $event->getId()]);

        if (!$this->isCsrfTokenValid('event_poster_' . $event->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Action refusée : token CSRF invalide.');

            return $back();
        }

        $uploadDir = $this->getParameter('kernel.project_dir') . '/public/uploads/events';
        $previous = $event->getPosterFilename();

        if ($request->request->get('action') === 'remove') {
            $event->setPosterFilename(null);
            $entityManager->flush();
            $this->deletePosterFile($uploadDir, $previous);
            $this->addFlash('success', 'Affiche retirée.');

            return $back();
        }

        $uploadedFile = $request->files->get('poster');
        if (!$uploadedFile instanceof UploadedFile) {
            $this->addFlash('error', 'Choisissez une image.');

            return $back();
        }

        $extension = strtolower($uploadedFile->guessExtension() ?: $uploadedFile->getClientOriginalExtension() ?: 'bin');
        if ($extension === 'jpeg') {
            $extension = 'jpg';
        }

        if (!in_array($extension, ['png', 'jpg', 'webp'], true)) {
            $this->addFlash('error', 'Choisissez une image PNG, JPG ou WEBP.');

            return $back();
        }

        if (!is_dir($uploadDir) && !mkdir($uploadDir, 0775, true) && !is_dir($uploadDir)) {
            $this->addFlash('error', 'Impossible de créer le dossier des affiches.');

            return $back();
        }


        // ⚠️ **Capped at the door (S80).** What a camera hands over is not what a
        // web page needs: the two posters this rule was written for were 23 MB
        // each. `capUploaded` also uprights the picture and can change the
        // container — a PNG with no alpha is a photograph in a format that
        // cannot compress photographs — so the filename is built from what it
        // RETURNS, never from what the browser sent.
        $extension = $images->capUploaded($uploadedFile->getPathname(), $extension);

        $fileName = sprintf('event-%d-%s.%s', $event->getId(), bin2hex(random_bytes(6)), $extension);

        try {
            $uploadedFile->move($uploadDir, $fileName);
        } catch (FileException) {
            $this->addFlash('error', 'Impossible de copier l\'image.');

            return $back();
        }

        $event->setPosterFilename($fileName);
        $entityManager->flush();

        // Only after the new one is safely in place and recorded.
        $this->deletePosterFile($uploadDir, $previous);
        $this->addFlash('success', 'Affiche mise à jour.');

        return $back();
    }

    private function deletePosterFile(string $uploadDir, ?string $fileName): void
    {
        if ($fileName === null || $fileName === '' || str_contains($fileName, '/') || str_contains($fileName, '..')) {
            return;
        }

        $path = $uploadDir . '/' . $fileName;
        if (is_file($path)) {
            @unlink($path);
        }
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

        $materialRows = $materials->findBy([], ['category' => 'ASC', 'name' => 'ASC']);

        return $this->render('site/admin-materials.html.twig', [
            'materials' => $materialRows,
            'materialCategoryTiles' => $this->categoryTiles($materialRows, static fn (Material $material): ?string => $material->getCategory()),
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
    public function loanableItems(Request $request, LoanableItemRepository $items, LoanRepository $loans, VenueContext $venueContext): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        // ⚠️ S131. `LoanableItem` has carried a non-nullable `venueId` since S107, but
        // this list ignored it — so on a two-venue install every object appeared on
        // both, and the sub-venue control was missing from the one Prêts screen that
        // could honour it. Same contract as machines/places/events: aggregate by
        // default, narrow on an explicit and valid `?location=`.
        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);
        $itemRows = $items->findBy($context['selected'] === null ? [] : ['venue' => $context['selected']], ['category' => 'ASC', 'name' => 'ASC']);

        return $this->render('site/admin-loanable-items.html.twig', [
            'items' => $itemRows,
            'venueContext' => $context,
            'itemCategoryTiles' => $this->categoryTiles($itemRows, static fn (LoanableItem $item): ?string => $item->getCategory()),
            'activeCounts' => $loans->activeCountsByItem(),
        ]);
    }

    #[Route('/loanable-items/new', name: 'app_admin_loanable_item_new', methods: ['GET', 'POST'])]
    public function newLoanableItem(Request $request, EntityManagerInterface $entityManager, VenueRepository $venues): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $item = new LoanableItem();
        $item->setVenue($this->requireDefaultVenue($venues));
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

    #[Route('/maintenance/{id}/edit', name: 'app_admin_maintenance_edit', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function editMaintenance(MaintenanceTask $task, Request $request, EntityManagerInterface $entityManager): Response
    {
        $form = $this->createForm(MaintenanceTaskAdminType::class, $task);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->flush();
            $this->addFlash('success', sprintf('Tâche de maintenance « %s » mise à jour.', $task->getTitle()));

            return $this->redirectToRoute('app_admin_maintenance_edit', ['id' => $task->getId()]);
        }

        return $this->render('site/admin-maintenance-new.html.twig', [
            'form' => $form,
            'task' => $task,
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
    private function ensureOpeningHourRows(OpeningHourRepository $openingHours, OpeningHoursProvider $openingHoursProvider, \App\Entity\Venue $venue, EntityManagerInterface $entityManager): array
    {
        $existingRows = $openingHours->findOrdered($venue);
        $existingByDay = [];
        foreach ($existingRows as $row) {
            $existingByDay[$row->getDayOfWeek()] = $row;
        }

        foreach ($openingHoursProvider->getOpeningHours() as $fallbackRow) {
            if (isset($existingByDay[$fallbackRow->getDayOfWeek()])) {
                continue;
            }

            $row = (new OpeningHour())
                ->setVenue($venue)
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

    private function requireDefaultVenue(VenueRepository $venues): \App\Entity\Venue
    {
        return $venues->findDefault()
            ?? throw new \LogicException('Le sous-lieu par défaut est introuvable.');
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

    /** @param Machine[] $machines @return array<int, array{label: string, count: int, category: string, query: array<string, string>, active: bool}> */
    private function machineStatusTiles(array $machines, array $filters): array
    {
        $baseQuery = array_filter(['q' => $filters['q'], 'category' => $filters['category']], static fn (string $value): bool => $value !== '');
        $counts = [];
        foreach ($machines as $machine) {
            $label = trim($machine->getStatut());
            if ($label === '') {
                continue;
            }
            $category = str_replace([' ', '_'], '-', mb_strtolower($label));
            $counts[$category] ??= [
                'label' => $label,
                'count' => 0,
                'category' => $category,
                'query' => $baseQuery + ['statut' => $label],
                'active' => $filters['statut'] === $label,
            ];
            $counts[$category]['count']++;
        }

        uasort($counts, static fn (array $left, array $right): int => strnatcasecmp($left['label'], $right['label']));

        array_unshift($counts, [
            'label' => 'admin_list.all',
            'label_is_key' => true,
            'count' => count($machines),
            'category' => 'all',
            'query' => $baseQuery,
            'active' => $filters['statut'] === '',
        ]);

        return array_values($counts);
    }

    /** @param Machine[] $machines @return array<int, array{label: string, count: int, query: array<string, string>, active: bool}> */
    private function machineCategoryTiles(array $machines, array $filters): array
    {
        $baseQuery = array_filter(['q' => $filters['q'], 'statut' => $filters['statut']], static fn (string $value): bool => $value !== '');
        $counts = [];
        foreach ($machines as $machine) {
            $slug = $machine->getCategorySlug();
            $label = trim($machine->getCategoryLabel());
            if ($slug === '' || $label === '') {
                continue;
            }

            $counts[$slug] ??= [
                'label' => $label,
                'count' => 0,
                'icon_slug' => $slug,
                'query' => $baseQuery + ['category' => $slug],
                'active' => $filters['category'] === $slug,
            ];
            $counts[$slug]['count']++;
        }

        uasort($counts, static fn (array $left, array $right): int => strnatcasecmp($left['label'], $right['label']));

        array_unshift($counts, [
            'label' => 'admin_list.all',
            'label_is_key' => true,
            'count' => count($machines),
            'query' => $baseQuery,
            'active' => $filters['category'] === '',
        ]);

        return array_values($counts);
    }

    /** @param array<int, object> $items @return array<int, array{label: string, count: int, category: string}> */
    private function categoryTiles(array $items, callable $categoryForItem): array
    {
        $counts = [];
        foreach ($items as $item) {
            $label = trim((string) $categoryForItem($item));
            if ($label === '') {
                continue;
            }
            $category = str_replace([' ', '_'], '-', mb_strtolower($label));
            $counts[$category] ??= ['label' => $label, 'count' => 0, 'category' => $category, 'query' => ['category' => $label]];
            $counts[$category]['count']++;
        }

        uasort($counts, static fn (array $left, array $right): int => strnatcasecmp($left['label'], $right['label']));

        return array_values($counts);
    }

    /** @return array{ReportScope, string, string, array<string, mixed>} */
    private function reportScope(string $workspace, Request $request, VenueContext $venueContext): array
    {
        $today = new \DateTimeImmutable('today');
        $from = \DateTimeImmutable::createFromFormat('!Y-m-d', $request->query->getString('from')) ?: $today->modify('-29 days');
        $to = \DateTimeImmutable::createFromFormat('!Y-m-d', $request->query->getString('to')) ?: $today;
        if ($to < $from) { [$from, $to] = [$to, $from]; }
        if ($from->diff($to)->days > 366) { $from = $to->modify('-366 days'); }
        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);

        return [
            new ReportScope($workspace, $from, $to->modify('+1 day'), $context['selected']?->getId()),
            $from->format('Y-m-d'),
            $to->format('Y-m-d'),
            $context,
        ];
    }

    /** @param Machine[] $machines @return list<array{label: string, count: int, machines: list<Machine>}> */
    private function machineTaxonomyRows(array $machines, callable $labelForMachine): array
    {
        $rows = [];
        foreach ($machines as $machine) {
            $label = trim((string) $labelForMachine($machine)) ?: 'Non renseigné';
            $rows[$label] ??= ['label' => $label, 'count' => 0, 'machines' => []];
            $rows[$label]['count']++;
            $rows[$label]['machines'][] = $machine;
        }
        uksort($rows, 'strnatcasecmp');

        return array_values($rows);
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
    private function handleCreationUploads(Creation $creation, FormInterface $form, SluggerInterface $slugger, ImageNormalizer $images): bool
    {
        return $this->handleCreationImageUpload($creation, $form, $slugger, $images)
            && $this->handleCreationFileUpload($creation, $form, $slugger);
    }

    /** @param FormInterface<Creation> $form */
    private function handleCreationImageUpload(Creation $creation, FormInterface $form, SluggerInterface $slugger, ImageNormalizer $images): bool
    {
        $uploadedFile = $form->get('imageUpload')->getData();
        if (!$uploadedFile instanceof UploadedFile) {
            return true;
        }

        $extension = strtolower($uploadedFile->guessExtension() ?: $uploadedFile->getClientOriginalExtension() ?: 'bin');
        if ($extension === 'jpeg') {
            $extension = 'jpg';
        }

        if (!in_array($extension, ['png', 'jpg', 'webp'], true)) {
            $form->get('imageUpload')->addError(new FormError('Choisissez une image PNG, JPG, JPEG ou WEBP.'));
            return false;
        }

        if (!$images->isAvailable()) {
            $form->get('imageUpload')->addError(new FormError('Optimisation impossible : activez l’extension PHP GD sur le serveur.'));
            return false;
        }

        // ⚠️ The extension validated above describes what was UPLOADED; the file
        // is named for what gets WRITTEN, which is WebP wherever GD supports it.
        // `storeCreationImage()` supersedes `capUploaded()` on this path — it
        // uprights, caps and re-encodes in the same pass, and also writes the
        // thumbnail the templates ask for.
        $projectDir = (string) $this->getParameter('kernel.project_dir');
        $uploadDir = $projectDir . '/public/uploads/creations/images';
        $thumbDir = $projectDir . '/public/uploads/creations/thumbs';
        $fileName = $this->buildUploadedCreationFileName($creation, $slugger, $images->outputExtension());

        if (!$images->storeCreationImage($uploadedFile->getPathname(), $uploadDir, $thumbDir, $fileName)) {
            $form->get('imageUpload')->addError(new FormError('Impossible d’optimiser l’image de la création. Vérifiez que le fichier est une image valide.'));
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
