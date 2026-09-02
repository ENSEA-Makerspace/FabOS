<?php

namespace App\Controller;

use App\Feature\FeatureAdvice;
use App\Feature\FeatureSurfaces;
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
use App\Calendar\EventSeries;
use App\Entity\EventCategory;
use App\Entity\MachineCategory;
use App\Entity\MaintenanceTask;
use App\Entity\Material;
use App\Entity\Place;
use App\Entity\MachineBadge;
use App\Entity\OpeningHour;
use App\Entity\Progression;
use App\Entity\RfidReader;
use App\Account\AccountAnonymiser;
use App\Account\AccountGuard;
use App\Entity\Utilisateur;
use App\Event\EventRegistrationService;
use App\Event\EventShareQr;
use App\Event\TicketLinker;
use App\Mail\MailLog;
use App\Mail\Mailer;
use App\Mail\MailSettings;
use App\Mail\NotificationCategory;
use App\Mail\ReminderLog;
use App\Mail\ReminderSettings;
use App\Form\BadgeAdminType;
use App\Form\CreationAdminType;
use App\Form\EventAdminType;
use App\Form\LoanableItemAdminType;
use App\Form\LoanAdminType;
use App\Form\MaintenanceBatchType;
use App\Form\Emails\MailAccountType;
use App\Form\Emails\MailRemindersType;
use App\Form\Emails\MailTestType;
use App\Form\Settings\AdvancedSettingsType;
use App\Form\Settings\AlertsSettingsType;
use App\Form\Settings\GeneralSettingsType;
use App\Form\Settings\LocalisationSettingsType;
use App\Form\Admin\CategoryCreateType;
use App\Form\Admin\OpeningHoursExceptionType;
use App\Form\Admin\PersonTypeType;
use App\Form\Admin\ThemeDraftType;
use App\Form\Admin\WizardType;
use App\Form\Settings\OperationsSettingsType;
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
use App\Repository\EventCategoryRepository;
use App\Repository\EventRepository;
use App\Repository\FormationRepository;
use App\Repository\LabPageRepository;
use App\Repository\LogUtilisationRepository;
use App\Repository\LoanableItemRepository;
use App\Repository\LoanRepository;
use App\Repository\MachineCategoryRepository;
use App\Repository\MachineDocumentRepository;
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
use App\Repository\UtilisateurBadgeRepository;
use App\Repository\UtilisateurRepository;
use App\Repository\VenueRepository;
use App\Feature\SiteFeatureService;
use App\Service\LocaleCatalog;
use App\Service\SiteSettingService;
use App\Repository\ScheduleExceptionRepository;
use App\Entity\ScheduleException;
use App\Schedule\ScheduleResolver;
use App\Service\TrainingQualificationService;
use App\Service\ThemeManager;
use App\Entity\HomepageSectionVisibility;
use App\Repository\HomepageSectionVisibilityRepository;
use App\Service\HomepageVisibilityService;
use App\UsageRights\UsageAllowanceService;
use App\UsageRights\UsageRightsService;
use App\UsageRights\UsageCapabilityRegistry;
use App\UsageRights\UsagePackageRepository;
use App\UsageRights\UserGroupRepository;
use App\UsageRights\AudienceResolver;
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
        ScheduleResolver $schedule,
        LabClock $clock,
    ): Response {
        $recentActivities = $this->buildRecentActivities($rfidLogs, $reservations, $progressions, $usageLogs);

        return $this->render('site/admin-dashboard.html.twig', [
            // ⚠️ **S153 — la bande porte un FAIT, et il vient du résolveur.**
            // Une bande qui ne dit que bonjour est un bandeau ; une bande qui dit
            // l'état du lab est une information. Le fait est donc calculé, jamais
            // écrit dans le gabarit : une bande qui affirme « Ouvert » un jour de
            // fermeture est pire que pas de bande du tout.
            'openState' => $this->openState($schedule, $clock),
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

    /**
     * L'état du lab maintenant, en trois valeurs, pour la bande du tableau de bord.
     *
     * 🔴 **`isOpenAt()` et non l'enveloppe** : à 12:30 dans un lab qui ferme le
     * midi, l'enveloppe dit « ouvert » et la porte est fermée. C'est la même
     * leçon que S134d, et la raison pour laquelle on parcourt les INTERVALLES
     * plutôt que `openMinutesFor()`.
     *
     * ⚠️ **`null` comme lieu, comme partout ailleurs** : c'est le lieu par
     * défaut. L'admin n'a pas de sélecteur de lieu sur cette page, et agréger
     * plusieurs lieux ne donne aucune réponse unique — voir la même note dans
     * `SiteController::machines()`.
     *
     * ⚠️ La raison n'existe que pour une fermeture DATÉE : `closureReasonFor()`
     * rend `null` un jour ouvré, y compris pendant la pause de midi. « Fermé »
     * seul y est donc la bonne phrase, et l'heure de réouverture la complète.
     *
     * @return array{open: bool, until: ?string, from: ?string, reason: ?string}
     */
    private function openState(ScheduleResolver $schedule, LabClock $clock): array
    {
        $now = $clock->now();
        $minute = ((int) $now->format('H')) * 60 + (int) $now->format('i');
        $intervals = $schedule->openIntervalsFor(null, $now);

        $clockOf = static fn (int $minutes): string => sprintf('%02d:%02d', intdiv($minutes, 60), $minutes % 60);

        foreach ($intervals as $interval) {
            if ($minute >= $interval['start'] && $minute < $interval['end']) {
                return ['open' => true, 'until' => $clockOf($interval['end']), 'from' => null, 'reason' => null];
            }
            // Trié et fusionné par le résolveur : le premier intervalle qui
            // commence après maintenant EST la prochaine ouverture du jour.
            if ($minute < $interval['start']) {
                return ['open' => false, 'until' => null, 'from' => $clockOf($interval['start']), 'reason' => null];
            }
        }

        return [
            'open' => false,
            'until' => null,
            'from' => null,
            'reason' => $schedule->closureReasonFor(null, $now),
        ];
    }

    #[Route('/homepage', name: 'app_admin_homepage', methods: ['GET', 'POST'])]
    public function homepage(
        Request $request,
        HomepageVisibilityService $homepageVisibility,
        HomepageSectionVisibilityRepository $homepageSections,
        EntityManagerInterface $entityManager,
        UserGroupRepository $userGroups,
    ): Response {
        // ⚠️ **S159f — la question se pose aux GROUPES.** Elle demandait « la table
        // des rôles connaît-elle staff ? » ; elle demande maintenant « le groupe
        // staff existe-t-il ? ». Il est intégré, donc la réponse est oui sur toute
        // installation migrée — et non sur une installation sans la migration
        // S133b, ce qui est le repli honnête : pas de colonne pour un groupe qui
        // n'existe pas.
        $showStaffColumn = false;
        foreach ($userGroups->all() as $group) {
            if ($group['key'] === 'staff') {
                $showStaffColumn = true;
                break;
            }
        }

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
            $this->addFlash('success', 'flash.configuration_de_laccueil_mise_a_jour');

            return $this->redirectToRoute('app_admin_homepage');
        }

        return $this->render('site/admin-homepage.html.twig', [
            'sections' => $homepageVisibility->getAdminRows(),
            'showStaffColumn' => $showStaffColumn,
        ]);
    }

    /**
     * ⚠️ **S148, J-22 — le brouillon de thème passe au thème (sans ironie).**
     *
     * 🔴 Les trois refus de `ThemeManager::saveDraft()` étaient des exceptions dont
     * le message partait en flash, suivies d'une redirection : une couleur tapée
     * sans son dièse effaçait AUSSI les deux noms publics et le fichier de logo.
     *
     * ⚠️ **« Remplacer le brouillon » n'est pas une soumission de formulaire** :
     * c'est une action destructrice qui ne lit aucun champ, et la faire passer par
     * la validation la rendrait impossible tant qu'un champ est refusé — c'est-à-
     * dire exactement quand on veut s'en servir. Elle garde son bouton et son
     * jeton propres.
     */
    #[Route('/themes', name: 'app_admin_themes', methods: ['GET', 'POST'])]
    public function themes(Request $request, ThemeManager $themes): Response
    {
        if ($request->isMethod('POST') && $request->request->getString('action') === 'discard') {
            if (!$this->isCsrfTokenValid('admin_themes_discard', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

                return $this->redirectToRoute('app_admin_themes');
            }

            $themes->discardDraft();
            $this->addFlash('success', 'flash.brouillon_remplace');

            return $this->redirectToRoute('app_admin_themes');
        }

        $form = $this->createForm(ThemeDraftType::class, $themes->draft());
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            /** @var array<string, mixed> $data */
            $data = $form->getData();
            $publish = $request->request->getString('action') === 'publish';

            try {
                // ⚠️ On repasse par `saveDraft()` et non par un `set()` direct : c'est
                // le point de passage, il tronque et il valide, et deux chemins
                // d'écriture divergeraient.
                $themes->saveDraft($data);
                if ($publish) {
                    $themes->publish();
                }
                $this->addFlash('success', $publish ? 'flash.theme_publie' : 'flash.brouillon_enregistre');

                return $this->redirectToRoute('app_admin_themes', $request->request->getString('action') === 'preview' ? ['preview' => 1] : []);
            } catch (\InvalidArgumentException $exception) {
                // Ce que les contraintes n'ont pas vu — le point de passage a le
                // dernier mot, et son message se pose sur le formulaire.
                $form->addError(new FormError($exception->getMessage()));
            }
        }

        return $this->render('site/admin-themes.html.twig', [
            'form' => $form->createView(),
            'draft' => $themes->draft(),
            'published' => $themes->published(),
            'preview' => $request->query->getBoolean('preview'),
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/machines', name: 'app_admin_machines', methods: ['GET'])]
    public function machines(Request $request, MachineRepository $machines, BadgeRepository $badges, VenueContext $venueContext): Response
    {
        $filters = $this->extractFilters($request, ['q', 'statut', 'niveau', 'badge', 'category']);
        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);
        $allMachines = $machines->findBy($context['selected'] === null ? [] : ['venue' => $context['selected']], ['nom' => 'ASC']);

        return $this->render('site/admin-machines.html.twig', [
            'machines' => $machines->findForAdminFilters($filters, $context['selected']),
            'machineCategoryTiles' => $this->machineCategoryTiles($allMachines, $filters),
            'machineStatusTiles' => $this->machineStatusTiles($allMachines, $filters),
            // ⚠️ S134h: every filter this page carries, not two of them. The
            // category tiles and the "Affiner" menus all rebuild the URL from
            // this, so a key missing here is a filter silently dropped the
            // moment another control is touched — `niveau` and `badge` were.
            'machineListQuery' => array_filter($filters, static fn (string $value): bool => $value !== ''),
            'machineCount' => count($allMachines),
            'filters' => $filters,
            'availableBadges' => $badges->findBy([], ['nom' => 'ASC']),
            'venueContext' => $context,
        ]);
    }

    /**
     * Machine categories — a real create / rename / archive, not a facet (S133).
     *
     * ⚠️ **What was wrong.** This screen grouped the distinct values of
     * `MACHINE.categoryLabel` and counted them, under a heading and a sub-menu
     * entry that read like management. Nothing on it could be created, renamed or
     * retired; the only way to rename a category was to open every machine
     * carrying it and retype the word, and mistyping it once silently created a
     * second category. The roadmap's rule — never present a facet as management —
     * offered a real CRUD or removing the tab.
     *
     * ⚠️ **Every action states its impact before it happens.** A rename moves N
     * machines; a rename onto an existing name is a *merge* and says so; an
     * archive leaves N machines carrying the label and only stops it being
     * offered. The counts are on screen beside each control and repeated in the
     * confirmation.
     *
     * ⚠️ **The list is the union of the catalogue and what is in use**, so a
     * label typed straight into a machine form still appears here (as
     * "unregistered") and can be adopted. Until the operator runs the S133
     * migration the catalogue half is simply empty and this screen degrades to
     * exactly what it used to be, rather than 500ing.
     */
    /**
     * Event categories — the lab's own vocabulary for what KIND an event is (S146f).
     *
     * ⚠️ **Much simpler than its machine sibling, and deliberately so.**
     * `MachineCategory` joins on the LABEL because `MACHINE.categoryLabel` predates
     * it, which is why that screen has to "adopt" orphan labels and perform a rename
     * as a mass `UPDATE`. Events had no category at all before this, so
     * `EVENEMENT.categoryId` is a real foreign key from the start: a rename is one
     * field, and archiving cannot orphan anything.
     *
     * 🔴 **Nothing here — or anywhere — may branch on WHICH category.** They are
     * labels an operator renames at will; see the note on `EventCategory`.
     */
    #[Route('/evenements/categories', name: 'app_admin_event_categories', methods: ['GET', 'POST'])]
    public function eventCategories(
        Request $request,
        EventCategoryRepository $categories,
        EntityManagerInterface $entityManager,
        TranslatorInterface $translator,
    ): Response {
        // ⚠️ **S148, J-22 — un seul des quatre POST de l'écran est un formulaire.**
        // Renommer, archiver et déplacer sont des ACTIONS portant un identifiant,
        // écrites une par ligne du tableau : un type unique poserait le même `name`
        // sur chaque ligne et chaque soumission écrirait sur la première. Seule la
        // création est une liste de champs, et on la reconnaît à son préfixe.
        if ($request->isMethod('POST') && !$request->request->has('category_create')) {
            return $this->handleEventCategoryAction($request, $categories, $entityManager);
        }

        $form = $this->createForm(CategoryCreateType::class, null, self::EVENT_CATEGORY_FORM_OPTIONS);
        $form->handleRequest($request);

        if ($form->isSubmitted()) {
            if ($form->isValid()) {
                /** @var array<string, mixed> $data */
                $data = $form->getData();
                $label = trim((string) $data['label']);

                $fresh = (new EventCategory())
                    ->setLabel($label)
                    ->setIconSlug(trim((string) ($data['icon_slug'] ?? '')));

                // ⚠️ Le slug est dérivé du libellé et doit rester unique. « Atelier
                // bois » et « Atelier Bois » entrent en collision, et une erreur de
                // clé dupliquée sur un formulaire tapé à la main n'est pas une
                // réponse. 🔴 L'erreur se pose SUR le champ, plus en haut de page.
                if ($fresh->getSlug() === '' || $categories->findOneBySlug($fresh->getSlug()) !== null) {
                    $form->get('label')->addError(new FormError($translator->trans('flash.une_categorie_existe_deja_ou_son', ['%p1%' => $label], 'messages')));
                } else {
                    $fresh->setPosition(count($categories->findAllOrdered()));
                    $entityManager->persist($fresh);
                    $entityManager->flush();
                    $this->addFlash('success', ['flash.categorie_creee_aucun_evenement_ne_la', ['%p1%' => $label]]);

                    return $this->redirectToRoute('app_admin_event_categories');
                }
            }

            return $this->renderEventCategories($categories, $form, Response::HTTP_UNPROCESSABLE_ENTITY);
        }

        return $this->renderEventCategories($categories, $form);
    }

    /**
     * ⚠️ Les libellés vivent ici plutôt que dans le type : le type est PARTAGÉ avec
     * les catégories de machines, et ce qui distingue les deux écrans est exactement
     * cette liste — pas du comportement.
     *
     * @var array<string, string>
     */
    private const EVENT_CATEGORY_FORM_OPTIONS = [
        'name_label' => 'event_categories.col_name',
        'name_placeholder' => 'event_categories.name_placeholder',
        'icon_label' => 'event_categories.icon',
        'icon_placeholder' => 'atelier',
        'csrf_token_id' => 'admin_event_categories',
    ];

    /**
     * 🔴 **A refused creation must not empty the form.** It used to flash an error and
     * redirect, so an operator who typed a name and an icon slug and hit a duplicate
     * got a blank, closed form back and had to type both again. That is the one rule
     * the operator wrote in red: an invalid field must never cost the rest of the
     * form. Rendering in place with the submitted values, at 422, is what the event
     * form already does — same pattern, now here too.
     *
     * @param \Symfony\Component\Form\FormInterface $form the creation form, submitted or not
     */
    private function renderEventCategories(
        EventCategoryRepository $categories,
        \Symfony\Component\Form\FormInterface $form,
        int $status = Response::HTTP_OK,
    ): Response {
        $counts = $categories->countEventsByCategory();
        $rows = [];
        foreach ($categories->findAllOrdered() as $category) {
            $rows[] = [
                'entity' => $category,
                'id' => $category->getId(),
                'label' => $category->getLabel(),
                'slug' => $category->getSlug(),
                'archived' => $category->isArchived(),
                'count' => $counts[$category->getId()] ?? 0,
            ];
        }

        return $this->render(
            'site/admin-event-categories.html.twig',
            ['rows' => $rows, 'form' => $form->createView(), 'refused' => $form->isSubmitted()],
            new Response(status: $status),
        );
    }

    private function handleEventCategoryAction(
        Request $request,
        EventCategoryRepository $categories,
        EntityManagerInterface $entityManager,
    ): Response {
        if (!$this->isCsrfTokenValid('admin_event_categories', (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_event_categories');
        }

        $action = (string) $request->request->get('action');
        $id = $request->request->get('id');
        $category = $id !== null && $id !== '' ? $categories->find((int) $id) : null;
        $label = trim((string) $request->request->get('label'));

        if ($category === null) {
            $this->addFlash('error', 'flash.categorie_inconnue');

            return $this->redirectToRoute('app_admin_event_categories');
        }

        switch ($action) {
            case 'rename':
                if ($label === '') {
                    $this->addFlash('error', 'flash.le_nom_de_la_categorie_est');
                    break;
                }
                // ⚠️ The slug is NOT recomputed. It is in every shared filter link,
                // and a typo fixed on Tuesday must not 404 a link sent on Monday.
                $category->setLabel($label);
                $entityManager->flush();
                $this->addFlash('success', ['flash.categorie_renommee_en_les_evenements_qui', ['%p1%' => $label]]);
                break;

            case 'archive':
            case 'restore':
                $action === 'archive' ? $category->archive() : $category->restore();
                $entityManager->flush();
                $this->addFlash('success', $action === 'archive'
                    // ⚠️ Says what archiving does NOT do: the events keep their
                    // category and keep showing it. Every other archive in this
                    // product works that way and an operator expects it here too.
                    ? ['flash.categorie_evenement_archivee', ['%p1%' => $category->getLabel()]]
                    : ['flash.categorie_reactivee', ['%p1%' => $category->getLabel()]]);
                break;

            case 'move_up':
            case 'move_down':
                $this->moveEventCategory($category, $action === 'move_up' ? -1 : 1, $categories, $entityManager);
                break;

            default:
                $this->addFlash('error', 'flash.action_inconnue');
        }

        return $this->redirectToRoute('app_admin_event_categories');
    }

    /**
     * ⚠️ Positions are renumbered from scratch on every move rather than swapped.
     * Rows created before `position` existed, or two rows that ended up sharing a
     * number, make a swap a no-op that looks like a broken button.
     */
    private function moveEventCategory(
        EventCategory $category,
        int $direction,
        EventCategoryRepository $categories,
        EntityManagerInterface $entityManager,
    ): void {
        $ordered = array_values(array_filter(
            $categories->findAllOrdered(),
            static fn (EventCategory $row): bool => !$row->isArchived(),
        ));

        $index = null;
        foreach ($ordered as $position => $row) {
            if ($row->getId() === $category->getId()) {
                $index = $position;
                break;
            }
        }

        if ($index === null) {
            return;
        }

        $target = $index + $direction;
        if ($target < 0 || $target >= count($ordered)) {
            return;
        }

        [$ordered[$index], $ordered[$target]] = [$ordered[$target], $ordered[$index]];
        foreach ($ordered as $position => $row) {
            $row->setPosition($position);
        }

        $entityManager->flush();
    }

    #[Route('/machines/categories', name: 'app_admin_machine_categories', methods: ['GET', 'POST'])]
    public function machineCategories(Request $request, MachineRepository $machines, MachineCategoryRepository $categories, EntityManagerInterface $entityManager, VenueContext $venueContext, TranslatorInterface $translator): Response
    {
        // ⚠️ Voir `eventCategories()` : seule la création est une liste de champs.
        // Reprendre, renommer et archiver sont des actions écrites une par ligne.
        if ($request->isMethod('POST') && !$request->request->has('category_create')) {
            return $this->handleMachineCategoryAction($request, $categories, $entityManager);
        }

        $form = $this->createForm(CategoryCreateType::class, null, self::MACHINE_CATEGORY_FORM_OPTIONS);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            /** @var array<string, mixed> $data */
            $data = $form->getData();
            $label = trim((string) $data['label']);

            // 🔴 Le doublon posait un flash et REDIRIGEAIT : le nom et l'icône
            // qu'on venait de taper étaient perdus. Il se pose sur le champ.
            if ($categories->findOneByLabel($label) !== null) {
                $form->get('label')->addError(new FormError($translator->trans('flash.la_categorie_existe_deja', ['%p1%' => $label], 'messages')));
            } else {
                $entityManager->persist((new MachineCategory())
                    ->setLabel($label)
                    ->setIconSlug(trim((string) ($data['icon_slug'] ?? ''))));
                $entityManager->flush();
                $this->addFlash('success', ['flash.categorie_creee_aucune_machine_ne_la', ['%p1%' => $label]]);

                return $this->redirectToRoute('app_admin_machine_categories');
            }
        }

        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);
        $scoped = $machines->findBy($context['selected'] === null ? [] : ['venue' => $context['selected']], ['categoryLabel' => 'ASC']);

        // ⚠️ Two different counts, on purpose. The machines shown are the ones in
        // the selected location; the count that governs a rename or an archive is
        // the count across the WHOLE installation, because that is what the write
        // touches. Showing the scoped number beside a control that moves every
        // machine would understate the impact on a multi-location install.
        $inScope = $this->machineTaxonomyRows($scoped, static fn (Machine $machine): string => $machine->getCategoryLabel());
        $totalByLabel = [];
        foreach ($machines->findAll() as $machine) {
            $label = trim($machine->getCategoryLabel());
            $totalByLabel[$label] = ($totalByLabel[$label] ?? 0) + 1;
        }

        $rows = [];
        foreach ($categories->allOrdered() as $category) {
            $rows[$category->getLabel()] = [
                'id' => $category->getId(),
                'label' => $category->getLabel(),
                'iconSlug' => $category->getIconSlug(),
                'archived' => $category->isArchived(),
                'registered' => true,
                'total' => $totalByLabel[$category->getLabel()] ?? 0,
                'count' => 0,
                'machines' => [],
            ];
        }
        foreach ($inScope as $row) {
            $rows[$row['label']] ??= [
                'id' => null,
                'label' => $row['label'],
                'iconSlug' => null,
                'archived' => false,
                'registered' => false,
                'total' => $totalByLabel[$row['label']] ?? $row['count'],
                'count' => 0,
                'machines' => [],
            ];
            $rows[$row['label']]['count'] = $row['count'];
            $rows[$row['label']]['machines'] = $row['machines'];
        }
        uksort($rows, 'strnatcasecmp');

        return $this->render('site/admin-machine-categories.html.twig', [
            'rows' => array_values($rows),
            'venueContext' => $context,
            'catalogueReady' => $categories->tableExists(),
            'form' => $form->createView(),
            'refused' => $form->isSubmitted(),
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    /** @var array<string, string> */
    private const MACHINE_CATEGORY_FORM_OPTIONS = [
        'name_label' => 'machine_taxonomy.col_name',
        'name_placeholder' => 'machine_categories.name_placeholder',
        'icon_label' => 'machine_categories.icon',
        'icon_placeholder' => 'decoupe-laser',
        'csrf_token_id' => 'admin_machine_categories',
    ];

    /**
     * The four writes of the categories screen, each with its impact reported.
     *
     * ⚠️ A rename is **two** statements that must happen together: the catalogue
     * row and every machine carrying the old word. `wrapInTransaction` is what
     * stops the half where the catalogue says "Découpe laser" and eleven machines
     * still say "Decoupe laser" — the exact partial-write shape S134g shipped once
     * and had to fix.
     */
    private function handleMachineCategoryAction(Request $request, MachineCategoryRepository $categories, EntityManagerInterface $entityManager): Response
    {
        if (!$this->isCsrfTokenValid('admin_machine_categories', (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_machine_categories');
        }

        $action = (string) $request->request->get('action');
        $label = trim((string) $request->request->get('label'));
        $newLabel = trim((string) $request->request->get('new_label'));

        try {
            switch ($action) {
                case 'adopt':
                    // A label typed straight into a machine form, brought into the
                    // catalogue so it can be renamed and archived like the others.
                    if ($label === '' || $categories->findOneByLabel($label) !== null) {
                        $this->addFlash('error', 'flash.cette_categorie_ne_peut_pas_etre');
                        break;
                    }
                    $entityManager->persist((new MachineCategory())->setLabel($label));
                    $entityManager->flush();
                    $this->addFlash('success', ['flash.categorie_reprise_au_catalogue', ['%p1%' => $label]]);
                    break;

                case 'rename':
                    if ($label === '' || $newLabel === '') {
                        $this->addFlash('error', 'flash.les_deux_noms_obligatoires');
                        break;
                    }
                    if ($newLabel === $label) {
                        break;
                    }
                    $target = $categories->findOneByLabel($newLabel);
                    $moved = $entityManager->wrapInTransaction(function () use ($entityManager, $categories, $label, $newLabel, $target): int {
                        $moved = (int) $entityManager->getConnection()->executeStatement(
                            'UPDATE MACHINE SET categoryLabel = :new WHERE TRIM(categoryLabel) = :old',
                            ['new' => $newLabel, 'old' => $label],
                        );

                        $source = $categories->findOneByLabel($label);
                        if ($source !== null && $target !== null) {
                            // A rename onto an existing name is a merge: the source
                            // row goes, the target keeps its icon and its state.
                            $entityManager->remove($source);
                        } elseif ($source !== null) {
                            $source->setLabel($newLabel);
                        } elseif ($target === null) {
                            $entityManager->persist((new MachineCategory())->setLabel($newLabel));
                        }
                        $entityManager->flush();

                        return $moved;
                    });
                    $this->addFlash('success', $target !== null
                        ? ['flash.categorie_fusionnee', ['p1' => $label, 'p2' => $newLabel, 'p3' => $moved]]
                        : ['flash.categorie_renommee', ['p1' => $label, 'p2' => $newLabel, 'p3' => $moved]]);
                    break;

                case 'archive':
                case 'restore':
                    $category = $categories->findOneByLabel($label);
                    if ($category === null) {
                        $this->addFlash('error', 'flash.categorie_inconnue');
                        break;
                    }
                    $action === 'archive' ? $category->archive() : $category->restore();
                    $entityManager->flush();
                    $this->addFlash('success', $action === 'archive'
                        // ⚠️ Says what archiving does NOT do. An operator who reads
                        // "archived" and expects the machines to lose the category
                        // has been misled by every other archive in this product.
                        ? ['flash.categorie_machine_archivee', ['%p1%' => $label]]
                        : ['flash.categorie_reactivee', ['%p1%' => $label]]);
                    break;

                default:
                    $this->addFlash('error', 'flash.action_inconnue');
            }
        } catch (\Throwable $e) {
            // The one likely cause is the S133 migration not having been run yet.
            $this->addFlash('error', 'flash.le_catalogue_de_categories_nest_pas');
        }

        return $this->redirectToRoute('app_admin_machine_categories');
    }

    #[Route('/machines/models', name: 'app_admin_machine_models', methods: ['GET'])]
    public function machineModels(Request $request, MachineRepository $machines, VenueContext $venueContext): Response
    {
        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);
        $rows = $machines->findBy($context['selected'] === null ? [] : ['venue' => $context['selected']], ['manufacturer' => 'ASC', 'model' => 'ASC']);

        return $this->render('site/admin-machine-taxonomy.html.twig', [
            'title' => 'machine_taxonomy.models_title',
            'description' => 'machine_taxonomy.models_description',
            'rows' => $this->machineTaxonomyRows($rows, static fn (Machine $machine): string => trim(($machine->getManufacturer() ?? '') . ' ' . ($machine->getModel() ?? '')) ?: 'Non renseigné'),
            'venueContext' => $context,
            'routeName' => 'app_admin_machine_models',
            'filterKey' => 'model',
        ]);
    }

    #[Route('/machines/new', name: 'app_admin_machine_new', methods: ['GET', 'POST'])]
    public function newMachine(Request $request, EntityManagerInterface $entityManager, BadgeRepository $badges, MachineRepository $machines, MachineCategoryRepository $categories, VenueRepository $venues): Response
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

            $this->addFlash('success', ['flash.machine_creee', ['%p1%' => $machine->getNom()]]);
            if ($submittedBadgeIds === []) {
                $this->addFlash('warning', 'flash.cette_machine_na_aucun_badge_requis');
            }

            return $this->redirectToRoute('app_admin_machines');
        }

        return $this->render('site/admin-machine-new.html.twig', [
            'machine' => $machine,
            'form' => $form,
            // ⚠️ Non-archived only. Archiving a category means it stops being
            // offered; a machine already carrying it keeps it, which is why the
            // field stays free text rather than becoming a ChoiceType (S133).
            'categoryChoices' => array_map(
                static fn (MachineCategory $category): string => $category->getLabel(),
                $categories->allOrdered(includeArchived: false),
            ),
            'availableBadges' => $availableBadges,
            'selectedBadgeIds' => array_map('intval', array_filter($submittedBadgeIds, static fn ($id): bool => is_scalar($id) && ctype_digit((string) $id))),
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/machines/{id}/edit', name: 'app_admin_machine_edit', requirements: ['id' => '\\d+'], methods: ['GET', 'POST'])]
    public function editMachine(Machine $machine, Request $request, EntityManagerInterface $entityManager, BadgeRepository $badges, MachineCategoryRepository $categories, MachineDocumentRepository $machineDocuments): Response
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
            $this->addFlash('success', ['flash.machine_mise_a_jour', ['%p1%' => $machine->getNom()]]);

            return $this->redirectToRoute('app_admin_machines');
        }

        return $this->render('site/admin-machine-edit.html.twig', [
            // Les documents attachés (S152). ⚠️ Passés au gabarit, pas au
            // `FormType` : ils ont leur propre formulaire et leur propre route.
            'machineDocuments' => $machineDocuments->forMachine($machine),
            'machine' => $machine,
            'form' => $form,
            'categoryChoices' => array_map(
                static fn (MachineCategory $category): string => $category->getLabel(),
                $categories->allOrdered(includeArchived: false),
            ),
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
    public function formations(Request $request, FormationRepository $formations, ProgressionRepository $progressions, BadgeRepository $badges): Response
    {
        $filters = $this->extractFilters($request, ['q', 'niveau', 'badge']);

        return $this->render('site/admin-formations.html.twig', [
            'formations' => $formations->findForAdminFilters($filters),
            // ⚠️ S134h — see the users action: tile counts and the "3 sur 11"
            // scope line are about the list BEFORE filtering.
            'allFormations' => $formations->findForAdminFilters(['q' => '', 'niveau' => '', 'badge' => '']),
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
            $this->addFlash('success', ['flash.formation_creee', ['%p1%' => $formation->getTitre()]]);
            if ($formation->getBadge() === null) {
                $this->addFlash('warning', 'flash.cette_formation_ne_donnera_aucun_badge');
            }

            return $this->redirectToRoute('app_admin_formations');
        }

        return $this->render('site/admin-formation-new.html.twig', [
            'formation' => $formation,
            'form' => $form,
            // 🔴 **S151 — la catégorie d'une formation est du texte libre, et DEUX
            // valeurs y ont un sens caché.** « Quiz interne » (53 lignes) et
            // « Validation physique » (7) font sortir la formation du catalogue
            // public : ce sont les échafaudages du parcours guidé, et
            // `/formations/{id}` répond 404 pour elles. Une faute de frappe créait
            // donc, en silence, une formation ORDINAIRE et publique là où
            // l'opérateur croyait poser un échafaudage.
            // ⚠️ Une liste des catégories DÉJÀ UTILISÉES, comme les machines en ont
            // une depuis S133 — pas un `ChoiceType` : une catégorie reste du texte
            // libre, la liste propose sans imposer.
            'categoryChoices' => $formations->usedCategories(),
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/formations/{id}/edit', name: 'app_admin_formation_edit', requirements: ['id' => '\\d+'], methods: ['GET', 'POST'])]
    public function editFormation(Formation $formation, Request $request, EntityManagerInterface $entityManager, FormationRepository $formations): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(FormationAdminType::class, $formation);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $entityManager->flush();
            $this->addFlash('success', ['flash.formation_mise_a_jour', ['%p1%' => $formation->getTitre()]]);

            return $this->redirectToRoute('app_admin_formations');
        }

        return $this->render('site/admin-formation-edit.html.twig', [
            'formation' => $formation,
            'form' => $form,
            'categoryChoices' => $formations->usedCategories(),
        ]);
    }

    #[Route('/reservations', name: 'app_admin_reservations', methods: ['GET'])]
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
        // ⚠️ S134h — the tiles count this, not `$rows`. Selecting "Annulées"
        // used to make all four tiles report the number of cancelled bookings.
        $allRows = $reservations->findForAdminFilters(
            ['q' => '', 'statut' => '', 'dateFrom' => '', 'dateTo' => '', 'reservableType' => $filters['reservableType']],
        );
        $reservables->warm($rows);

        return $this->render('site/admin-reservations.html.twig', [
            'reservations' => $rows,
            'allReservations' => $allRows,
            'filters' => $filters,
        ]);
    }


    #[Route('/horaires', name: 'app_admin_opening_hours', methods: ['GET', 'POST'])]
    public function openingHours(Request $request, OpeningHourRepository $openingHours, ScheduleResolver $schedule,
        ScheduleExceptionRepository $exceptions, MachineRepository $machines, PlaceRepository $places,
        VenueContext $venueContextService, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        // ⚠️ S131. This used to be `$venues->findDefault()`, so the screen could only
        // ever edit the default venue's week — while the table has been keyed
        // `(venueId, dayOfWeek)` since S106. A second location was creatable from
        // S129 and could never be given opening hours. `single()` is the shared
        // resolution for venue-scoped *editing*; it never answers "all", because
        // there is no row to write for "all".
        $venueContext = $venueContextService->single($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);
        $venue = $venueContext['selected'];
        // ⚠️ Only the LOCATION's level is seeded. A scoped level starts empty on
        // purpose: empty means "nothing to say, the level above answers", and
        // seeding it with a week would silently attach hours nobody asked for.
        $this->ensureOpeningHourRows($openingHours, $schedule, $venue, $entityManager);
        [$scopeType, $scopeId] = $this->parseScheduleScope((string) $request->query->get('scope', ''));
        $errors = [];

        // ⚠️ **S148, J-22 — un seul des trois formulaires de l'écran se convertit.**
        // Le sélecteur de portée est un filtre GET (sa place est l'URL, elle y est
        // déjà) et la semaine est une matrice `open_2[]` / `close_2[]`. Reste
        // l'exception, qui est bien une liste de champs.
        $exceptionForm = $this->createForm(OpeningHoursExceptionType::class, ['exception_closed' => true], [
            'reason_placeholder' => 'hours.exception_reason_placeholder',
        ]);
        $exceptionForm->handleRequest($request);

        if ($exceptionForm->isSubmitted()) {
            if ($exceptionForm->isValid()) {
                /** @var array<string, mixed> $data */
                $data = $exceptionForm->getData();
                $this->addScheduleException($data, $venue, $entityManager);

                return $this->redirectToRoute('app_admin_opening_hours', array_filter([
                    'location' => $venueContext['location'],
                    'scope' => $this->scopeKey($scopeType, $scopeId),
                ]));
            }
            // 🔴 On tombe dans le rendu final avec le formulaire SOUMIS. Avant, les
            // quatre refus étaient des phrases françaises en dur en haut de page et
            // le formulaire revenait vide.
        } elseif ($request->isMethod('POST')) {
            $action = (string) $request->request->get('action', 'save_week');
            if (!$this->isCsrfTokenValid('admin_opening_hours', (string) $request->request->get('_token'))) {
                $errors['_global'][] = 'Token CSRF invalide.';
            } elseif ($action === 'delete_exception') {
                $exception = $exceptions->find($request->request->getInt('exception_id'));
                // Scoped to the venue on screen: an id from another location's
                // form must not delete across the boundary.
                if ($exception !== null && $exception->getVenue()?->getId() === $venue->getId()) {
                    $entityManager->remove($exception);
                    $entityManager->flush();
                    $this->addFlash('success', 'flash.exception_supprimee');
                }
            } else {
                [$scopeType, $scopeId] = $this->parseScheduleScope((string) $request->request->get('scope', ''));
                $errors = $this->saveOpeningWeek($request, $openingHours, $venue, $entityManager, $scopeType, $scopeId);
            }

            if ($errors === []) {
                // Keep the venue in the URL: redirecting bare would bounce the operator
                // back to their default venue after editing another one's week.
                return $this->redirectToRoute('app_admin_opening_hours', array_filter([
                    'location' => $venueContext['location'],
                    'scope' => $this->scopeKey($scopeType, $scopeId),
                ]));
            }
        }

        return $this->render('site/admin-opening-hours.html.twig', [
            // ⚠️ Grouped BY DAY (S134d). A day is no longer one row, so a flat
            // list would render Tuesday twice with no indication that the two
            // lines belong together.
            'week' => $this->weekFor($openingHours, $venue, $scopeType, $scopeId),
            'exceptions' => $exceptions->upcomingFor($venue),
            'exceptionsReady' => $exceptions->tableExists(),
            'errors' => $errors,
            'venueContext' => $venueContext,
            'scopeKey' => $this->scopeKey($scopeType, $scopeId),
            'scopeChoices' => $this->scheduleScopeChoices($openingHours, $machines, $places, $venue),
            'exceptionForm' => $exceptionForm->createView(),
            // ⚠️ **What the hours actually DO, after intersection.** A resource
            // level can only narrow its location's, so a range written wider than
            // the location has no effect — and an operator who cannot see that
            // has written a control that does nothing. This is the price the
            // intersect decision agreed to pay, paid here.
            'effective' => $scopeType === null ? [] : $this->effectiveWeek($schedule, $venue, $scopeType, $scopeId),
        ], $request->isMethod('POST') ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    /**
     * `""` | `machine` | `machine:12` → [kind, id]. (S134d)
     *
     * ⚠️ One field, and the kind derived from it, for the same reason the grant
     * editor does it: two selects let an operator pair "espace" with a machine
     * and store a scope that matches nothing.
     *
     * @return array{0: ?string, 1: ?int}
     */
    private function parseScheduleScope(string $raw): array
    {
        $raw = trim($raw);
        if ($raw === '') {
            return [null, null];
        }

        [$kind, $id] = array_pad(explode(':', $raw, 2), 2, null);
        $type = ReservableType::tryParse($kind);
        if ($type === null) {
            return [null, null];
        }

        return [$type->value, ctype_digit((string) $id) ? (int) $id : null];
    }

    private function scopeKey(?string $scopeType, ?int $scopeId): string
    {
        return $scopeType === null ? '' : $scopeType . ($scopeId !== null ? ':' . $scopeId : '');
    }

    /**
     * The levels the picker offers: the location, each resource kind, and every
     * individual resource — with a mark on the ones that already carry hours, so
     * an operator can find what they wrote without remembering it.
     *
     * @return list<array{key: string, label: string, written: bool}>
     */
    private function scheduleScopeChoices(OpeningHourRepository $openingHours, MachineRepository $machines, PlaceRepository $places, \App\Entity\Venue $venue): array
    {
        $written = [];
        foreach ($openingHours->scopesWithRows($venue) as $scope) {
            $written[$this->scopeKey($scope['scopeType'], $scope['scopeId'])] = true;
        }

        $choices = [
            ['key' => '', 'label' => 'hours.scope_venue', 'written' => true, 'is_key' => true],
            ['key' => 'machine', 'label' => 'hours.scope_all_machines', 'written' => isset($written['machine']), 'is_key' => true],
            ['key' => 'place', 'label' => 'hours.scope_all_places', 'written' => isset($written['place']), 'is_key' => true],
        ];

        foreach ($machines->findBy(['venue' => $venue], ['nom' => 'ASC']) as $machine) {
            $key = 'machine:' . $machine->getId();
            $choices[] = ['key' => $key, 'label' => $machine->getNom(), 'written' => isset($written[$key]), 'is_key' => false];
        }
        foreach ($places->findBy(['venue' => $venue], ['nom' => 'ASC']) as $place) {
            $key = 'place:' . $place->getId();
            $choices[] = ['key' => $key, 'label' => $place->getNom(), 'written' => isset($written[$key]), 'is_key' => false];
        }

        return $choices;
    }

    /**
     * What a scoped week actually resolves to, day by day (S134d).
     *
     * 🔴 **This is what makes the intersect decision honest.** A resource may only
     * NARROW its location's hours, so a range written wider does nothing at all —
     * and hours that silently do nothing are the exact fault this codebase has
     * hit three times in a week. Rather than forbid the input, the screen shows
     * the resolved answer beside it and marks the day as having no effect.
     *
     * @return array<int, array{intervals: list<array{start:int,end:int}>, venue: list<array{start:int,end:int}>}>
     */
    private function effectiveWeek(ScheduleResolver $schedule, \App\Entity\Venue $venue, string $scopeType, ?int $scopeId): array
    {
        $out = [];
        // Any week containing all seven weekdays does; only the day-of-week is
        // read, never the date. Monday-first so day 1 is Monday.
        $monday = new \DateTimeImmutable('monday this week');
        for ($day = 1; $day <= 7; $day++) {
            $date = $monday->modify(sprintf('+%d days', $day - 1));
            $out[$day] = [
                'intervals' => $schedule->openIntervalsFor($venue->getId(), $date, $scopeType, $scopeId),
                'venue' => $schedule->openIntervalsFor($venue->getId(), $date),
            ];
        }

        return $out;
    }

    /**
     * The week of one location, grouped by weekday (S134d).
     *
     * @return list<array{dayOfWeek:int,label:string,closed:bool,ranges:list<OpeningHour>}>
     */
    private function weekFor(OpeningHourRepository $openingHours, \App\Entity\Venue $venue, ?string $scopeType = null, ?int $scopeId = null): array
    {
        $byDay = [];
        foreach ($openingHours->findOrderedForScope($venue, $scopeType, $scopeId) as $row) {
            $byDay[$row->getDayOfWeek()] ??= ['dayOfWeek' => $row->getDayOfWeek(), 'label' => $row->getLabel(), 'closed' => true, 'ranges' => []];
            if (!$row->isClosed() && $row->getOpenTime() !== null && $row->getCloseTime() !== null) {
                $byDay[$row->getDayOfWeek()]['closed'] = false;
                $byDay[$row->getDayOfWeek()]['ranges'][] = $row;
            }
        }
        ksort($byDay);

        return array_values($byDay);
    }

    /**
     * Rewrite one location's week from the form (S134d).
     *
     * 🔴 **Delete-then-insert, inside a transaction.** The form posts parallel
     * arrays of ranges per day and the operator can add or remove lines, so
     * matching submitted values back onto existing row ids would need a hidden id
     * per line and would still break the moment two lines were swapped. Replacing
     * a day wholesale is the only version with one obvious meaning — and it must
     * be atomic, or a validation error halfway through leaves a lab with three
     * days of opening hours.
     *
     * ⚠️ Validation runs over **everything** before anything is written, for the
     * same reason.
     *
     * @return array<int|string, list<string>> errors by day
     */
    private function saveOpeningWeek(Request $request, OpeningHourRepository $openingHours, \App\Entity\Venue $venue, EntityManagerInterface $entityManager, ?string $scopeType = null, ?int $scopeId = null): array
    {
        $errors = [];
        $planned = [];

        for ($day = 1; $day <= 7; $day++) {
            $closed = $request->request->getBoolean('closed_' . $day);
            $opens = array_values((array) $request->request->all('open_' . $day));
            $closes = array_values((array) $request->request->all('close_' . $day));
            $ranges = [];

            if (!$closed) {
                foreach ($opens as $index => $rawOpen) {
                    $open = $this->parseAdminTime(trim((string) $rawOpen));
                    $close = $this->parseAdminTime(trim((string) ($closes[$index] ?? '')));

                    // A wholly empty line is how a range is removed, not an error.
                    if (trim((string) $rawOpen) === '' && trim((string) ($closes[$index] ?? '')) === '') {
                        continue;
                    }
                    if (!$open || !$close) {
                        $errors[$day][] = 'Chaque plage demande une heure d\'ouverture et une heure de fermeture au format HH:mm.';
                        continue;
                    }
                    if ($close <= $open) {
                        $errors[$day][] = 'L\'heure de fermeture doit être après l\'heure d\'ouverture.';
                        continue;
                    }
                    $ranges[] = ['open' => $open, 'close' => $close];
                }

                usort($ranges, static fn (array $a, array $b): int => $a['open'] <=> $b['open']);
                // ⚠️ Overlapping ranges are refused rather than merged. Merging
                // would silently accept 09:00–14:00 beside 12:00–18:00 and show
                // back a week the operator did not type, which is how somebody
                // stops trusting the screen.
                foreach ($ranges as $index => $range) {
                    if ($index > 0 && $range['open'] < $ranges[$index - 1]['close']) {
                        $errors[$day][] = 'Deux plages de cette journée se chevauchent.';
                        break;
                    }
                }

                if ($ranges === []) {
                    $errors[$day][] = 'Une journée ouverte demande au moins une plage — sinon, cochez « fermé ».';
                }
            }

            $planned[$day] = ['closed' => $closed, 'ranges' => $ranges, 'label' => $this->dayLabelFor($openingHours, $venue, $day)];
        }

        if ($errors !== []) {
            return $errors;
        }

        $entityManager->wrapInTransaction(function () use ($openingHours, $venue, $planned, $entityManager, $scopeType, $scopeId): void {
            // ⚠️ Scoped: replacing a machine's week must not delete the location's.
            foreach ($openingHours->findOrderedForScope($venue, $scopeType, $scopeId) as $existing) {
                $entityManager->remove($existing);
            }
            $entityManager->flush();

            foreach ($planned as $day => $data) {
                $rows = $data['closed']
                    ? [['open' => null, 'close' => null]]
                    : $data['ranges'];

                foreach ($rows as $index => $range) {
                    $entityManager->persist((new OpeningHour())
                        ->setVenue($venue)
                        ->setDayOfWeek($day)
                        ->setLabel($data['label'])
                        ->setIsClosed($data['closed'])
                        ->setOpenTime($range['open'])
                        ->setCloseTime($range['close'])
                        // Ordered within the day so the screen reads back in the
                        // order the resolver evaluates.
                        ->setSortOrder($day * 10 + $index)
                        ->setScopeType($scopeType)
                        ->setScopeId($scopeId)
                        ->setUpdatedAt(new \DateTimeImmutable()));
                }
            }
        });

        $this->addFlash('success', 'flash.horaires_d_ouverture_mis_a_jour');

        return [];
    }

    private function dayLabelFor(OpeningHourRepository $openingHours, \App\Entity\Venue $venue, int $day): string
    {
        foreach ($openingHours->findOrdered($venue) as $row) {
            if ($row->getDayOfWeek() === $day) {
                return $row->getLabel();
            }
        }

        return ['1' => 'Lundi', '2' => 'Mardi', '3' => 'Mercredi', '4' => 'Jeudi', '5' => 'Vendredi', '6' => 'Samedi', '7' => 'Dimanche'][(string) $day] ?? '';
    }

    /**
     * One dated exception (S134d).
     *
     * ⚠️ A closure needs no times; a special opening needs both. The form can
     * produce "not closed, no times", which is a row nothing can read as an
     * opening — refused here rather than stored and silently ignored.
     *
     * @return array<int|string, list<string>>
     */
    /**
     * ⚠️ **Les quatre refus sont partis dans `OpeningHoursExceptionType`.** Ce qui
     * reste ici est l'écriture, et elle ne peut plus rien refuser : la méthode
     * n'est appelée qu'avec un formulaire valide.
     *
     * @param array<string, mixed> $data
     */
    private function addScheduleException(array $data, \App\Entity\Venue $venue, EntityManagerInterface $entityManager): void
    {
        $raw = (string) $data['exception_date'];
        // ⚠️ **S146g — the end is optional and means "one day" when absent.** A blank
        // field is the common case, so it must not be an error.
        $rawEnd = trim((string) ($data['exception_end'] ?? ''));
        $closed = (bool) ($data['exception_closed'] ?? false);
        $open = $this->parseAdminTime(trim((string) ($data['exception_open'] ?? '')));
        $close = $this->parseAdminTime(trim((string) ($data['exception_close'] ?? '')));

        $exception = (new ScheduleException())
            ->setVenue($venue)
            ->setExceptionDate(new \DateTimeImmutable($raw))
            ->setIsClosed($closed)
            ->setOpenTime($closed ? null : $open)
            ->setCloseTime($closed ? null : $close)
            ->setReason(trim((string) ($data['exception_reason'] ?? '')));
        // ⚠️ Set after the start date: `setEndDate()` compares against it, and an end
        // that is not after the start is stored as a single day rather than refused.
        $exception->setEndDate($rawEnd !== '' ? new \DateTimeImmutable($rawEnd) : null);

        $entityManager->persist($exception);
        $entityManager->flush();
        $this->addFlash('success', $exception->spansSeveralDays()
            ? ['flash.fermeture_plusieurs_jours', ['%p1%' => $exception->dayCount()]]
            : 'flash.exception_enregistree');
    }

    #[Route('/utilisateurs', name: 'app_admin_users', methods: ['GET'])]
    public function users(
        Request $request,
        UtilisateurRepository $users,
        AccessRfidLogRepository $logs,
        ProgressionRepository $progressions,
        UsagePackageRepository $packages,
        UserGroupRepository $userGroups,
        AudienceResolver $audiences,
        LabClock $clock,
    ): Response {
        // 🔴 **S159f — le filtre « rôle » est parti, il faisait double emploi.**
        // Depuis la fusion, l'appartenance à un groupe intégré EST le rôle : deux
        // menus côte à côte posaient la même question, et celui des rôles la
        // posait moins bien — il joignait `UTILISATEUR_ROLE`, donc il ne voyait
        // pas quelqu'un rendu staff par son groupe.
        $filters = $this->extractFilters($request, ['q', 'statut', 'package', 'groupe']);
        $logCounts = [];
        foreach ($logs->createQueryBuilder('log')
            ->select('IDENTITY(log.utilisateur) AS userId, COUNT(log.id) AS logCount')
            ->where('log.utilisateur IS NOT NULL')
            ->groupBy('log.utilisateur')
            ->getQuery()
            ->getArrayResult() as $row) {
            $logCounts[(int) $row['userId']] = (int) $row['logCount'];
        }

        // ⚠️ S134h: the tile counts and the "3 sur 11" scope line both need the
        // list BEFORE filtering. The template used to count `users`, which is the
        // filtered result — so with a status selected every tile reported the
        // same number, the number of rows already on screen.
        $allUsers = $users->findForAdminFilters(['q' => '', 'statut' => '', 'package' => '', 'groupe' => '']);

        // ⚠️ Les deux filtres se composent, et dans cet ordre parce qu'il n'en a
        // aucune importance : chacun ne fait que retirer des lignes.
        $rows = $this->filterByPackage(
            $users->findForAdminFilters($filters),
            trim((string) ($filters['package'] ?? '')),
            $packages,
            $audiences,
            $clock,
        );

        return $this->render('site/admin-utilisateurs.html.twig', [
            'users' => $this->filterByGroup(
                $rows,
                trim((string) ($filters['groupe'] ?? '')),
                $userGroups,
                $audiences,
            ),
            'allUsers' => $allUsers,
            'logCounts' => $logCounts,
            'progressionCounts' => $this->buildUserProgressionStats($progressions),
            'filters' => $filters,
            // ⚠️ Tous les packages, actifs ou non : un package désactivé garde ses
            // attributions, et « qui avait ce forfait » est justement la question
            // qu'on pose le jour où on le désactive.
            'packageChoices' => $packages->findAll(),
            // ⚠️ Les audiences VIRTUELLES sont proposées comme les autres : « tout
            // compte actif » est un filtre parfaitement sensé, et c'est justement
            // celui qu'aucune jointure ne saurait écrire.
            'groupChoices' => $userGroups->all(),
        ]);
    }

    /**
     * Les personnes que ce groupe contient, dans la liste déjà filtrée (S158c).
     *
     * 🔴 **L'appartenance se demande à `AudienceResolver`, jamais à une jointure
     * écrite ici.** Elle est l'union de trois choses — les lignes stockées, les
     * rôles, et l'audience `user` qui n'est écrite NULLE PART et vaut « tout
     * compte actif ». Un filtre qui joindrait `USER_GROUP_MEMBER` répondrait
     * « personne » sur `user`, et raterait la moitié des autres tant que
     * l'amorçage par les rôles n'est pas sorti. `primeFor()` est là pour que ce
     * soit une requête et pas une par ligne.
     *
     * ⚠️ Une clé inconnue rend la liste INCHANGÉE, jamais vide — même règle que
     * le filtre par forfait juste au-dessus.
     *
     * @param list<Utilisateur> $rows
     * @return list<Utilisateur>
     */
    private function filterByGroup(array $rows, string $groupId, UserGroupRepository $groups, AudienceResolver $audiences): array
    {
        if ($groupId === '' || !ctype_digit($groupId)) {
            return array_values($rows);
        }
        $group = $groups->find((int) $groupId);
        if ($group === null) {
            return array_values($rows);
        }

        $audiences->primeFor($rows);

        return array_values(array_filter(
            $rows,
            static fn (Utilisateur $user): bool => in_array($group['key'], $audiences->keysFor($user), true),
        ));
    }

    /**
     * Les personnes que ce package atteint, dans la liste déjà filtrée (S153c).
     *
     * 🔴 **Les DEUX chemins d'attribution, et c'est tout l'enjeu.** Une
     * attribution est personnelle ou collective ; un filtre qui n'en verrait
     * qu'un afficherait « personne » pour un package donné à une équipe entière.
     *
     * 🔴 **Et l'appartenance à un groupe se demande à `AudienceResolver`, jamais
     * à une requête écrite ici.** Elle est l'union de trois choses — des lignes
     * stockées, des rôles, et l'audience `user` qui n'est écrite nulle part et
     * vaut « tout compte actif ». La refaire en SQL serait une deuxième vérité
     * sur l'appartenance, et elle dériverait le jour où S134 déplace celle-ci
     * dans les groupes. `primeFor()` existe pour que ce soit UNE requête et pas
     * une par ligne.
     *
     * ⚠️ Une clé inconnue rend la liste INCHANGÉE, jamais vide : un package
     * supprimé pendant qu'on tient son lien ne doit pas répondre « aucun membre »,
     * qui est une phrase fausse sur le lab plutôt qu'un filtre sans objet. Même
     * règle que la catégorie d'événement de S146f.
     *
     * @param list<Utilisateur> $rows
     * @return list<Utilisateur>
     */
    private function filterByPackage(
        array $rows,
        string $packageId,
        UsagePackageRepository $packages,
        AudienceResolver $audiences,
        LabClock $clock,
    ): array {
        if ($packageId === '' || !ctype_digit($packageId) || $packages->find((int) $packageId) === null) {
            return array_values($rows);
        }

        $reach = $packages->reachOf((int) $packageId, $clock->now());
        if ($reach['userIds'] === [] && $reach['groupKeys'] === []) {
            return [];
        }

        $direct = array_flip($reach['userIds']);
        $groupKeys = array_flip($reach['groupKeys']);
        $audiences->primeFor($rows);

        return array_values(array_filter($rows, static function (Utilisateur $user) use ($direct, $groupKeys, $audiences): bool {
            if (isset($direct[(int) $user->getId()])) {
                return true;
            }
            foreach ($audiences->keysFor($user) as $key) {
                if (isset($groupKeys[$key])) {
                    return true;
                }
            }

            return false;
        }));
    }

    #[Route('/utilisateurs/new', name: 'app_admin_user_new', methods: ['GET', 'POST'])]
    public function newUser(
        Request $request,
        EntityManagerInterface $entityManager,
        UtilisateurRepository $users,
        UserPasswordHasherInterface $passwordHasher,
        LocaleCatalog $locales,
        UserGroupRepository $userGroups,
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

        $roleChoices = $this->buildAdminRoleChoices($userGroups);
        $form = $this->createForm(UserAdminType::class, $user, [
            'role_choices' => $roleChoices,
            'locale_choices' => $locales->choices(),
        ]);
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

            // ⚠️ La valeur est désormais une clé de GROUPE, et le refus se fait
            // contre la liste offerte — plus contre la table `ROLE`.
            $selectedGroup = (string) $form->get('role')->getData();
            if (!in_array($selectedGroup, array_values($roleChoices), true)) {
                $form->get('role')->addError(new FormError('Groupe invalide.'));
            }
        }

        if ($form->isSubmitted() && $form->isValid()) {
            $user->setPassword($passwordHasher->hashPassword($user, (string) $form->get('plainPassword')->getData()));
            $entityManager->persist($user);
            $entityManager->flush();

            // 🔴 **S159f — la création écrit une APPARTENANCE, plus une ligne de
            // rôle.** Après le `flush()`, parce qu'il faut l'identifiant du
            // compte : `UserGroupRepository` écrit en DBAL, il ne connaît pas
            // l'objet en attente.
            // ⚠️ **Et rien n'est écrit pour « utilisateur »** : c'est une audience
            // RÉSOLUE, tout compte actif en fait partie sans ligne. C'est
            // précisément ce que l'ancienne inscription écrivait en double.
            $selectedGroup = (string) $form->get('role')->getData();
            foreach ($userGroups->all() as $group) {
                if ($group['key'] === $selectedGroup && !$group['virtual']) {
                    $userGroups->addMember($group['id'], (int) $user->getId());
                    break;
                }
            }

            $this->addFlash('success', ['flash.utilisateur_cree', ['%p1%' => $user->getEmail()]]);

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
        UsageAllowanceService $usageBudgets,
        AccountGuard $accountGuard,
        UserGroupRepository $userGroups,
        AudienceResolver $audiences,
    ): Response {
        $user = $users->find($id);
        if (!$user) {
            throw $this->createNotFoundException('Utilisateur introuvable');
        }

        // ⚠️ **S158b — les groupes de cette personne, la même appartenance vue par
        // l'autre bout.** On y pense aussi souvent que « qui est dans ce groupe »,
        // et c'est le même dépôt qui écrit : deux vues, pas deux surfaces.
        // 🔴 L'appartenance est l'UNION des lignes stockées, des rôles et de
        // l'audience `user` — `AudienceResolver` seul sait la calculer, et chaque
        // ligne est une appartenance écrite, et elle se retire d'ici.
        $memberKeys = array_flip($audiences->keysFor($user));
        $storedGroupIds = array_flip($userGroups->storedGroupIdsFor((int) $user->getId()));

        $groupRows = [];
        $joinable = [];
        foreach ($userGroups->all() as $row) {
            if (isset($memberKeys[$row['key']])) {
                // ⚠️ **Plus de distinction de source depuis S159h** : pour un
                // groupe non virtuel, l'appartenance est toujours une ligne — les
                // rôles ne sont plus une source indépendante, ils en dérivent.
                $groupRows[] = $row + ['stored' => isset($storedGroupIds[$row['id']])];
                continue;
            }
            // ⚠️ Une audience virtuelle ne s'ajoute pas : elle se résout depuis le
            // compte, et une ligne y serait lue par personne.
            if (!$row['virtual']) {
                $joinable[] = $row;
            }
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
            'personTypeForm' => $this->createForm(PersonTypeType::class, [
                'is_staff' => $user->isStaff(),
                'is_trainer' => $user->isTrainer(),
                'is_bookable' => $user->isBookable(),
            ], ['user_id' => $id])->createView(),
            'rfidLogs' => $logs->findBy(['utilisateur' => $user], ['createdAt' => 'DESC']),
            'progressions' => $progressions->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']),
            'reservations' => $reservations->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']),
            'usageLogs' => $usageLogs->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']),
            'physicalTrainingRows' => $physicalTrainingRows,
            'usageRightsSummary' => $usageRights->overview($user),
            // Staff answering "why was I refused?" need the same figures the
            // member sees, on the same screen they are already looking at.
            'usageBudgets' => $usageBudgets->summaryFor($user),
            // ⚠️ The verdict is read here so the panel can EXPLAIN a refusal
            // instead of hiding a button — the same guard the POST re-runs.
            'anonymiseRefusal' => $accountGuard->refusalFor($user),
            'groupRows' => $groupRows,
            'joinableGroups' => $joinable,
        ]);
    }

    /**
     * The operator's side of the right to erasure — for a request that arrived
     * by e-mail rather than through the member's own screen.
     *
     * ⚠️ It runs the **same** `AccountAnonymiser`, deliberately. A second
     * erasure path written for the admin is a second definition of what
     * "erased" means, and the one that gets forgotten is the one that leaves
     * personal data behind.
     *
     * ⚠️ POST only, CSRF checked, and it refuses through `AccountGuard` exactly
     * as the member's page does — an administrator erasing the last remaining
     * administrator locks the installation out just as effectively.
     */
    #[Route('/utilisateurs/{id}/anonymiser', name: 'app_admin_user_anonymise', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function anonymiseUser(
        int $id,
        Request $request,
        UtilisateurRepository $users,
        AccountAnonymiser $anonymiser,
        AccountGuard $guard,
        TranslatorInterface $translator,
    ): Response {
        $user = $users->find($id);
        if (!$user instanceof Utilisateur) {
            throw $this->createNotFoundException('Utilisateur introuvable');
        }

        if (!$this->isCsrfTokenValid('anonymise_' . $id, (string) $request->request->get('_token'))) {
            $this->addFlash('error', $translator->trans('account_delete.error_csrf'));

            return $this->redirectToRoute('app_admin_user_detail', ['id' => $id]);
        }

        $refusal = $guard->refusalFor($user);
        if ($refusal !== null) {
            $this->addFlash('error', $translator->trans(
                $refusal === AccountGuard::REFUSED_LAST_ADMIN
                    ? 'account_delete.refused_last_admin'
                    : 'account_delete.refused_already',
            ));

            return $this->redirectToRoute('app_admin_user_detail', ['id' => $id]);
        }

        $anonymiser->anonymise($user);
        $this->addFlash('success', $translator->trans('account_delete.done'));

        return $this->redirectToRoute('app_admin_users');
    }

    #[Route('/utilisateurs/{id}/person-type', name: 'app_admin_user_person_type', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function updatePersonType(
        int $id,
        Request $request,
        UtilisateurRepository $users,
        ReservationRepository $reservations,
        ReservationMailer $reservationMails,
        EntityManagerInterface $entityManager,
        UserGroupRepository $userGroups,
    ): Response {
        $user = $users->find($id);
        if (!$user instanceof Utilisateur) {
            throw $this->createNotFoundException('Utilisateur introuvable.');
        }

        // ⚠️ **S148, J-22 — trois cases, un `FormType`, et aucun refus possible.**
        // Toute combinaison est un état que l'opérateur a le droit de demander : ce
        // que la conversion apporte ici, c'est le balisage du thème, pas la
        // validation. Le jeton reste par utilisateur.
        $form = $this->createForm(PersonTypeType::class, [
            'is_staff' => $user->isStaff(),
            'is_trainer' => $user->isTrainer(),
            'is_bookable' => $user->isBookable(),
        ], ['user_id' => $id]);
        $form->handleRequest($request);

        if (!$form->isSubmitted() || !$form->isValid()) {
            $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');

            return $this->redirectToRoute('app_admin_user_detail', ['id' => $id]);
        }

        /** @var array<string, mixed> $data */
        $data = $form->getData();

        // 🔴 **S159e — « staff » et « formateur » s'écrivent désormais dans le
        // GROUPE, et la ligne de rôle héritée part avec.** C'est la première
        // moitié du contract : la table `UTILISATEUR_ROLE` cesse d'être écrite
        // pour ces deux-là, une personne à la fois, au moment où l'opérateur
        // touche la case.
        //
        // ⚠️ **Retirer la ligne de rôle n'est pas un extra, c'est ce qui rend la
        // case capable de dire NON.** `getRoles()` rend l'union : tant qu'une
        // ligne de rôle subsiste, décocher la case laisserait la personne staff.
        // Et c'est aussi ce qui redonne son bouton « Retirer » à la fiche du
        // groupe, qui le cache tant qu'un rôle est une seconde raison.
        $this->setPersonTypeGroup($user, 'staff', (bool) $data['is_staff'], $userGroups);
        $this->setPersonTypeGroup($user, 'trainers', (bool) $data['is_trainer'], $userGroups);

        // Being bookable is the admin's call; the person then owns their own
        // slots and durations. Turning it off cancels what was already booked —
        // leaving live appointments on a page nobody can reach would strand them.
        $wasBookable = $user->isBookable();
        $isBookable = (bool) $data['is_bookable'];
        $user->setBookable($isBookable);
        if ($wasBookable && !$isBookable) {
            $stranded = $reservations->findUpcomingActiveForReservable(ReservableType::User, $id);
            $reservations->cancelUpcomingForReservable(ReservableType::User, $id);
            $reservationMails->cancelledBatch($stranded);
        }

        $entityManager->flush();
        $this->addFlash('success', 'flash.type_de_personne_mis_a_jour');

        return $this->redirectToRoute('app_admin_user_detail', ['id' => $id]);
    }

    /**
     * Poser ou retirer « staff » / « formateur », dans le GROUPE (S159e).
     *
     * 🔴 **Et en retirant la ligne de rôle héritée, toujours.** `getRoles()` rend
     * l'union des deux sources : laisser la ligne de `UTILISATEUR_ROLE` en place
     * rendrait la case incapable de dire NON — on décocherait « staff » et la
     * personne resterait staff, par l'autre source, sans que rien ne le dise.
     *
     * ⚠️ **C'est le contract, fait une personne à la fois.** Chaque passage sur
     * cette case déplace un compte de l'ancienne source vers la nouvelle. Rien
     * n'oblige à tout migrer d'un coup, et l'union couvre ceux qui n'ont pas
     * encore bougé.
     *
     * ⚠️ **L'écriture du groupe passe par `UserGroupRepository`**, jamais par
     * l'ORM : c'est lui qui porte les gardes, et une seconde surface d'écriture
     * sur la même table est la faute que cette phase entière range.
     */
    private function setPersonTypeGroup(
        Utilisateur $user,
        string $groupKey,
        bool $shouldHave,
        UserGroupRepository $userGroups,
    ): void {
        $group = null;
        foreach ($userGroups->all() as $row) {
            if ($row['key'] === $groupKey) {
                $group = $row;
                break;
            }
        }

        if ($group !== null) {
            $stored = \in_array((int) $user->getId(), $userGroups->storedMemberIds($group['id']), true);
            try {
                if ($shouldHave && !$stored) {
                    $userGroups->addMember($group['id'], (int) $user->getId());
                } elseif (!$shouldHave && $stored) {
                    $userGroups->removeMember($group['id'], (int) $user->getId());
                }
            } catch (\Throwable $e) {
                // La garde du dernier administrateur, ou une table absente. On le
                // dit plutôt que d'écrire à moitié — mais on ne casse pas les
                // deux autres cases du formulaire pour autant.
                $this->addFlash('error', $e->getMessage());

                return;
            }
        }

        // ⚠️ **S159f — plus de ligne de rôle à retirer.** `getRoles()` ne lit plus
        // `UTILISATEUR_ROLE` et la migration `Version20260902100000` la supprime :
        // le groupe est la seule source, donc décocher la case suffit à dire non.
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
            $this->addFlash('error', 'flash.la_validation_physique_a_ete_refusee');
            return $this->redirectToRoute('app_admin_user_detail', ['id' => $userId]);
        }

        $physicalFormation = $qualification->getPhysicalFormation($formation);
        if (!$physicalFormation instanceof Formation) {
            $this->addFlash('error', 'flash.la_validation_physique_nest_pas_configuree');
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
                ? ['flash.formation_physique_badge', ['%p1%' => $formation->getBadge()?->getNom() ?? 'formation']]
                : ['flash.formation_physique_seuil', ['%p1%' => $status['overallPercent']]],
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
            $this->addFlash('success', ['flash.creation_ajoutee', ['%p1%' => $creation->getTitle()]]);

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
            $this->addFlash('success', ['flash.creation_mise_a_jour', ['%p1%' => $creation->getTitle()]]);

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
            $this->addFlash('error', 'flash.modification_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_creations');
        }

        $creation
            ->setIsPublished(!$creation->isPublished())
            ->setUpdatedAt(new \DateTimeImmutable());
        $entityManager->flush();
        $this->addFlash('success', ['flash.statut_de_mis_a_jour', ['%p1%' => $creation->getTitle()]]);

        return $this->redirectToRoute('app_admin_creations');
    }

    #[Route('/creations/{id}/toggle-pinned', name: 'app_admin_creation_toggle_pinned', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function toggleCreationPinned(Creation $creation, Request $request, EntityManagerInterface $entityManager): Response
    {
        if (!$this->isCsrfTokenValid('toggle_pinned_creation_' . $creation->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.modification_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_creations');
        }

        $creation
            ->setIsPinned(!$creation->isPinned())
            ->setUpdatedAt(new \DateTimeImmutable());
        $entityManager->flush();
        $this->addFlash('success', ['flash.epinglage_de_mis_a_jour', ['%p1%' => $creation->getTitle()]]);

        return $this->redirectToRoute('app_admin_creations');
    }

    /**
     * The default page of Configuration — five short cards, each saved on its own.
     *
     * ⚠️ **S132 split one form into five** (roadmap: "cartes courtes et liées …
     * avec résumé d'état et sauvegarde par section"). It was a single 180-line
     * `<form>` with one Save at the bottom covering ten unrelated headings, which
     * had two costs. The visible one: no way to change the timezone without
     * re-submitting the alert banner, the vocabulary, the rules and the privacy
     * roles as collateral. The invisible one, and the reason this is a fix and not
     * a re-style — **every write sat inside `if (array_key_exists($locale, …))`**,
     * so a POST carrying an unrecognised language silently discarded the timezone,
     * the vocabulary, the rules, the roles and the banner along with it, and
     * flashed only "Langue invalide."
     *
     * Each card posts its own `section`; nothing outside that section is read or
     * written. The dangerous control — usage-rights enforcement — keeps its
     * preflight and its confirmation and now also keeps a disclosure, so it is not
     * one stray click away from refusing every member at once.
     */
    #[Route('/settings', name: 'app_admin_settings', methods: ['GET', 'POST'])]
    public function settings(
        Request $request,
        SiteSettingService $siteSettings,
        UsagePackageRepository $usagePackages,
        UsageCapabilityRegistry $usageCapabilities,
        TranslatorInterface $translator,
        LocaleCatalog $locales,
        UserGroupRepository $userGroups,
    ): Response
    {
        $availableLocales = $locales->choices();
        $capabilityKeys = array_map(static fn ($capability): string => $capability->key, $usageCapabilities->all());
        $rightsReadiness = $usagePackages->readiness($capabilityKeys, new \DateTimeImmutable('now', new \DateTimeZone($siteSettings->getTimezone())));

        // The operator's own role list, not a hardcoded set: a deployment that added
        // "formateur" must be able to tick it here. Mapped through the same helper the
        // firewall will later be asked about, so the two cannot drift.
        // ⚠️ Remonté au-dessus des formulaires en S147 : la carte « Exploitation » en a
        // besoin pour construire ses cases, pas seulement le rendu.
        // 🔴 **S159f — construits depuis les GROUPES intégrés.** La table `ROLE`
        // n'accorde plus rien ; la lire ici aurait offert un réglage portant sur
        // des noms que plus personne ne détient.
        // ⚠️ La valeur reste le rôle de SÉCURITÉ (`securityRole`) : c'est ce que
        // `BookingIdentityPolicy` compare, et ce n'est pas ce pas-ci qui change
        // son vocabulaire.
        $assignableRoles = [];
        foreach ($userGroups->all() as $group) {
            if ($group['builtin'] && !$group['virtual']) {
                $assignableRoles[] = [
                    'label' => $group['label'],
                    'securityRole' => Utilisateur::securityRoleFor(
                        $group['key'] === 'trainers' ? 'trainer' : $group['key'],
                    ),
                ];
            }
        }

        // ⚠️ **S147, J-22 — cinq cartes, cinq formulaires, et c'était déjà le cas.**
        // La page n'a jamais été un seul gros formulaire depuis S132 : chaque carte
        // poste la sienne avec un champ `section`. La conversion garde cette forme
        // et remplace le `switch` qui lisait `$request->request->get()` à la main.
        //
        // 🔴 **Ce que la conversion change vraiment, c'est le refus.** Avant, un
        // fuseau horaire inconnu affichait un message en haut de page et le champ
        // revenait silencieusement à sa valeur enregistrée. Maintenant le formulaire
        // est **re-rendu tel que soumis** : l'erreur est sur le champ, et ce que
        // l'opérateur avait choisi est encore là. C'est le point 8, structurel au
        // lieu d'accidentel.
        $forms = [
            'general' => $this->createForm(GeneralSettingsType::class, [
                'org_name' => $siteSettings->getOrgName(),
                'venue_label' => $siteSettings->getVenueLabel(),
                'lab_pages_nav_label' => $siteSettings->getLabPagesNavLabel(),
            ]),
            'localisation' => $this->createForm(LocalisationSettingsType::class, [
                'default_locale' => $siteSettings->getDefaultLocale(),
                'timezone' => $siteSettings->getTimezone(),
            ], ['available_locales' => $availableLocales]),
            'alerts' => $this->createForm(AlertsSettingsType::class, [
                'alert_banner_enabled' => $siteSettings->isAlertBannerEnabled(),
                'alert_banner_text' => $siteSettings->getAlertBannerText(),
            ]),
            'operations' => $this->createForm(OperationsSettingsType::class, [
                'lab_rules_html' => $siteSettings->getLabRulesHtml(),
                'lab_rules_pdf_url' => $siteSettings->getLabRulesPdfUrl(),
                'booking_identity_roles' => $siteSettings->getBookingIdentityRoles(),
                'regenerate_ical_token' => false,
            ], ['assignable_roles' => $assignableRoles]),
            'advanced' => $this->createForm(AdvancedSettingsType::class, [
                'development_mode' => $siteSettings->isDevelopmentMode(),
                'usage_rights_enforced' => $siteSettings->isUsageRightsEnforced(),
                'usage_rights_confirm_enable' => false,
            ]),
        ];

        // 🔴 **S150 — quelle carte a été refusée, et le gabarit doit le savoir.**
        // Deux de ces cartes ont désormais un `<details>` (règle 1) : le texte du
        // bandeau et le flux iCal. Un repli qui reste FERMÉ sur un refus cache à la
        // fois l'erreur et ce qui vient d'être tapé — la règle 1 le dit en toutes
        // lettres. Le drapeau vient d'ici et jamais de `form.vars.submitted` lu dans
        // le gabarit : `prod` n'a pas `strict_variables`, une variable absente y vaut
        // `null` en silence, et le repli resterait fermé sans que rien ne le signale.
        $refusedEditor = null;

        foreach ($forms as $section => $form) {
            $form->handleRequest($request);
            if (!$form->isSubmitted()) {
                continue;
            }

            if (!$form->isValid()) {
                // ⚠️ On REND, on ne redirige pas : c'est toute la différence.
                // Rediriger renverrait chercher les valeurs en base et effacerait ce
                // qui vient d'être tapé.
                $refusedEditor = $section;
                break;
            }

            $data = $form->getData();

            switch ($section) {
                case 'general':
                    $siteSettings->setVocabulary((string) $data['org_name'], (string) $data['venue_label']);
                    $siteSettings->setLabPagesNavLabel((string) $data['lab_pages_nav_label']);
                    break;

                case 'localisation':
                    $siteSettings->setDefaultLocale((string) $data['default_locale']);
                    // ⚠️ Un fuseau vide n'efface pas le réglage : le contrôleur
                    // l'ignorait déjà, et la contrainte du type le laisse passer.
                    if (($data['timezone'] ?? '') !== '') {
                        $siteSettings->setTimezone((string) $data['timezone']);
                    }
                    break;

                case 'alerts':
                    $siteSettings->setAlertBanner((bool) $data['alert_banner_enabled'], (string) $data['alert_banner_text']);
                    break;

                case 'operations':
                    $siteSettings->setLabRules((string) $data['lab_rules_html'], (string) $data['lab_rules_pdf_url']);
                    // ⚠️ « Les rôles cochés », donc tout décocher enregistre « personne
                    // d'autre que le propriétaire de la réservation » plutôt que de
                    // retomber sur le défaut — un choix auquel l'opérateur a droit.
                    $siteSettings->setBookingIdentityRoles(array_map('strval', (array) ($data['booking_identity_roles'] ?? [])));
                    if ((bool) ($data['regenerate_ical_token'] ?? false)) {
                        $siteSettings->regenerateIcalFeedToken();
                        $this->addFlash('success', 'flash.jeton_des_flux_ical_regenere_les');
                    }
                    break;

                case 'advanced':
                    $siteSettings->setDevelopmentMode((bool) $data['development_mode']);
                    // 🔴 La règle reste ici, pas dans le type : elle dépend de l'état
                    // ACTUEL du réglage et d'un décompte calculé plus haut. Une
                    // contrainte qui aurait besoin des deux serait une seconde copie
                    // de la décision, et deux copies divergent.
                    $enableRights = (bool) $data['usage_rights_enforced'];
                    if ($enableRights && !$siteSettings->isUsageRightsEnforced()) {
                        if ($rightsReadiness['packages'] < 1 || $rightsReadiness['members'] < 1) {
                            $enableRights = false;
                            $form->get('usage_rights_enforced')->addError(new FormError($translator->trans('usage_rights.settings_not_ready')));
                        } elseif (!(bool) $data['usage_rights_confirm_enable']) {
                            $enableRights = false;
                            $form->get('usage_rights_confirm_enable')->addError(new FormError($translator->trans('usage_rights.settings_confirmation_required')));
                        }
                    }
                    if (!$form->isValid()) {
                        // ⚠️ Les deux erreurs ajoutées juste au-dessus vivent DANS le
                        // repli « Contrôle des droits d'usage ». Sans ce drapeau, le
                        // refus s'affichait dans une boîte fermée.
                        $refusedEditor = $section;
                        break 2;
                    }
                    $siteSettings->setUsageRightsEnforced($enableRights);
                    break;
            }

            if ($form->isValid()) {
                $this->addFlash('success', 'flash.reglages_mis_a_jour');

                // Back to the card that was saved, not to the top of the page.
                return $this->redirectToRoute('app_admin_settings', ['_fragment' => $section]);
            }
            break;
        }

        // ---- S150 : les deux lignes de conséquence de cet écran (règle 4) ----
        //
        // ⚠️ **Calculées ici, jamais dans le gabarit.** Une conséquence dit ce que le
        // formulaire s'apprête à FAIRE, donc elle se lit sur les données du
        // FORMULAIRE et pas sur ce qu'il y a en base : sinon, sur un refus, la page
        // affiche l'ancien réglage sous le champ qui porte le nouveau, et se
        // contredit elle-même à un centimètre d'écart.

        // 1 · Le fuseau — l'heure qu'il est vraiment dans la zone CHOISIE.
        // 🔴 `getData()` rend `null` quand la valeur soumise n'est pas dans la liste
        // (requête forgée, base tz du serveur plus ancienne) : on retombe sur le
        // réglage enregistré plutôt que de construire un `DateTimeZone('')`, qui
        // lève. La validation, elle, a déjà posé son erreur sur le champ.
        $clockZone = (string) ($forms['localisation']->get('timezone')->getData() ?? '');
        if ($clockZone === '' || !SiteSettingService::isValidTimezone($clockZone)) {
            $clockZone = $siteSettings->getTimezone();
        }
        // 🔴 **Formaté ICI, et surtout pas par un filtre.** `|lab_date()` reprojette
        // dans le fuseau ENREGISTRÉ : appliqué à une heure déjà calculée dans le
        // fuseau *choisi*, il annulerait précisément ce qu'on veut montrer, et il le
        // ferait en silence. `|date()` sans argument tomberait, lui, dans le fuseau
        // par défaut de Twig. La zone n'est appliquée qu'une fois, ci-dessus, et le
        // gabarit ne reçoit qu'une chaîne — il n'y a plus de piège à rater.
        $clockTime = (new \DateTimeImmutable('now', new \DateTimeZone($clockZone)))->format('H:i');

        // 2 · Les droits d'usage — QUELLES fonctionnalités l'activation fermerait.
        // Le préflight disait déjà « %packages% packages couvrent %members% membres » ;
        // ce qu'il ne disait pas, c'est *laquelle* tombe à zéro. Une capacité sans un
        // seul membre couvert est une capacité que l'enregistrement ferme à tout le
        // monde d'un coup, et c'est la seule chose de cet écran qui mérite d'être
        // écrite avant le clic plutôt qu'après.
        $rightsGapKeys = [];
        foreach ($usageCapabilities->all() as $capability) {
            if ((int) ($rightsReadiness['coverage'][$capability->key] ?? 0) < 1) {
                $rightsGapKeys[] = $capability->labelKey;
            }
        }

        return $this->render('site/admin-settings.html.twig', [
            'forms' => array_map(static fn ($form) => $form->createView(), $forms),
            'refusedEditor' => $refusedEditor,
            'clockZone' => $clockZone,
            'clockTime' => $clockTime,
            'usageRightsGapKeys' => $rightsGapKeys,
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
            'assignableRoles' => $assignableRoles,
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
     *
     * ⚠️ **S132 removed the last of the second vocabulary.** The page grouped
     * itself under Ressources / Activités / Annuaires — a taxonomy that exists
     * nowhere else in the product — and described each switch's effect in prose.
     * It reads `FeatureSurfaces` now: the order is the workspace order the
     * sidebar uses, and the effect of a switch is computed by building the
     * navigation without it. Nothing on this page is written down twice.
     */
    #[Route('/features', name: 'app_admin_features', methods: ['GET', 'POST'])]
    public function features(Request $request, SiteFeatureService $features, SiteFeatureRegistry $registry, FeatureAdvice $advice, FeatureSurfaces $surfaces): Response
    {
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_features', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

                return $this->redirectToRoute('app_admin_features');
            }

            $checked = (array) $request->request->all('features');
            foreach ($registry->keys() as $key) {
                $features->setEnabled($key, in_array($key, $checked, true));
            }
            $this->addFlash('success', 'flash.fonctionnalites_mises_a_jour');

            return $this->redirectToRoute('app_admin_features');
        }

        return $this->render('site/admin-features.html.twig', [
            'registry' => $registry,
            // ⚠️ `surfaces` replaced `grouped` (S132). The old value was the
            // catalogue's three-way split; this one is the workspace order plus,
            // per feature, the destinations that stop existing without it.
            'surfaces' => $surfaces->all(),
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
    public function design(Request $request, VenueContext $venueContext, MachineRepository $machines): Response
    {
        // ⚠️ S134h — the "#filtres" section renders the REAL filter component, so
        // it needs the shape a real list passes it. Fabricated numbers on purpose:
        // this is a style guide, not a report, and a live count here would make
        // the page disagree with itself the moment a machine is added. What must
        // be real is the STRUCTURE — the same keys `/admin/machines` sends.
        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);

        return $this->render('site/admin-design.html.twig', [
            'demoFilters' => [
                'tiles' => [
                    ['label' => 'admin_list.all', 'label_is_key' => true, 'count' => 11, 'query' => ['category' => ''], 'active' => true],
                    ['label' => 'Découpe', 'count' => 2, 'query' => ['category' => 'decoupe']],
                    ['label' => 'Fraisage', 'count' => 1, 'query' => ['category' => 'fraisage']],
                    ['label' => 'Impression 3D', 'count' => 4, 'query' => ['category' => 'impression-3d']],
                    ['label' => 'Textile', 'count' => 1, 'query' => ['category' => 'textile']],
                    ['label' => 'Usinage', 'count' => 1, 'query' => ['category' => 'usinage']],
                    ['label' => 'Électronique', 'count' => 2, 'query' => ['category' => 'electronique']],
                ],
                'refine' => [
                    ['name' => 'statut', 'label' => 'Statut', 'value' => '', 'options' => [
                        ['value' => '', 'label' => 'Tous'],
                        ['value' => 'machines.st_available', 'label' => 'Disponible (8)'],
                        ['value' => 'machines.st_maintenance', 'label' => 'Maintenance (2)'],
                        ['value' => 'machines.st_broken', 'label' => 'Panne (1)'],
                    ]],
                    ['name' => 'niveau', 'label' => 'Niveau', 'value' => '', 'options' => [
                        ['value' => '', 'label' => 'Tous'],
                        ['value' => '1', 'label' => 'Niveau 1'],
                        ['value' => '2', 'label' => 'Niveau 2'],
                        ['value' => '3', 'label' => 'Niveau 3'],
                    ]],
                ],
                // Real, so the page shows what THIS install sees: on a
                // single-location install the dropdown is absent here too, which
                // is the behaviour the guide is claiming.
                'venue_context' => $context,
                'count_of' => '11 machine(s)',
            ],
            // ⚠️ S134i asks for the column vocabulary shown with REAL data, and
            // it is the right demand: a media cell proves nothing against a
            // placeholder rectangle, and a title cell proves nothing until a row
            // has no name. Four real machines, so the page shows what this
            // install's own data does to each type — including the empty cases,
            // which is where the six partials earn their keep.
            'demoRows' => array_slice($machines->findBy([], ['nom' => 'ASC']), 0, 4),
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
    public function wizard(Request $request, SiteSettingService $siteSettings, FirstRun $firstRun, LocaleCatalog $locales): Response
    {
        // ⚠️ **S147, J-22 — le bouton « passer » est lu AVANT le formulaire.**
        // Il poste `action=skip` hors du formulaire, et c'est ce qui le sauve : si on
        // le laissait passer par la validation, un champ mal rempli empêcherait de
        // sauter une étape qu'on a explicitement le droit de sauter.
        $skipping = $request->isMethod('POST') && $request->request->get('action') === 'skip';
        if ($skipping && !$this->isCsrfTokenValid('admin_wizard', (string) $request->request->all('admin_wizard')['_token'] ?? '')) {
            $skipping = false;
        }

        $wizardForm = $this->createForm(WizardType::class, [
            'org_name' => $siteSettings->getOrgName(),
            'venue_label' => $siteSettings->getVenueLabel(),
            'public_base_url' => $siteSettings->getPublicBaseUrl(),
            'lab_address' => $siteSettings->getLabAddress(),
            'timezone' => $siteSettings->getTimezone(),
            'default_locale' => $siteSettings->getDefaultLocale(),
        ], ['available_locales' => $locales->choices()]);
        $wizardForm->handleRequest($request);

        if ($skipping || ($wizardForm->isSubmitted() && $wizardForm->isValid())) {
            // Skipping is a real answer, not a failure: an operator who already
            // knows the settings screens should be able to say so once.
            if (!$skipping) {
                $data = $wizardForm->getData();
                $siteSettings->setVocabulary((string) $data['org_name'], (string) $data['venue_label']);
                $siteSettings->setPublicBaseUrl((string) $data['public_base_url']);
                $siteSettings->setLabAddress((string) $data['lab_address']);
                $siteSettings->setDefaultLocale((string) $data['default_locale']);
                // Asked here because the box runs UTC and the fallback is Europe/Paris:
                // an install anywhere else would otherwise show every recorded
                // timestamp in the wrong zone until somebody thought to look.
                if (($data['timezone'] ?? '') !== '') {
                    $siteSettings->setTimezone((string) $data['timezone']);
                }
            }

            $firstRun->markCompleted();
            $this->addFlash('success', $skipping
                ? 'flash.config_initiale_passee'
                : 'flash.config_initiale_enregistree');

            return $this->redirectToRoute('app_admin_setup');
        }

        return $this->render('site/admin-wizard.html.twig', [
            'wizardForm' => $wizardForm->createView(),
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
                $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

                return $this->redirectToRoute('app_admin_missing_pages');
            }

            if ($request->request->get('action') === 'clear') {
                $this->addFlash('success', ['flash.journal_vide_adresse_s_oubliee_s', ['p1' => $log->clear()]]);
            } else {
                $this->addFlash('success', ['flash.adresse_s_inactive_s_depuis_jours', ['p1' => $log->prune()]]);
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
        $this->addFlash('info', 'flash.les_portails_ont_ete_retires_cette');
        // ⚠️ **S159 — la cible a changé, pas le sens** : la page « structure » était une
        // MAQUETTE, retirée avec les autres. Les portails ayant été remplacés par les
        // LIEUX, c'est la liste des lieux qui répond à « où sont passés les portails ».
        // La redirection reste permanente : ces URLs n'ont jamais cessé d'être mortes.
        return $this->redirectToRoute('app_admin_venues', [], Response::HTTP_MOVED_PERMANENTLY);

        /*
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_portals', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

                return $this->redirectToRoute('app_admin_portals');
            }

            try {
                if ($request->request->get('action') === 'delete') {
                    $id = $request->request->getInt('id');
                    $name = $portals->find($id)?->name ?? '';
                    $removed = $portals->delete($id);
                    $this->addFlash('success', ['flash.portail_supprime_avec_reglage_s_qui', ['p1' => $name, 'p2' => $removed]]);
                } else {
                    $id = $portals->create(
                        trim((string) $request->request->get('name')),
                        trim((string) $request->request->get('slug')),
                        (string) $request->request->get('hostname'),
                    );
                    $this->addFlash('success', 'flash.portail_cree_choisissez_maintenant_ce_qu');

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
        // ⚠️ **S159 — la cible a changé, pas le sens** : la page « structure » était une
        // MAQUETTE, retirée avec les autres. Les portails ayant été remplacés par les
        // LIEUX, c'est la liste des lieux qui répond à « où sont passés les portails ».
        // La redirection reste permanente : ces URLs n'ont jamais cessé d'être mortes.
        return $this->redirectToRoute('app_admin_venues', [], Response::HTTP_MOVED_PERMANENTLY);
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
        // ⚠️ **S159 — la cible a changé, pas le sens** : la page « structure » était une
        // MAQUETTE, retirée avec les autres. Les portails ayant été remplacés par les
        // LIEUX, c'est la liste des lieux qui répond à « où sont passés les portails ».
        // La redirection reste permanente : ces URLs n'ont jamais cessé d'être mortes.
        return $this->redirectToRoute('app_admin_venues', [], Response::HTTP_MOVED_PERMANENTLY);

        /*
        $portal = $portals->find($id);
        if ($portal === null) {
            throw $this->createNotFoundException();
        }

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_portal_edit', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

                return $this->redirectToRoute('app_admin_portal_edit', ['id' => $id]);
            }

            try {
                if ($request->request->get('action') === 'delete') {
                    if ($portal->isDefault) {
                        throw new \LogicException('Le portail par défaut ne peut pas être supprimé.');
                    }
                    $name = $portal->name;
                    $removed = $portals->delete($id);
                    $this->addFlash('success', ['flash.portail_supprime_avec_reglage_s_qui', ['p1' => $name, 'p2' => $removed]]);

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

                $this->addFlash('success', 'flash.portail_enregistre');
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
     * Mail, as three readable tasks (S132): state, preferences, journal.
     *
     * It was one scroll — a notice, the sender account, a test send, five
     * reminder switches, two count strips and a log — with `<h2 style="margin-top">`
     * separating them and no way to tell which Save belonged to which block. The
     * roadmap asked for three tasks and this is them, on the same card-and-anchor
     * shape `/admin/settings` now uses.
     *
     * ⚠️ **The journal filters are server-side.** It used to render
     * `recent(50)` — the fiftieth-most-recent message was the horizon, so a
     * question about last Tuesday could not be asked at all, and a client-side
     * search box over those same fifty rows would have hidden that rather than
     * fixed it.
     *
     * ⚠️ Secrets stay masked: `getMaskedTransportDsn()` is what reaches the page,
     * and `MailSettings::isMasked()` is what stops a re-post writing the dots back
     * over the real password.
     */
    #[Route('/emails', name: 'app_admin_emails', methods: ['GET', 'POST'])]
    public function emails(Request $request, MailSettings $mailSettings, MailLog $mailLog, Mailer $mailer, SiteFeatureService $modules, ReminderSettings $reminderSettings, ReminderLog $reminderLog, SiteSettingService $siteSettings): Response
    {
        // ⚠️ **S147, J-22 — trois cartes, trois `FormType`.** Le `action=` caché qui
        // distinguait les trois envois disparaît : chaque formulaire porte son nom et
        // son jeton, donc c'est celui qui a été soumis qui répond.
        $accountForm = $this->createForm(MailAccountType::class, [
            'mail_transport_dsn' => $mailSettings->getMaskedTransportDsn(),
            'mail_from_address' => $mailSettings->getFromAddress(),
            'mail_from_name' => $mailSettings->getFromName(),
            'mail_reply_to' => $mailSettings->getReplyTo(),
            'public_base_url' => $siteSettings->getPublicBaseUrl(),
            'mail_paused' => $mailSettings->isPaused(),
        ]);
        $testForm = $this->createForm(MailTestType::class, [
            'test_recipient' => $this->getUser() instanceof Utilisateur ? $this->getUser()->getEmail() : '',
        ]);
        $reminderData = ['reminder_booking_lead_hours' => $reminderSettings->getBookingLeadHours(),
            'reminder_loan_lead_days' => $reminderSettings->getLoanLeadDays(),
            'reminder_event_lead_hours' => $reminderSettings->getEventLeadHours()];
        foreach (ReminderSettings::KINDS as $kind) {
            $reminderData['reminder_' . $kind] = (bool) ($reminderSettings->all()[$kind] ?? false);
        }
        $remindersForm = $this->createForm(MailRemindersType::class, $reminderData);

        $accountForm->handleRequest($request);
        $testForm->handleRequest($request);
        $remindersForm->handleRequest($request);

        if ($accountForm->isSubmitted() && $accountForm->isValid()) {
            $data = $accountForm->getData();

            // The form shows the DSN with its password masked; posting it back unchanged
            // must not overwrite the stored password with dots.
            $dsn = (string) $data['mail_transport_dsn'];
            if (MailSettings::isMasked($dsn)) {
                $dsn = $mailSettings->getTransportDsn();
            }

            $mailSettings->save(
                $dsn,
                (string) $data['mail_from_address'],
                (string) $data['mail_from_name'],
                (string) $data['mail_reply_to'],
            );
            $siteSettings->setPublicBaseUrl((string) $data['public_base_url']);
            // A pause is a deliberate, visible state — not a side effect of
            // saving the form, so it is its own checkbox.
            $mailSettings->setPaused((bool) $data['mail_paused']);
            $this->addFlash('success', 'flash.compte_d_envoi_enregistre');

            return $this->redirectToRoute('app_admin_emails');
        }

        if ($testForm->isSubmitted() && $testForm->isValid()) {
            $recipient = trim((string) $testForm->getData()['test_recipient']);
            if ($recipient === '') {
                $recipient = $this->getUser() instanceof Utilisateur ? $this->getUser()->getEmail() : '';
            }

            $error = $mailer->sendNow($recipient, null, 'test', ['sent_at' => (new \DateTimeImmutable())->format('d/m/Y H:i')]);
            if ($error === null) {
                $this->addFlash('success', ['flash.e_mail_de_test_envoye_a', ['%p1%' => $recipient]]);
            } else {
                // ⚠️ S147, J-3 — ce flash était la dernière chaîne française en dur de
                // `src/`, ratée par le balayage parce qu'elle est concaténée.
                $this->addFlash('error', ['flash.echec_de_l_envoi', ['%p1%' => $error]]);
            }

            return $this->redirectToRoute('app_admin_emails');
        }

        if ($remindersForm->isSubmitted() && $remindersForm->isValid()) {
            $data = $remindersForm->getData();
            $enabled = [];
            foreach (ReminderSettings::KINDS as $kind) {
                $enabled[$kind] = (bool) ($data['reminder_' . $kind] ?? false);
            }

            $reminderSettings->save(
                $enabled,
                (int) $data['reminder_booking_lead_hours'],
                (int) $data['reminder_loan_lead_days'],
                (int) $data['reminder_event_lead_hours'],
            );
            $this->addFlash('success', 'flash.rappels_programmes_enregistres');

            return $this->redirectToRoute('app_admin_emails');
        }

        // 🔴 **S150 — quel des trois formulaires a été refusé.** Deux d'entre eux ont
        // maintenant un `<details>` (l'arrêt d'urgence, les options d'envoi, l'envoi
        // de test) et la règle 1 est catégorique : un repli DOIT se rouvrir sur un
        // refus, sinon l'opérateur ne voit ni son erreur ni ce qu'il a tapé. Le
        // drapeau vient d'ici — pas de `form.vars.submitted` dans le gabarit, `prod`
        // n'a pas `strict_variables` et le repli resterait fermé en silence.
        $refusedEditor = null;
        foreach (['account' => $accountForm, 'test' => $testForm, 'reminders' => $remindersForm] as $name => $form) {
            if ($form->isSubmitted() && !$form->isValid()) {
                $refusedEditor = $name;
                break;
            }
        }

        // ---- S150 : ce que les trois délais s'apprêtent à faire (règle 4) ----
        //
        // 🔴 **C'est l'arithmétique du VRAI service, pas une seconde copie.** Les
        // trois scanners balaient `now → now + délai` (`BookingReminderScanner::scan`,
        // `EventReminderScanner::scan`) et le prêt part de MINUIT (`today()` fait
        // `setTime(0,0)`), pas de l'instant courant. Une ligne de conséquence qui
        // recalculerait « à peu près » serait un mensonge affiché avec autorité.
        //
        // ⚠️ **Et ce n'est pas « le rappel part à T-24 h ».** Le scanner ramasse tout
        // ce qui commence dans la fenêtre, donc une réservation créée à l'intérieur
        // du délai est rappelée à la passe suivante quand même. La phrase dit donc
        // la FENÊTRE — ce qui est vrai — et pas un instant d'envoi — qui ne l'est pas.
        //
        // ⚠️ Le délai lu est celui du FORMULAIRE, pour qu'un refus n'affiche pas
        // l'ancien horizon sous le nouveau champ ; hors bornes, on retombe sur la
        // valeur enregistrée, que `ReminderSettings` a déjà bornée.
        $lead = static function (string $child, int $saved, int $min, int $max) use ($remindersForm): int {
            $typed = $remindersForm->get($child)->getData();

            return is_numeric($typed) && (int) $typed >= $min && (int) $typed <= $max ? (int) $typed : $saved;
        };
        $reminderNow = new \DateTimeImmutable('now');
        $bookingHorizon = $reminderNow->modify(sprintf('+%d hours', $lead('reminder_booking_lead_hours', $reminderSettings->getBookingLeadHours(), 1, 168)));
        $eventHorizon = $reminderNow->modify(sprintf('+%d hours', $lead('reminder_event_lead_hours', $reminderSettings->getEventLeadHours(), 1, 168)));
        $loanHorizon = $reminderNow->setTime(0, 0)->modify(sprintf('+%d days', $lead('reminder_loan_lead_days', $reminderSettings->getLoanLeadDays(), 0, 30)));

        // The journal's filter state. Read from the query so a filtered view is a
        // shareable address, per the roadmap's "URL explicable".
        $logDays = $request->query->getInt('days', 30);
        $logDays = in_array($logDays, [1, 7, 30, 0], true) ? $logDays : 30;
        $logStatus = (string) $request->query->get('status', '');
        $logStatus = in_array($logStatus, [MailLog::STATUS_QUEUED, MailLog::STATUS_SENT, MailLog::STATUS_FAILED], true) ? $logStatus : '';
        $logRecipient = trim((string) $request->query->get('recipient', ''));

        return $this->render('site/admin-emails.html.twig', [
            'accountForm' => $accountForm->createView(),
            'testForm' => $testForm->createView(),
            'remindersForm' => $remindersForm->createView(),
            'refusedEditor' => $refusedEditor,
            'bookingHorizon' => $bookingHorizon,
            'eventHorizon' => $eventHorizon,
            'loanHorizon' => $loanHorizon,
            'mailPaused' => $mailSettings->isPaused(),
            'configured' => $mailSettings->isConfigured(),
            'transportDsn' => $mailSettings->getMaskedTransportDsn(),
            'fromAddress' => $mailSettings->getFromAddress(),
            'fromName' => $mailSettings->getFromName(),
            'replyTo' => $mailSettings->getReplyTo(),
            'logs' => $mailLog->search($logDays, $logStatus ?: null, $logRecipient, 200),
            'logMatching' => $mailLog->countMatching($logDays, $logStatus ?: null, $logRecipient),
            'logDays' => $logDays,
            'logStatus' => $logStatus,
            'logRecipient' => $logRecipient,
            'optOutable' => NotificationCategory::OPTOUTABLE,
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
        // ⚠️ **The bare URL is canonicalised, exactly as `/admin/reservations`
        // already was (S141).** One route serves three sections — machine, space
        // and person quotas — told apart by this parameter, so with it missing
        // `NavBuilder` can match no entry: the sub-menu strip lit nothing and,
        // once the page heading became the menu entry's name, the band came out
        // with an empty `<h1>`. Defaulting silently in PHP left the URL saying
        // one thing and the navigation another; redirecting makes the address,
        // the lit entry and the title agree.
        $selectedType = ReservableType::tryParse($request->query->getString('reservableType'));
        if ($selectedType === null && $request->isMethod('GET')) {
            return $this->redirectToRoute('app_admin_booking_policies', ['reservableType' => ReservableType::Machine->value]);
        }
        $selectedType ??= ReservableType::Machine;
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('admin_booking_policies', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

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

            $this->addFlash('success', 'flash.quotas_de_reservation_enregistres');

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
            $this->addFlash('error', 'flash.suppression_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_creations');
        }

        $title = $creation->getTitle();

        if ($creation->isArchived()) {
            $creation->restore();
            $entityManager->flush();
            $this->addFlash('success', ['flash.element_restaure', ['%p1%' => $title]]);

            return $this->redirectToRoute('app_admin_creations');
        }

        // 🔴 **Les fichiers RESTENT, et c'est le cœur de la différence.** L'ancienne
        // action effaçait l'image, sa vignette et le fichier projet du disque : une
        // restauration n'aurait rendu qu'une carte vide. Un projet archivé sort de la
        // galerie et du classement en gardant de quoi revenir.
        // ⚠️ `deleteCreationUploadIfSafe()` n'est donc plus appelée d'ici. Elle reste
        // le bon outil pour une purge définitive — qui n'existe pas encore, et qui est
        // la seule chose qui devrait jamais toucher ces fichiers.
        $creation->archive();
        $entityManager->flush();
        $this->addFlash('success', ['flash.element_archive', ['%p1%' => $title]]);

        return $this->redirectToRoute('app_admin_creations');
    }

    #[Route('/badges', name: 'app_admin_badges', methods: ['GET'])]
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
            $this->addFlash('success', ['flash.badge_cree', ['%p1%' => $badge->getNom()]]);

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
            $this->addFlash('success', ['flash.badge_mis_a_jour', ['%p1%' => $badge->getNom()]]);

            return $this->redirectToRoute('app_admin_badges');
        }

        return $this->render('site/admin-badge-edit.html.twig', [
            'badge' => $badge,
            'form' => $form,
        ]);
    }

    /**
     * Retire a machine, and keep it (S134b).
     *
     * 🔴 **Phase G's exit criterion said every announced object must be archivable
     * from its workspace. A machine was not.** It could be created and edited and
     * then never removed: a laser cutter sold last year stayed in every catalogue,
     * every booking picker and every calendar, and the only way out was the database.
     *
     * ⚠️ **Archived, not deleted**, because `RESERVATION`, `LOG_UTILISATION` and
     * `ACCESS_RFID_LOG` point at it — deleting would take the usage history with it.
     * ⚠️ **And archiving is enforced server-side**, not merely hidden:
     * `ReservationService` refuses a booking on an archived machine, because
     * `/machines/{id}` still answers for anyone holding the link.
     */
    #[Route('/machines/{id}/archive', name: 'app_admin_machine_archive', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function archiveMachine(Machine $machine, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('archive_machine_' . $machine->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_machine_edit', ['id' => $machine->getId()]);
        }

        $restore = $request->request->getBoolean('restore');
        $restore ? $machine->restore() : $machine->archive();
        $entityManager->flush();

        $this->addFlash('success', $restore
            ? ['flash.machine_de_nouveau_au_parc', ['%p1%' => $machine->getNom()]]
            // ⚠️ Says what archiving does NOT do. An operator who reads "archivée" and
            // expects the past bookings to disappear has been misled by the word.
            : ['flash.machine_retiree_du_parc', ['%p1%' => $machine->getNom()]]);

        return $this->redirectToRoute('app_admin_machines');
    }

    /**
     * Retire a training, and keep it (S134b). Same gap, same rule.
     *
     * ⚠️ `PROGRESSION` points at it, and badges are awarded through it: deleting would
     * erase what people had done towards a qualification. Archiving takes it out of
     * the catalogue and out of the event form's session picker; the progressions and
     * the sessions already attached keep reading correctly.
     */
    #[Route('/formations/{id}/archive', name: 'app_admin_formation_archive', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function archiveFormation(Formation $formation, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('archive_formation_' . $formation->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_formation_edit', ['id' => $formation->getId()]);
        }

        $restore = $request->request->getBoolean('restore');
        $restore ? $formation->restore() : $formation->archive();
        $entityManager->flush();

        $this->addFlash('success', $restore
            ? ['flash.formation_de_nouveau_au_catalogue', ['%p1%' => $formation->getTitre()]]
            : ['flash.formation_archivee_catalogue', ['%p1%' => $formation->getTitre()]]);

        return $this->redirectToRoute('app_admin_formations');
    }

    #[Route('/badges/{id}/archive', name: 'app_admin_badge_archive', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function archiveBadge(Badge $badge, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('archive_badge_' . $badge->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.archivage_refuse_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_badge_edit', ['id' => $badge->getId()]);
        }

        $badge->archive();
        $entityManager->flush();
        $this->addFlash('success', ['flash.badge_archive_ses_attributions_restent_dans', ['%p1%' => $badge->getNom()]]);

        return $this->redirectToRoute('app_admin_badges');
    }

    #[Route('/badges/{id}/restore', name: 'app_admin_badge_restore', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function restoreBadge(Badge $badge, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');
        if (!$this->isCsrfTokenValid('restore_badge_' . $badge->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.restauration_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_badge_edit', ['id' => $badge->getId()]);
        }

        $badge->restore();
        $entityManager->flush();
        $this->addFlash('success', ['flash.badge_restaure', ['%p1%' => $badge->getNom()]]);

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
            $this->addFlash('success', ['flash.institution_creee', ['%p1%' => $institution->getNom()]]);

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
            $this->addFlash('success', ['flash.institution_mise_a_jour', ['%p1%' => $institution->getNom()]]);

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
            $this->addFlash('error', 'flash.suppression_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_institutions');
        }

        $name = $institution->getNom();

        if ($institution->isArchived()) {
            $institution->restore();
            $entityManager->flush();
            $this->addFlash('success', ['flash.element_restaure', ['%p1%' => $name]]);

            return $this->redirectToRoute('app_admin_institutions');
        }

        // Les membres gardent leur institution d'origine : la supprimer effaçait
        // la provenance de comptes qui existent toujours.
        $institution->archive();
        $entityManager->flush();
        $this->addFlash('success', ['flash.element_archive', ['%p1%' => $name]]);

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
            $this->addFlash('success', ['flash.page_creee', ['%p1%' => $page->getTitre()]]);

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
            $this->addFlash('success', ['flash.page_mise_a_jour', ['%p1%' => $page->getTitre()]]);

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
            $this->addFlash('error', 'flash.suppression_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_lab_pages');
        }

        $title = $page->getTitre();

        if ($page->isArchived()) {
            $page->restore();
            $entityManager->flush();
            $this->addFlash('success', ['flash.element_restaure', ['%p1%' => $title]]);

            return $this->redirectToRoute('app_admin_lab_pages');
        }

        // ⚠️ **Archiver n'est pas dépublier**, et une page a déjà les deux : dépubliée,
        // elle est cachée et revient d'un clic ; archivée, elle sort de l'édition
        // courante. Ses images restent sur le disque — `deleteLabPageImageFileIfSafe()`
        // sert encore à retirer UNE image d'une page, ce qui est une autre action.
        $page->archive();
        $entityManager->flush();
        $this->addFlash('success', ['flash.element_archive', ['%p1%' => $title]]);

        return $this->redirectToRoute('app_admin_lab_pages');
    }

    #[Route('/lab-pages/{id}/photos', name: 'app_admin_lab_page_photo_add', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function addLabPagePhoto(LabPage $page, Request $request, EntityManagerInterface $entityManager, SluggerInterface $slugger, ImageNormalizer $images): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('lab_page_photo_' . $page->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.ajout_refuse_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
        }

        $uploadedFile = $request->files->get('photo');
        if (!$uploadedFile instanceof UploadedFile) {
            $this->addFlash('error', 'flash.choisissez_une_photo_a_ajouter');

            return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
        }

        $extension = strtolower($uploadedFile->guessExtension() ?: $uploadedFile->getClientOriginalExtension() ?: 'bin');
        if ($extension === 'jpeg') {
            $extension = 'jpg';
        }

        if (!in_array($extension, ['png', 'jpg', 'webp'], true)) {
            $this->addFlash('error', 'flash.image_png_jpg_jpeg_webp');

            return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
        }

        $uploadDir = $this->getParameter('kernel.project_dir') . '/public/uploads/lab-pages';
        if (!is_dir($uploadDir) && !mkdir($uploadDir, 0775, true) && !is_dir($uploadDir)) {
            $this->addFlash('error', 'flash.impossible_de_creer_le_dossier_des');

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
            $this->addFlash('error', 'flash.copie_photo_impossible');

            return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
        }

        $image = (new LabPageImage())->setLabPage($page)->setImageFilename($fileName);
        $entityManager->persist($image);
        $entityManager->flush();
        $this->addFlash('success', 'flash.photo_ajoutee');

        return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
    }

    #[Route('/lab-pages/{id}/photos/{photoId}/delete', name: 'app_admin_lab_page_photo_delete', requirements: ['id' => '\d+', 'photoId' => '\d+'], methods: ['POST'])]
    public function deleteLabPagePhoto(LabPage $page, int $photoId, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('lab_page_photo_delete_' . $photoId, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.suppression_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_lab_page_edit', ['id' => $page->getId()]);
        }

        foreach ($page->getImages() as $image) {
            if ($image->getId() === $photoId) {
                $filename = $image->getImageFilename();
                $entityManager->remove($image);
                $entityManager->flush();
                $this->deleteLabPageImageFileIfSafe($filename);
                $this->addFlash('success', 'flash.photo_supprimee');
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
            // ⚠️ S134h — the "3 sur 11" scope line beside the search box.
            'placeTotal' => count($allRows),
            'venueContext' => $context,
            'filters' => $filters,
            'placeCategoryTiles' => $this->categoryTiles($allRows, static fn (Place $place): ?string => $place->getCategory(), $filters['category']),
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
            $this->addFlash('success', ['flash.espace_cree', ['%p1%' => $place->getNom()]]);

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
            $this->addFlash('success', ['flash.espace_mis_a_jour', ['%p1%' => $place->getNom()]]);

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
            $this->addFlash('error', 'flash.suppression_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_places');
        }

        $name = $place->getNom();

        if ($place->isArchived()) {
            $place->restore();
            $entityManager->flush();
            $this->addFlash('success', ['flash.element_restaure', ['%p1%' => $name]]);

            return $this->redirectToRoute('app_admin_places');
        }

        // ⚠️ **S147, J-2 — le verbe change, la promesse ne change pas.** Les
        // réservations ne portent pas de clé étrangère vers l'espace, donc rien ne
        // cascade et rien ne cascadait déjà : annuler celles À VENIR reste explicite,
        // et les inscrits sont prévenus. C'est précisément la règle de S134f, et elle
        // était déjà tenue ici — seule la destruction de la ligne disparaît.
        // Les réservations passées restent, avec le nom de la ressource figé dans
        // `reservableLabel`.
        $stranded = $reservations->findUpcomingActiveForReservable(ReservableType::Place, $place->getId());
        $cancelled = $reservations->cancelUpcomingForReservable(ReservableType::Place, $place->getId());
        $reservationMails->cancelledBatch($stranded);

        $place->archive();
        $entityManager->flush();
        $this->addFlash('success', $cancelled > 0
            ? ['flash.espace_archive_avec_annulations', ['p1' => $name, 'p2' => $cancelled]]
            : ['flash.element_archive', ['%p1%' => $name]]);

        return $this->redirectToRoute('app_admin_places');
    }

    #[Route('/events', name: 'app_admin_events', methods: ['GET'])]
    public function events(Request $request, EventRepository $events, VenueContext $venueContext, LabClock $labClock): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        // ⚠️ **S133 — "Ailleurs / externe" is an explicit choice here.** An event
        // can happen somewhere this installation does not run; those rows carry
        // `venue = NULL`, and with only "all" and a location to pick from they
        // were folded in beside the onsite ones with no way to isolate or exclude
        // them. The roadmap's words: "jamais mêlés en silence".
        $context = $venueContext->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null, allowOffsite: true);

        $query = mb_strtolower(mb_substr(trim($request->query->getString('q')), 0, 80));
        $period = in_array($request->query->getString('type'), ['upcoming', 'past'], true) ? $request->query->getString('type') : '';
        // ⚠️ Convention B: `Event` dates are human-entered wall-clock, stored as the
        // digits the operator typed. `new DateTimeImmutable()` is the server instant in
        // UTC, so comparing the two was off by the lab's offset and an event changed tab
        // up to two hours early or late around midnight. `storedFormOf()` gives "now" in
        // the same digits the column holds, which is what these rows actually mean.
        $now = $labClock->storedFormOf($labClock->now());
        // Three criteria, not two: a venue, no venue at all, or every one of them.
        // ⚠️ `['venue' => null]` is a real criterion in Doctrine (`venue IS NULL`)
        // and is NOT the same as `[]`; conflating them is exactly how offsite
        // events became invisible as a set.
        $criteria = match (true) {
            $context['offsite'] => ['venue' => null],
            $context['selected'] !== null => ['venue' => $context['selected']],
            default => [],
        };
        $allRows = $events->findBy($criteria, ['dateDebut' => 'DESC']);
        $rows = array_values(array_filter(
            $allRows,
            static function (Event $event) use ($query, $period, $now): bool {
                $end = $event->getDateFin() ?? $event->getDateDebut();
                return ($query === '' || str_contains(mb_strtolower($event->getTitre() . ' ' . ($event->getLieu() ?? '')), $query))
                    && ($period === '' || ($period === 'past' ? $end < $now : $end >= $now));
            },
        ));

        return $this->render('site/admin-events.html.twig', [
            'events' => $rows,
            // ⚠️ S134h — the tiles count these, not `events`. Counting the
            // filtered rows made "À venir" and "Passés" report the same number as
            // soon as either was selected.
            'allEvents' => $allRows,
            'venueContext' => $context,
            'filters' => ['q' => $query, 'type' => $period],
            // The template compares against this, never Twig's `date()`, which would
            // reintroduce the server-UTC bug in the tile counts.
            'now' => $now,
        ]);
    }

    /**
     * ⚠️ **"Every week, ×4" creates four events, here and now** (S146d) — not a
     * recurrence rule evaluated at read time. Each one then moves, fills up or gets
     * called off individually, which is what a course actually does and what a rule
     * cannot express. See `App\\Calendar\\EventSeries`.
     */
    #[Route('/events/new', name: 'app_admin_event_new', methods: ['GET', 'POST'])]
    public function newEvent(
        Request $request,
        EntityManagerInterface $entityManager,
        VenueRepository $venues,
        EventSeries $series,
    ): Response {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $event = new Event();
        $form = $this->createForm(EventAdminType::class, $event, ['allow_repeat' => true]);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            // ⚠️ This forced `requireDefaultVenue()` on every onsite event AFTER the form
            // had bound, silently discarding the location the operator picked. Offsite
            // events genuinely have none; onsite ones keep the choice, falling back to the
            // default only when none was made (S133).
            if (!$event->isOnsite()) {
                $event->setVenue(null);
            } elseif ($event->getVenue() === null) {
                $event->setVenue($this->requireDefaultVenue($venues));
            }
            $entityManager->persist($event);

            // ⚠️ Generated from the event AFTER the venue fallback above, so every
            // occurrence carries the same location the operator ended up with rather
            // than the null they submitted.
            $extras = $series->extraOccurrences(
                $event,
                (string) ($form->get('repeatEvery')->getData() ?? EventSeries::NONE),
                (int) ($form->get('repeatCount')->getData() ?? 1),
            );
            foreach ($extras as $occurrence) {
                $entityManager->persist($occurrence);
            }

            // ⚠️ One flush for the whole series: half a course in the database because
            // the fourth row failed is worse than none of it.
            $entityManager->flush();

            $this->addFlash('success', $extras === []
                ? ['flash.evenement_cree', ['%p1%' => $event->getTitre()]]
                : ['flash.seances_creees', [
                    '%p1%' => count($extras) + 1,
                    '%p2%' => $event->getTitre(),
                    '%p3%' => $event->getDateDebut()?->format('d/m/Y') ?? '',
                    '%p4%' => end($extras)->getDateDebut()?->format('d/m/Y') ?? '',
                ]]);

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
            // had bound, silently discarding the location the operator picked. Offsite
            // events genuinely have none; onsite ones keep the choice, falling back to the
            // default only when none was made (S133).
            if (!$event->isOnsite()) {
                $event->setVenue(null);
            } elseif ($event->getVenue() === null) {
                $event->setVenue($this->requireDefaultVenue($venues));
            }
            $entityManager->flush();
            $this->addFlash('success', ['flash.evenement_mis_a_jour', ['%p1%' => $event->getTitre()]]);

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
                $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

                return $this->redirectToRoute('app_admin_event_registrations', ['id' => $event->getId()]);
            }

            $action = (string) $request->request->get('action');

            // Door desk: toggle attendance. Deliberately not a one-way switch —
            // the commonest thing that happens at a door is tapping the wrong row.
            if ($action === 'checkin' || $action === 'undo_checkin') {
                $row = $registrations->find($request->request->getInt('registration'));

                if ($row === null || $row->getEvent()?->getId() !== $event->getId()) {
                    $this->addFlash('error', 'flash.inscription_introuvable_pour_cet_evenement');
                } elseif ($action === 'checkin' && !$row->isCheckInEligible()) {
                    $this->addFlash('error', 'flash.seules_les_personnes_inscrites_hors_liste');
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
                    $this->addFlash('error', 'flash.inscription_introuvable_pour_cet_evenement');
                } else {
                    $result = $registrationService->cancel($row);
                    $this->addFlash(
                        $result->ok ? 'success' : 'error',
                        $result->ok
                            ? ['flash.inscription_annulee_prevenue', ['%p1%' => $row->getDisplayName()]]
                            : (string) $result->message,
                    );
                }

                return $this->redirectToRoute('app_admin_event_registrations', ['id' => $event->getId()]);
            }

            $notified = $registrationService->callOff($event, (string) $request->request->get('reason'));
            $this->addFlash('success', ['flash.evenement_annule_personne_s_prevenue_s', ['p1' => $notified]]);

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
            $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

            return $back();
        }

        $uploadDir = $this->getParameter('kernel.project_dir') . '/public/uploads/events';
        $previous = $event->getPosterFilename();

        if ($request->request->get('action') === 'remove') {
            $event->setPosterFilename(null);
            $entityManager->flush();
            $this->deletePosterFile($uploadDir, $previous);
            $this->addFlash('success', 'flash.affiche_retiree');

            return $back();
        }

        $uploadedFile = $request->files->get('poster');
        if (!$uploadedFile instanceof UploadedFile) {
            $this->addFlash('error', 'flash.choisir_une_image');

            return $back();
        }

        $extension = strtolower($uploadedFile->guessExtension() ?: $uploadedFile->getClientOriginalExtension() ?: 'bin');
        if ($extension === 'jpeg') {
            $extension = 'jpg';
        }

        if (!in_array($extension, ['png', 'jpg', 'webp'], true)) {
            $this->addFlash('error', 'flash.image_png_jpg_webp');

            return $back();
        }

        if (!is_dir($uploadDir) && !mkdir($uploadDir, 0775, true) && !is_dir($uploadDir)) {
            $this->addFlash('error', 'flash.impossible_de_creer_le_dossier_des_2');

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
            $this->addFlash('error', 'flash.copie_image_impossible');

            return $back();
        }

        $event->setPosterFilename($fileName);
        $entityManager->flush();

        // Only after the new one is safely in place and recorded.
        $this->deletePosterFile($uploadDir, $previous);
        $this->addFlash('success', 'flash.affiche_mise_a_jour');

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
            $this->addFlash('error', 'flash.suppression_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_events');
        }

        $name = $event->getTitre();

        if ($event->isArchived()) {
            $event->restore();
            $entityManager->flush();
            $this->addFlash('success', ['flash.element_restaure', ['%p1%' => $name]]);

            return $this->redirectToRoute('app_admin_events');
        }

        // ⚠️ **Archiver n'est pas annuler, et les deux existent.** `callOff()`
        // prévient les inscrits d'une séance qui n'aura pas lieu ; archiver retire
        // de l'affiche une séance dont on ne veut plus parler. Supprimer emportait
        // les inscriptions et, depuis S146e, les progressions qu'elles avaient
        // créées — une qualification effacée parce qu'on rangeait un calendrier.
        $event->archive();
        $entityManager->flush();
        $this->addFlash('success', ['flash.element_archive', ['%p1%' => $name]]);

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
            $this->addFlash('success', ['flash.materiau_cree', ['%p1%' => $material->getName()]]);

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
            $this->addFlash('success', ['flash.materiau_mis_a_jour', ['%p1%' => $material->getName()]]);

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
            $this->addFlash('error', 'flash.suppression_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_materials');
        }

        $name = $material->getName();

        if ($material->isArchived()) {
            $material->restore();
            $entityManager->flush();
            $this->addFlash('success', ['flash.element_restaure', ['%p1%' => $name]]);

            return $this->redirectToRoute('app_admin_materials');
        }

        $material->archive();
        $entityManager->flush();
        $this->addFlash('success', ['flash.element_archive', ['%p1%' => $name]]);

        return $this->redirectToRoute('app_admin_materials');
    }

    #[Route('/loanable-items', name: 'app_admin_loanable_items', methods: ['GET'])]
    public function loanableItems(Request $request, LoanableItemRepository $items, LoanRepository $loans, VenueContext $venueContext): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        // ⚠️ S131. `LoanableItem` has carried a non-nullable `venueId` since S107, but
        // this list ignored it — so on a two-venue install every object appeared on
        // both, and the location control was missing from the one Prêts screen that
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
            $this->addFlash('success', ['flash.objet_cree', ['%p1%' => $item->getName()]]);

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
            $this->addFlash('success', ['flash.objet_mis_a_jour', ['%p1%' => $item->getName()]]);

            return $this->redirectToRoute('app_admin_loanable_items');
        }

        return $this->render('site/admin-loanable-item-edit.html.twig', [
            'item' => $item,
            'form' => $form,
        ]);
    }

    /**
     * Retiring a loan object — archive, never delete (S133).
     *
     * ⚠️ **The route name and its CSRF token are unchanged on purpose**, so no
     * template, no bookmark and no half-deployed page ends up pointing at a route
     * that no longer exists. What changed is what it does: it used to
     * `remove($item)`, and `LOAN.itemId` cascades, so retiring a battery pack
     * destroyed the record of every borrowing of it — the "deleted (and its
     * loans)" in its own success message was accurate and nobody read it that way.
     *
     * ⚠️ It also **refuses while the object is out**. Archiving is reversible and
     * an out loan still resolves, but an operator archiving something that is
     * physically in somebody's bag has almost certainly picked the wrong row.
     */
    #[Route('/loanable-items/{id}/delete', name: 'app_admin_loanable_item_delete', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function archiveLoanableItem(LoanableItem $item, Request $request, EntityManagerInterface $entityManager, LoanRepository $loans): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('delete_loanable_item_' . $item->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_loanable_items');
        }

        $name = $item->getName();

        if ($item->isArchived()) {
            $item->restore();
            $entityManager->flush();
            $this->addFlash('success', ['flash.objet_reactive_il_est_de_nouveau', ['%p1%' => $name]]);

            return $this->redirectToRoute('app_admin_loanable_items');
        }

        $out = (int) ($loans->activeCountsByItem()[$item->getId()] ?? 0);
        if ($out > 0) {
            $this->addFlash('error', ['flash.est_encore_sorti_pret_s_en_cours', ['p1' => $name, 'p2' => $out]]);

            return $this->redirectToRoute('app_admin_loanable_items');
        }

        $item->archive();
        $entityManager->flush();
        $this->addFlash('success', ['flash.objet_archive_il_quitte_le_catalogue', ['%p1%' => $name]]);

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
            $this->addFlash('success', ['flash.pret_enregistre_pour', ['%p1%' => $loan->getBorrowerDisplay()]]);

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
            $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_loans');
        }

        $loan
            ->setStatus(Loan::STATUS_RETURNED)
            ->setActualReturnDate(new \DateTimeImmutable('today'))
            ->setConditionReturn((string) $request->request->get('conditionReturn') ?: null);
        $entityManager->flush();
        $this->addFlash('success', 'flash.pret_marque_comme_rendu');

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
            $this->addFlash('success', ['flash.tache_de_maintenance_creee', ['%p1%' => $task->getTitle()]]);

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
            $this->addFlash('success', ['flash.tache_de_maintenance_mise_a_jour', ['%p1%' => $task->getTitle()]]);

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
            $this->addFlash('success', ['flash.tache_s_de_maintenance_creee_s', ['p1' => $count]]);

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
            $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

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
        $this->addFlash('success', 'flash.tache_marquee_comme_faite');

        return $this->redirectToRoute('app_admin_maintenance');
    }

    #[Route('/maintenance/{id}/delete', name: 'app_admin_maintenance_delete', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function deleteMaintenance(MaintenanceTask $task, Request $request, EntityManagerInterface $entityManager): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('delete_maintenance_' . $task->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.suppression_refusee_token_csrf_invalide');

            return $this->redirectToRoute('app_admin_maintenance');
        }

        // Une tâche faite est la preuve qu'une machine a été entretenue.
        if ($task->isArchived()) {
            $task->restore();
            $entityManager->flush();
            $this->addFlash('success', 'flash.tache_de_maintenance_restauree');

            return $this->redirectToRoute('app_admin_maintenance');
        }

        $task->archive();
        $entityManager->flush();
        $this->addFlash('success', 'flash.tache_de_maintenance_archivee');

        return $this->redirectToRoute('app_admin_maintenance');
    }

    /**
     * The badge journal — period, reader, machine, result, and the detail on demand.
     *
     * ⚠️ **S132 moved the filtering to the server.** The page fetched the hundred
     * most recent rows and narrowed them in the browser, so on a door badged fifty
     * times a day the hundredth row is yesterday afternoon: a question about last
     * Tuesday had no answer, and a tile row sitting on those hundred rows made it
     * look as though it did.
     *
     * ⚠️ **The fallback path keeps its old behaviour on purpose.** When the reader
     * columns predate their migration the controller answers with raw arrays, and
     * none of these filters can be expressed against a schema that has no
     * `readerId`. That install gets the unfiltered hundred and the migration
     * notice it already had, rather than controls that would quietly do nothing.
     */
    #[Route('/access-rfid-logs', name: 'app_admin_access_rfid_logs', methods: ['GET'])]
    public function accessRfidLogs(Request $request, AccessRfidLogRepository $logs, RfidReaderRepository $readers, MachineRepository $machines, EntityManagerInterface $entityManager): Response
    {
        $readerColumnsExist = $this->accessRfidLogReaderColumnsExist($entityManager);

        if (!$readerColumnsExist) {
            return $this->render('site/admin-access-rfid-logs.html.twig', [
                'logs' => $this->findAccessRfidLogsWithoutReaderColumns($entityManager),
                'readerColumnsExist' => false,
                'logDays' => 0,
                'logReader' => '',
                'logMachine' => '',
                'logResult' => '',
                'logMatching' => 0,
                'resultCounts' => ['all' => 0, 'yes' => 0, 'no' => 0],
                'readerOptions' => [],
                'machineOptions' => [],
            ]);
        }

        $days = $request->query->getInt('days', 30);
        $days = in_array($days, [1, 7, 30, 0], true) ? $days : 30;
        $readerId = $request->query->getInt('reader') ?: null;
        $machineId = $request->query->getInt('machine') ?: null;
        $result = (string) $request->query->get('result', '');
        $result = in_array($result, ['yes', 'no'], true) ? $result : '';

        return $this->render('site/admin-access-rfid-logs.html.twig', [
            'logs' => $logs->search($days, $readerId, $machineId, $result ?: null, 200),
            'logMatching' => $logs->countMatching($days, $readerId, $machineId, $result ?: null),
            'resultCounts' => $logs->resultCounts($days, $readerId, $machineId),
            'readerColumnsExist' => true,
            'logDays' => $days,
            'logReader' => $readerId ?? '',
            'logMachine' => $machineId ?? '',
            'logResult' => $result,
            'readerOptions' => $readers->findForAdmin(),
            'machineOptions' => $machines->findBy([], ['nom' => 'ASC']),
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

            $this->addFlash('success', ['flash.lecteur_rfid_cree_vous_pouvez_maintenant', ['%p1%' => $reader->getName()]]);

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

            $this->addFlash('success', ['flash.lecteur_rfid_mis_a_jour', ['%p1%' => $reader->getName()]]);

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
            $this->addFlash('error', 'flash.jeton_csrf_invalide_suppression_annulee');

            return $this->redirectToRoute('app_admin_rfid_readers');
        }

        $readerName = $reader->getName();

        if ($reader->isArchived()) {
            $reader->restore();
            $entityManager->flush();
            $this->addFlash('success', ['flash.element_restaure', ['%p1%' => $readerName]]);

            return $this->redirectToRoute('app_admin_rfid_readers');
        }

        // ⚠️ Le `try/catch` disparaît avec la suppression : il n'était là que pour
        // rattraper la violation de clé étrangère des journaux d'accès qui pointent
        // sur le lecteur. Archiver ne peut pas la déclencher — et ces journaux
        // gardent enfin de quoi dire de quelle porte ils parlaient.
        $reader->archive();
        $entityManager->flush();
        $this->addFlash('success', ['flash.element_archive', ['%p1%' => $readerName]]);

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

    /** @return OpeningHour[] */
    private function ensureOpeningHourRows(OpeningHourRepository $openingHours, ScheduleResolver $schedule, \App\Entity\Venue $venue, EntityManagerInterface $entityManager): array
    {
        $existingRows = $openingHours->findOrdered($venue);
        $existingByDay = [];
        foreach ($existingRows as $row) {
            $existingByDay[$row->getDayOfWeek()] = $row;
        }

        // ⚠️ Seeded from the DEFAULT venue's week, which is what a
            // location without rows was already being judged against. Seeding
            // from its own (absent) rows would write the built-in week over an
            // install that had deliberately changed it.
            foreach ($schedule->rowsFor(null) as $fallbackRow) {
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
            ?? throw new \LogicException('Le lieu par défaut est introuvable.');
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
            // 🔴 S134c: this used to group by the raw `statut` string, so an install
            // holding both `disponible` and `idle` — which the shipped data does —
            // got two tiles for one state, labelled with the stored words in two
            // languages. Grouped by display key now; `Machine` owns the mapping and
            // `statusFilterForKey()` makes the link select the same rows the tile
            // counted.
            $key = $machine->getStatusKey();
            $category = str_replace('machines.st_', '', $key);
            $counts[$key] ??= [
                'label' => $key,
                'label_is_key' => true,
                'count' => 0,
                'category' => $category,
                'query' => $baseQuery + ['statut' => $key],
                'active' => $filters['statut'] === $key,
            ];
            $counts[$key]['count']++;
        }

        uasort($counts, static fn (array $left, array $right): int => strnatcasecmp($left['label'], $right['label']));

        array_unshift($counts, [
            'label' => 'admin_list.all',
            'label_is_key' => true,
            'count' => count($machines),
            'category' => 'all',
            // ⚠️ S134h: `'statut' => ''` is explicit, not noise. The filter
            // component merges a tile's query INTO the current one, so a tile
            // that simply omits the key it owns cannot clear it — "Toutes" would
            // leave the previous status in place and count rows it does not show.
            // The empty value is stripped from the URL when the link is built.
            'query' => $baseQuery + ['statut' => ''],
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
            // ⚠️ Same as above: the tile that means "no category filter" has to
            // say so, or merging leaves the old one behind.
            'query' => $baseQuery + ['category' => ''],
            'active' => $filters['category'] === '',
        ]);

        return array_values($counts);
    }

    /**
     * ⚠️ `$selected` is not optional decoration: without it no tile carries
     * `active`, so on `/admin/places` clicking a category narrowed the list and
     * **nothing on screen said which tile was on** — the row stayed uniformly
     * unselected while the results changed underneath it. The client-side lists
     * (materials, loanable items) toggle `is-on` in the Stimulus controller
     * instead, so they pass nothing and keep the old behaviour.
     *
     * @param array<int, object> $items @return array<int, array{label: string, count: int, category: string}>
     */
    private function categoryTiles(array $items, callable $categoryForItem, string $selected = ''): array
    {
        $counts = [];
        foreach ($items as $item) {
            $label = trim((string) $categoryForItem($item));
            if ($label === '') {
                continue;
            }
            $category = str_replace([' ', '_'], '-', mb_strtolower($label));
            $counts[$category] ??= [
                'label' => $label,
                'count' => 0,
                'category' => $category,
                'query' => ['category' => $label],
                'active' => $selected !== '' && $selected === $label,
            ];
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
    /**
     * Les groupes intégrés qu'un écran peut proposer comme « rôle » (S159f).
     *
     * 🔴 **Construit depuis les GROUPES, plus depuis la table `ROLE`.** Depuis la
     * fusion, l'appartenance à un groupe intégré EST le rôle ; lire l'ancienne
     * table ici aurait laissé un écran proposer un choix que plus rien n'écrit.
     *
     * ⚠️ **La valeur reste la clé du GROUPE**, pas un `ROLE_*` : c'est elle que
     * `UserGroupRepository` attend, et traduire au dernier moment est ce qui a
     * produit les orthographes divergentes qu'on vient de ranger.
     *
     * @return array<string, string> libellé => clé de groupe
     */
    private function buildAdminRoleChoices(UserGroupRepository $groups): array
    {
        $available = [];
        foreach ($groups->all() as $group) {
            // ⚠️ Les audiences résolues n'ont pas de membres à inscrire : les
            // offrir serait un choix qui n'écrit rien.
            if ($group['builtin'] && !$group['virtual']) {
                $available[$group['label']] = $group['key'];
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
                    'title_key' => $log->isAuthorized() ? 'admin_dashboard.act_rfid_allowed' : 'admin_dashboard.act_rfid_denied',
                    'message_key' => 'admin_dashboard.act_rfid_message',
                    'user' => $user?->getDisplayName(),
                    'machine' => $machine?->getNom(),
                    'badge' => $log->getBadgeUid(),
                    'date' => $log->getCreatedAt(),
                ];
            }
        } catch (\Throwable $e) {
            // La migration readerId/readerToken peut ne pas encore être appliquée.
        }

        foreach ($reservations->findBy([], ['created' => 'DESC'], 5) as $reservation) {
            $activities[] = [
                'type' => 'reservation',
                'title_key' => 'admin_dashboard.act_reservation',
                'message_key' => 'admin_dashboard.act_reservation_message',
                'user' => $reservation->getUtilisateur()?->getDisplayName(),
                'resource' => $reservation->getReservableLabel() ?: null,
                'date' => $reservation->getCreated(),
            ];
        }

        foreach ($progressions->findBy([], ['dateDebut' => 'DESC'], 5) as $progression) {
            $activities[] = [
                'type' => 'formation',
                'title_key' => $progression->isCompleted() ? 'admin_dashboard.act_formation_done' : 'admin_dashboard.act_formation_started',
                'message_key' => 'admin_dashboard.act_formation_message',
                'user' => $progression->getUtilisateur()?->getDisplayName(),
                'formation' => $progression->getFormation()?->getTitre(),
                'score' => $progression->getScore(),
                'date' => $progression->getDateEnd() ?? $progression->getDateDebut(),
            ];
        }

        foreach ($usageLogs->findBy([], ['createdAt' => 'DESC'], 5) as $usageLog) {
            $activities[] = [
                'type' => 'usage',
                'title_key' => 'admin_dashboard.act_usage',
                'message_key' => 'admin_dashboard.act_usage_message',
                'user' => $usageLog->getUtilisateur()?->getDisplayName(),
                'machine' => $usageLog->getMachine()?->getNom(),
                'source' => $usageLog->getSource(),
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
