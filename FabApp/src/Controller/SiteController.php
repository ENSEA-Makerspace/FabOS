<?php

namespace App\Controller;

use App\Entity\Badge;
use App\Entity\Creation;
use App\Entity\CreationVote;
use App\Entity\Event;
use App\Entity\LabPage;
use App\Entity\Place;
use App\Entity\Reservation;
use App\Form\CreationUserType;
use App\Repository\AccessRfidLogRepository;
use App\Repository\BadgeRepository;
use App\Repository\CreationRepository;
use App\Repository\CreationVoteRepository;
use App\Repository\EventRepository;
use App\Repository\FormationRepository;
use App\Repository\LabPageRepository;
use App\Repository\LogUtilisationRepository;
use App\Repository\PlaceRepository;
use App\Repository\MachineFavoriteRepository;
use App\Repository\MachineRepository;
use App\Repository\LoanRepository;
use App\Repository\MaterialRepository;
use App\Repository\ProgressionRepository;
use App\Repository\ReservationRepository;
use App\Repository\SectionRepository;
use App\Repository\QuizRepository;
use App\Repository\QuestionRepository;
use App\Repository\RoleRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Entity\Formation;
use App\Entity\Machine;
use App\Entity\Role;
use App\Entity\Utilisateur;
use App\Entity\UtilisateurRole;
use App\Repository\UtilisateurRepository;
use App\Service\FormationPageContentService;
use App\Service\GuidedTrainingService;
use App\Service\MachineQualificationService;
use App\Service\OpeningHoursProvider;
use App\Service\QuizCatalogService;
use App\Service\TrainingQualificationService;
use App\Service\TrainingPolicyService;
use App\Entity\HomepageUserPreference;
use App\Repository\HomepageUserPreferenceRepository;
use App\Service\HomepagePersonalizationService;
use App\Service\HomepageVisibilityService;
use App\Service\ModuleService;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Form\FormError;
use Symfony\Component\Form\FormInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\File\Exception\FileException;
use Symfony\Component\HttpFoundation\File\UploadedFile;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\String\Slugger\SluggerInterface;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Core\User\PasswordAuthenticatedUserInterface;
use Symfony\Component\Security\Http\Attribute\IsGranted;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;

final class SiteController extends AbstractController
{
    #[Route('/', name: 'app_home', methods: ['GET'])]
    #[Route('/index.html', name: 'app_home_html', methods: ['GET'])]
    public function home(
        UtilisateurRepository $users,
        MachineRepository $machines,
        FormationRepository $formations,
        ProgressionRepository $progressions,
        ReservationRepository $reservations,
        AccessRfidLogRepository $rfidLogs,
        BadgeRepository $badges,
        UtilisateurBadgeRepository $userBadges,
        OpeningHoursProvider $openingHours,
        MachineFavoriteRepository $favorites,
        HomepageVisibilityService $homepageVisibility,
        HomepagePersonalizationService $homepagePersonalization,
        EventRepository $events,
        ModuleService $modules,
    ): Response
    {
        $currentUser = $this->getUser();
        $visibility = $homepageVisibility->getVisibilityMap($currentUser);
        $sectionOrder = $homepagePersonalization->getSectionOrder($currentUser, $visibility);
        $personalizationRows = $homepagePersonalization->getPersonalizationRows($currentUser, $visibility);
        $topUsers = [];
        $homeMachines = [];
        $homeMachinesMode = 'default';
        $homeStats = [
            'users' => 0,
            'machines' => 0,
            'formations' => 0,
            'reservations' => 0,
            'rfidLogs' => 0,
            'badges' => 0,
            'completedFormations' => 0,
        ];
        $latestRfidLogs = [];

        if ($visibility['mini_leaderboard'] ?? false) {
            foreach ($users->findBy([], ['tempsPresenceTotal' => 'DESC'], 3) as $index => $user) {
                $topUsers[] = [
                    'rank' => $index + 1,
                    'user' => $user,
                    'completedFormations' => $progressions->countCompletedVisible($user),
                    'rfidLogs' => $rfidLogs->count(['utilisateur' => $user]),
                    'badges' => $userBadges->count(['utilisateur' => $user]),
                ];
            }
        }

        if ($visibility['featured_machines'] ?? false) {
            $homeMachines = $machines->findBy([], ['createdAt' => 'DESC'], 6);
            if ($currentUser instanceof Utilisateur) {
                $favoriteMachines = $favorites->findMachinesForUser($currentUser, 6);
                if ($favoriteMachines !== []) {
                    $homeMachines = $favoriteMachines;
                    $homeMachinesMode = 'favorites';
                }
            }
        }

        if ($visibility['fablab_stats'] ?? false) {
            $homeStats = [
                'users' => $users->count([]),
                'machines' => $machines->count([]),
                'formations' => $formations->countVisible(),
                'reservations' => $reservations->count([]),
                'rfidLogs' => $rfidLogs->count([]),
                'badges' => $badges->count([]),
                'completedFormations' => $progressions->countCompletedVisible(),
            ];
        }

        if ($visibility['latest_rfid_logs'] ?? false) {
            $latestRfidLogs = $rfidLogs->findBy([], ['createdAt' => 'DESC'], 5);
        }

        $upcomingEvents = [];
        if (($visibility['upcoming_events'] ?? false) && $modules->isEnabled('events')) {
            $upcomingEvents = $events->findUpcoming(4);
        }

        return $this->render('site/index.html.twig', [
            'homeStats' => $homeStats,
            'machines' => $homeMachines,
            'homeMachinesMode' => $homeMachinesMode,
            'latestRfidLogs' => $latestRfidLogs,
            'topUsers' => $topUsers,
            'upcomingEvents' => $upcomingEvents,
            'openingHours' => $openingHours->getOpeningHours(),
            'homepageVisibility' => $visibility,
            'homepageSectionOrder' => $sectionOrder,
            'homepagePersonalizationRows' => $personalizationRows,
        ]);
    }

    #[Route('/homepage/preferences', name: 'app_homepage_preferences_update', methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function updateHomepagePreferences(
        Request $request,
        HomepageVisibilityService $homepageVisibility,
        HomepagePersonalizationService $homepagePersonalization,
        HomepageUserPreferenceRepository $preferences,
        EntityManagerInterface $entityManager,
    ): Response {
        if (!$this->isCsrfTokenValid('homepage_preferences', (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Impossible d’enregistrer la personnalisation : token invalide.');

            return $this->redirectToRoute('app_home');
        }

        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException('Authentification requise');
        }

        $rawOrder = trim((string) $request->request->get('section_order', ''));
        $decodedOrder = json_decode($rawOrder, true);
        if (!is_array($decodedOrder)) {
            $this->addFlash('error', 'Impossible d’enregistrer la personnalisation : ordre invalide.');

            return $this->redirectToRoute('app_home');
        }

        $visibility = $homepageVisibility->getVisibilityMap($user);
        $sectionOrder = $homepagePersonalization->sanitizeSubmittedOrder($decodedOrder, $visibility);
        $preference = $preferences->findOneForUser($user) ?? (new HomepageUserPreference())->setUser($user);
        $preference
            ->setSectionOrderArray($sectionOrder)
            ->setUpdatedAt(new \DateTimeImmutable());

        $entityManager->persist($preference);
        $entityManager->flush();

        $this->addFlash('success', 'Votre accueil a été personnalisé.');

        return $this->redirectToRoute('app_home');
    }

    #[Route('/calendrier', name: 'app_calendar', methods: ['GET'])]
    #[Route('/calendrier.html', name: 'app_calendar_html', methods: ['GET'])]
    #[Route('/calendar', name: 'app_calendar_legacy_en', methods: ['GET'])]
    #[Route('/calendar.html', name: 'app_calendar_legacy_en_html', methods: ['GET'])]
    public function calendar(
        MachineRepository $machines,
        ReservationRepository $reservations,
        OpeningHoursProvider $openingHours,
        MachineQualificationService $machineAccess,
        EventRepository $events,
        ModuleService $modules,
    ): Response {
        $machineRows = $machines->findBy([], ['nom' => 'ASC']);

        return $this->render('site/calendrier.html.twig', [
            'machines' => $machineRows,
            'reservations' => $reservations->findAllActive(['dateDebut' => 'ASC']),
            'openingHoursJson' => $openingHours->getOpeningHoursForJson(),
            'calendarStartHour' => $openingHours->getCalendarStartHour(),
            'calendarEndHour' => $openingHours->getCalendarEndHour(),
            'bookingAccessByMachine' => $this->buildCalendarBookingAccess($machineRows, $machineAccess),
            'upcomingEvents' => $modules->isEnabled('events') ? $events->findUpcoming(6) : [],
        ]);
    }

    #[Route('/mes-reservations', name: 'app_my_reservations', methods: ['GET'])]
    #[IsGranted('ROLE_USER')]
    public function myReservations(ReservationRepository $reservations): Response
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException('Authentification requise');
        }

        $items = $reservations->findForUser($user, ['dateDebut' => 'DESC']);
        $now = new \DateTimeImmutable('now', new \DateTimeZone('Europe/Paris'));
        $current = [];
        $upcoming = [];
        $past = [];
        $cancelled = [];
        $nextReservation = null;

        foreach ($items as $reservation) {
            if ($reservation->isCancelled()) {
                $cancelled[] = $reservation;
                continue;
            }

            if ($reservation->getDateFin() < $now) {
                $past[] = $reservation;
                continue;
            }

            if ($reservation->getDateDebut() <= $now && $reservation->getDateFin() >= $now) {
                $current[] = $reservation;
                continue;
            }

            $upcoming[] = $reservation;
            if ($nextReservation === null || $reservation->getDateDebut() < $nextReservation->getDateDebut()) {
                $nextReservation = $reservation;
            }
        }

        usort($current, static fn ($a, $b): int => $a->getDateDebut() <=> $b->getDateDebut());
        usort($upcoming, static fn ($a, $b): int => $a->getDateDebut() <=> $b->getDateDebut());
        usort($past, static fn ($a, $b): int => $b->getDateDebut() <=> $a->getDateDebut());
        usort($cancelled, static fn ($a, $b): int => $b->getDateDebut() <=> $a->getDateDebut());

        return $this->render('site/mes-reservations.html.twig', [
            'reservations' => $items,
            'currentReservations' => $current,
            'upcomingReservations' => $upcoming,
            'pastReservations' => $past,
            'cancelledReservations' => $cancelled,
            'nextReservation' => $nextReservation,
            'reservationStats' => [
                'current' => count($current),
                'upcoming' => count($upcoming),
                'past' => count($past),
                'cancelled' => count($cancelled),
            ],
            'now' => $now,
        ]);
    }

    #[Route('/machines', name: 'app_machines', methods: ['GET'])]
    #[Route('/machines.html', name: 'app_machines_html', methods: ['GET'])]
    #[Route('/machine', name: 'app_machine_legacy_singular', methods: ['GET'])]
    #[Route('/machine.html', name: 'app_machine_legacy_singular_html', methods: ['GET'])]
    public function machines(MachineRepository $machines, MachineFavoriteRepository $favorites): Response
    {
        $currentUser = $this->getUser();
        $favoriteMachineIds = $currentUser instanceof Utilisateur ? $favorites->findMachineIdsForUser($currentUser) : [];
        $machineRows = $machines->findBy([], ['createdAt' => 'DESC']);
        if ($favoriteMachineIds !== []) {
            $favoriteLookup = array_flip($favoriteMachineIds);
            usort($machineRows, static function ($a, $b) use ($favoriteLookup): int {
                $aFavorite = isset($favoriteLookup[$a->getId()]);
                $bFavorite = isset($favoriteLookup[$b->getId()]);

                return $aFavorite === $bFavorite ? 0 : ($aFavorite ? -1 : 1);
            });
        }

        return $this->render('site/machines.html.twig', [
            'machines' => $machineRows,
            'favoriteMachineIds' => $favoriteMachineIds,
            'favoritesEnabled' => $favorites->isStorageReady(),
        ]);
    }


    #[Route('/machine/new', name: 'app_machine_new_legacy', methods: ['GET'])]
    #[Route('/machines/new', name: 'app_machines_new_legacy', methods: ['GET'])]
    public function machineNewLegacy(MachineRepository $machines): Response
    {
        return $this->render('site/admin-machines.html.twig', [
            'machines' => $machines->findBy([], ['createdAt' => 'DESC']),
        ]);
    }

    #[Route('/machines/{id}', name: 'app_machine_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}', name: 'app_machine_detail_legacy_singular', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine-detail.html', name: 'app_machine_detail_html', methods: ['GET'])]
    public function machineDetail(
        Request $request,
        MachineRepository $machines,
        AccessRfidLogRepository $rfidLogs,
        LogUtilisationRepository $usageLogs,
        ReservationRepository $reservations,
        MachineQualificationService $machineAccess,
        MachineFavoriteRepository $favorites,
        MaterialRepository $materials,
        ModuleService $modules,
        ?int $id = null,
    ): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        $machine = $machines->find($id);
        if (!$machine) {
            throw $this->createNotFoundException('Machine introuvable');
        }

        $currentUser = $this->getUser();
        $favoritesEnabled = $favorites->isStorageReady();
        $isFavorite = $favoritesEnabled
            && $currentUser instanceof Utilisateur
            && $favorites->findOneForUserAndMachine($currentUser, $machine) !== null;
        $accessStatus = $machineAccess->getStatus(
            $machine,
            $currentUser instanceof Utilisateur ? $currentUser : null,
        );
        $requiredBadgeRows = $accessStatus['badgeRows'];
        $hasRequiredBadge = $accessStatus['authorized'];
        $authorizationStatus = $accessStatus['authorizationStatus'];

        return $this->render('site/machine-detail.html.twig', [
            'machine' => $machine,
            'requiredBadgeRows' => $requiredBadgeRows,
            'hasRequiredBadge' => $hasRequiredBadge,
            'authorizationStatus' => $authorizationStatus,
            'rfidLogCount' => $rfidLogs->count(['machine' => $machine]),
            'usageLogCount' => $usageLogs->count(['machine' => $machine]),
            'reservationCount' => $reservations->count(['machine' => $machine]),
            'favoritesEnabled' => $favoritesEnabled,
            'isFavorite' => $isFavorite,
            'materialsEnabled' => $modules->isEnabled('materials'),
            'machineMaterials' => $materials->findByMachine($machine->getId()),
        ]);
    }

    #[Route('/machines/{id}/calendrier', name: 'app_machine_calendar', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machines/{id}/calendar', name: 'app_machine_calendar_legacy_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/calendrier', name: 'app_machine_calendar_legacy_singular', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/calendar', name: 'app_machine_calendar_legacy_singular_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine-calendrier.html', name: 'app_machine_calendar_html', methods: ['GET'])]
    public function machineCalendar(
        Request $request,
        MachineRepository $machines,
        ReservationRepository $reservations,
        OpeningHoursProvider $openingHours,
        MachineQualificationService $machineAccess,
        ?int $id = null,
    ): Response {
        $id ??= max(1, (int) $request->query->get('id', 1));
        $machine = $machines->find($id);
        if (!$machine) {
            throw $this->createNotFoundException('Machine introuvable');
        }

        $bookingAccessByMachine = $this->buildCalendarBookingAccess([$machine], $machineAccess);

        return $this->render('site/machine-calendrier.html.twig', [
            'machine' => $machine,
            'machines' => $machines->findBy([], ['nom' => 'ASC']),
            'reservations' => $reservations->findActiveByMachine($machine, ['dateDebut' => 'ASC']),
            'openingHoursJson' => $openingHours->getOpeningHoursForJson(),
            'calendarStartHour' => $openingHours->getCalendarStartHour(),
            'calendarEndHour' => $openingHours->getCalendarEndHour(),
            'bookingAccess' => $bookingAccessByMachine[$machine->getId()] ?? null,
        ]);
    }

    #[Route('/machines/{id}/historique', name: 'app_machine_history', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machines/{id}/history', name: 'app_machine_history_legacy_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/historique', name: 'app_machine_history_legacy_singular', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/history', name: 'app_machine_history_legacy_singular_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine-historique.html', name: 'app_machine_history_html', methods: ['GET'])]
    public function machineHistory(Request $request, MachineRepository $machines, AccessRfidLogRepository $rfidLogs, LogUtilisationRepository $usageLogs, ReservationRepository $reservations, ?int $id = null): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        $machine = $machines->find($id);
        if (!$machine) {
            throw $this->createNotFoundException('Machine introuvable');
        }

        return $this->render('site/machine-historique.html.twig', [
            'machine' => $machine,
            'rfidLogs' => $rfidLogs->findBy(['machine' => $machine], ['createdAt' => 'DESC']),
            'usageLogs' => $usageLogs->findBy(['machine' => $machine], ['dateDebut' => 'DESC']),
            'reservations' => $reservations->findBy(['machine' => $machine], ['dateDebut' => 'DESC']),
        ]);
    }

    #[Route('/formations', name: 'app_formations', methods: ['GET'])]
    #[Route('/formations.html', name: 'app_formations_html', methods: ['GET'])]
    public function formations(
        FormationRepository $formations,
        ProgressionRepository $progressions,
        TrainingPolicyService $trainingPolicy,
    ): Response {
        $progressionStats = $this->buildFormationProgressionStats($progressions);
        $formationItems = $formations->findVisible(['id' => 'DESC']);
        $formationPolicies = [];
        $physicalRequiredCount = 0;

        foreach ($formationItems as $formation) {
            if ($formation->getId() === null) {
                continue;
            }

            $policy = $trainingPolicy->getFormationPolicy($formation);
            $formationPolicies[$formation->getId()] = $policy;
            if ($policy['physicalRequired']) {
                ++$physicalRequiredCount;
            }
        }

        return $this->render('site/formations.html.twig', [
            'formations' => $formationItems,
            'formationVisuals' => $this->buildFormationVisuals($formationItems),
            'formationPolicies' => $formationPolicies,
            'progressionStats' => $progressionStats,
            'catalogSummary' => [
                'total' => count($formationItems),
                'physicalRequired' => $physicalRequiredCount,
                'withoutPhysical' => max(0, count($formationItems) - $physicalRequiredCount),
            ],
        ]);
    }

    #[Route('/formations/{id}', name: 'app_formation_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/formation-detail.html', name: 'app_formation_detail_html', methods: ['GET'])]
    public function formationDetail(
        Request $request,
        FormationRepository $formations,
        ProgressionRepository $progressions,
        QuizRepository $quizzes,
        QuizCatalogService $quizCatalog,
        GuidedTrainingService $guidedTraining,
        FormationPageContentService $pageContent,
        TrainingPolicyService $trainingPolicy,
        ?int $id = null,
    ): Response {
        $id ??= max(1, (int) $request->query->get('id', 1));
        $formation = $formations->find($id);
        if (!$formation || TrainingQualificationService::isInternalCategory($formation->getCategorie())) {
            throw $this->createNotFoundException('Formation introuvable');
        }

        $user = $this->getUser();
        if ($user instanceof Utilisateur) {
            $guidedTraining->synchronizeParentProgress($formation, $user, false);
        }

        $sectionJourney = $guidedTraining->buildJourney(
            $formation,
            $user instanceof Utilisateur ? $user : null,
        );

        $requiredQuizCount = 0;
        foreach ($formations->findQuizFormationsForParent($formation->getId() ?? 0) as $quizFormation) {
            if ($quizCatalog->isSectionQuizFormation($quizFormation) || $quizCatalog->isBonusQuizFormation($quizFormation)) {
                continue;
            }
            $requiredQuizCount += $quizzes->count(['formation' => $quizFormation]);
        }

        // Compatibilité avec les anciennes bases : les quiz directement rattachés
        // à la formation restent considérés comme obligatoires.
        if ($requiredQuizCount === 0) {
            $requiredQuizCount = $quizzes->count(['formation' => $formation]);
        }

        $formationPolicy = $trainingPolicy->getFormationPolicy($formation);

        return $this->render('site/formation-detail.html.twig', [
            'formation' => $formation,
            'formationVisual' => $this->buildFormationVisual($formation),
            'formationPolicy' => $formationPolicy,
            'pageContent' => $pageContent->getContent($formation),
            'sectionJourney' => $sectionJourney,
            'journeyWeights' => [
                'training' => $requiredQuizCount > 0 ? 60 : 100,
                'quiz' => $requiredQuizCount > 0 ? 40 : 0,
            ],
            'progressions' => $progressions->findBy(['formation' => $formation], ['dateDebut' => 'DESC']),
            'stats' => $this->buildFormationProgressionStats($progressions)[$formation->getId()] ?? ['total' => 0, 'completed' => 0, 'incomplete' => 0],
        ]);
    }

    #[Route('/formations/{id}/suivi', name: 'app_formation_follow', requirements: ['id' => '\d+'], methods: ['GET'])]
    #[Route('/formation-suivi.html', name: 'app_formation_follow_html', methods: ['GET'])]
    public function formationFollow(
        Request $request,
        FormationRepository $formations,
        ProgressionRepository $progressions,
        SectionRepository $sections,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        MachineRepository $machines,
        QuizCatalogService $quizCatalog,
        TrainingQualificationService $qualification,
        TrainingPolicyService $trainingPolicy,
        GuidedTrainingService $guidedTraining,
        ?int $id = null,
    ): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        $formation = $formations->find($id);
        if (!$formation || TrainingQualificationService::isInternalCategory($formation->getCategorie())) {
            throw $this->createNotFoundException('Formation introuvable');
        }

        $formationSections = $sections->findJourneySections($formation);
        $requiredPageQuizzes = [];
        $bonusQuizzes = [];

        foreach ($formations->findQuizFormationsForParent($formation->getId() ?? 0) as $quizFormation) {
            if ($quizCatalog->isSectionQuizFormation($quizFormation)) {
                continue;
            }

            foreach ($quizzes->findBy(['formation' => $quizFormation], ['id' => 'ASC']) as $quiz) {
                if ($quizCatalog->isBonusQuizFormation($quizFormation)) {
                    $bonusQuizzes[] = $quiz;
                } else {
                    $requiredPageQuizzes[] = $quiz;
                }
            }
        }

        // Compatibilité avec les anciennes bases : un quiz directement rattaché à la
        // formation reste visible lorsqu'aucun quiz de page persistant n'a été importé.
        if ($requiredPageQuizzes === [] && $bonusQuizzes === []) {
            $requiredPageQuizzes = $quizzes->findBy(['formation' => $formation], ['id' => 'ASC']);
        }

        $allPageQuizzes = [...$requiredPageQuizzes, ...$bonusQuizzes];
        $quizQuestions = [];
        $quizResults = [];
        $quizMeta = [];
        $user = $this->getUser();

        if ($user instanceof Utilisateur) {
            $guidedTraining->synchronizeParentProgress($formation, $user, false);
        }
        $sectionJourney = $guidedTraining->buildJourney(
            $formation,
            $user instanceof Utilisateur ? $user : null,
        );

        foreach ($allPageQuizzes as $quiz) {
            $quizQuestions[$quiz->getId()] = $questions->findBy(['quiz' => $quiz], ['ordre' => 'ASC']);
            $quizFormation = $quiz->getFormation();
            $machineId = $quizCatalog->getMachineId($quizFormation);
            $machine = $machineId !== null ? $machines->find($machineId) : null;
            $isBonus = $quizCatalog->isBonusQuizFormation($quizFormation);

            $quizMeta[$quiz->getId()] = [
                'machine' => $machine,
                'key' => $quizCatalog->getQuizKey($quizFormation),
                'bonus' => $isBonus,
                'title' => $quiz->getSection()?->getTitre() ?: ($quizFormation?->getTitre() ?: 'Quiz – ' . ($machine?->getNom() ?? $formation->getTitre())),
            ];

            if ($user instanceof Utilisateur && $quizFormation !== null) {
                $quizResults[$quiz->getId()] = $progressions->findOneBy([
                    'utilisateur' => $user,
                    'formation' => $quizFormation,
                ]);
            }
        }

        $trainingProgression = $user instanceof Utilisateur
            ? $progressions->findOneBy(['utilisateur' => $user, 'formation' => $formation])
            : null;

        if ($sectionJourney['configured']) {
            $trainingScore = $sectionJourney['percent'];
            $trainingStepTotal = $sectionJourney['total'];
            $trainingValidated = $sectionJourney['completed'];
        } else {
            $trainingScore = $trainingProgression?->isCompleted()
                ? 100
                : max(0, min(100, $trainingProgression?->getScore() ?? 0));
            $trainingStepTotal = max(1, count($formationSections));
            $trainingValidated = min($trainingStepTotal, (int) round(($trainingScore / 100) * $trainingStepTotal));
        }

        $quizTotal = count($requiredPageQuizzes);
        $quizValidated = 0;
        foreach ($requiredPageQuizzes as $quiz) {
            $result = $quizResults[$quiz->getId()] ?? null;
            if ($result !== null && $result->isCompleted()) {
                ++$quizValidated;
            }
        }

        $trainingWeight = $quizTotal > 0 ? 60 : 100;
        $quizWeight = $quizTotal > 0 ? 40 : 0;
        $quizAverage = $quizTotal > 0 ? ($quizValidated / $quizTotal) * 100 : 100;
        $overallPercent = (int) round(($trainingScore / 100) * $trainingWeight + ($quizAverage / 100) * $quizWeight);
        $totalSteps = $trainingStepTotal + $quizTotal;
        $completedSteps = $trainingValidated + $quizValidated;

        $qualificationStatus = $user instanceof Utilisateur
            ? $qualification->getStatus($formation, $user)
            : [
                'badge' => $formation->getBadge(),
                'badgeOwned' => false,
                'badgeUnlocked' => false,
                'badgeObtainedAt' => null,
                'overallPercent' => $overallPercent,
                'theoryReady' => false,
                'pathCompleted' => false,
                'physicalRequired' => $trainingPolicy->formationRequiresPhysicalTraining($formation),
                'physicalFormation' => $qualification->getPhysicalFormation($formation),
                'physicalProgression' => null,
                'physicalCompleted' => false,
                'eligible' => false,
            ];
        $overallPercent = (int) $qualificationStatus['overallPercent'];

        return $this->render('site/formation-suivi.html.twig', [
            'formation' => $formation,
            'formationVisual' => $this->buildFormationVisual($formation),
            'progressions' => $trainingProgression !== null ? [$trainingProgression] : [],
            'trainingProgression' => $trainingProgression,
            'sections' => $formationSections,
            'quizzes' => $requiredPageQuizzes,
            'bonusQuizzes' => $bonusQuizzes,
            'sectionJourney' => $sectionJourney,
            'quizQuestions' => $quizQuestions,
            'quizResults' => $quizResults,
            'quizMeta' => $quizMeta,
            'qualificationStatus' => $qualificationStatus,
            'formationPolicy' => $trainingPolicy->getFormationPolicy($formation),
            'progressSummary' => [
                'overallPercent' => $overallPercent,
                'completed' => (bool) $qualificationStatus['pathCompleted'],
                'trainingWeight' => $trainingWeight,
                'quizWeight' => $quizWeight,
                'trainingScore' => $trainingScore,
                'trainingStepTotal' => $trainingStepTotal,
                'trainingValidated' => $trainingValidated,
                'quizTotal' => $quizTotal,
                'quizValidated' => $quizValidated,
                'quizAverage' => (int) round($quizAverage),
                'totalSteps' => $totalSteps,
                'completedSteps' => $completedSteps,
            ],
        ]);
    }

    #[Route('/leaderboard', name: 'app_leaderboard', methods: ['GET'])]
    #[Route('/leaderboard.html', name: 'app_leaderboard_html', methods: ['GET'])]
    public function leaderboard(
        Request $request,
        UtilisateurRepository $users,
        UtilisateurBadgeRepository $userBadges,
        ProgressionRepository $progressions,
        LogUtilisationRepository $usageLogs,
        MachineRepository $machines,
        CreationRepository $creations,
    ): Response
    {
        $activeTab = in_array($request->query->get('tab'), ['presence', 'prints'], true) ? (string) $request->query->get('tab') : 'presence';
        $activePeriod = in_array($request->query->get('period'), ['week', 'month', 'all'], true) ? (string) $request->query->get('period') : 'week';
        [$periodStart, $periodEnd, $periodLabel] = $this->resolveLeaderboardPeriod($activePeriod);

        $allUsers = $users->findAll();
        $presenceByUser = $usageLogs->computePresenceMinutesByUser($allUsers, $periodStart, $periodEnd);
        $printsByUser = $usageLogs->count3dPrintsByUser($allUsers, $periodStart, $periodEnd);
        $machinesByUser = $usageLogs->countMachinesUsedByUser($allUsers, $periodStart, $periodEnd);

        $rows = [];
        foreach ($allUsers as $user) {
            $userId = $user->getId();
            if ($userId === null) {
                continue;
            }

            $presenceMinutes = $presenceByUser[$userId] ?? 0;
            $printCount = $printsByUser[$userId] ?? 0;
            $rows[] = [
                'rank' => 0,
                'user' => $user,
                'presenceMinutes' => $presenceMinutes,
                'printCount' => $printCount,
                'score' => $activeTab === 'prints' ? $printCount : $presenceMinutes,
                'badges' => 0,
                'completedFormations' => 0,
                'machinesUsed' => $machinesByUser[$userId] ?? 0,
            ];
        }

        usort($rows, static function (array $a, array $b) use ($activeTab): int {
            $primary = $activeTab === 'prints'
                ? ($b['printCount'] <=> $a['printCount'])
                : ($b['presenceMinutes'] <=> $a['presenceMinutes']);

            if ($primary !== 0) {
                return $primary;
            }

            return strcasecmp($a['user']->getDisplayName(), $b['user']->getDisplayName());
        });

        $currentUserRank = null;
        $currentUser = $this->getUser();
        foreach ($rows as $index => &$row) {
            $row['rank'] = $index + 1;
            if ($currentUser instanceof Utilisateur && $row['user']->getId() === $currentUser->getId()) {
                $currentUserRank = $row['rank'];
            }
        }
        unset($row);

        $leaderboard = array_slice($rows, 0, 20);
        foreach ($leaderboard as &$entry) {
            $entry['badges'] = $userBadges->count(['utilisateur' => $entry['user']]);
            $entry['completedFormations'] = $progressions->count(['utilisateur' => $entry['user'], 'completed' => true]);
        }
        unset($entry);

        return $this->render('site/leaderboard.html.twig', [
            'leaderboard' => $leaderboard,
            'activeTab' => $activeTab,
            'activePeriod' => $activePeriod,
            'periodLabel' => $periodLabel,
            'currentUserRank' => $currentUserRank,
            'stats' => [
                'users' => $users->count([]),
                'machines' => $machines->count([]),
                'prints' => $usageLogs->count3dPrints($periodStart, $periodEnd),
                'creations' => $creations->count(['isPublished' => true]),
            ],
        ]);
    }

    #[Route('/leaderboard/creations', name: 'app_leaderboard_creations', methods: ['GET'])]
    public function leaderboardCreations(Request $request, CreationRepository $creations, CreationVoteRepository $votes): Response
    {
        $sort = (string) $request->query->get('sort', 'recent');
        if (!in_array($sort, ['recent', 'rating'], true)) {
            $sort = 'recent';
        }

        $creationItems = $creations->findPublishedForGallery($sort);
        $ratingStats = $votes->getStatsByCreation($creationItems);
        $votersByCreation = $votes->getVotersByCreation($creationItems);
        $currentUser = $this->getUser();
        $userRatings = $currentUser instanceof Utilisateur ? $votes->getUserRatingsByCreation($creationItems, $currentUser) : [];
        $creationRows = [];

        foreach ($creationItems as $creation) {
            $creationId = $creation->getId();
            $stats = $ratingStats[$creationId] ?? ['average' => null, 'count' => 0];
            $userRating = $creationId !== null ? ($userRatings[$creationId] ?? null) : null;

            $creationRows[] = [
                'creation' => $creation,
                'averageRating' => $stats['average'],
                'ratingCount' => $stats['count'],
                'voteCount' => $stats['count'],
                'userRating' => $userRating,
                'userHasVoted' => $userRating !== null,
                'voters' => $creationId !== null ? ($votersByCreation[$creationId] ?? []) : [],
            ];
        }

        $topCreations = $creations->findTopRatedPublished(3);
        $topStats = $votes->getStatsByCreation($topCreations);
        $topRows = [];

        foreach ($topCreations as $creation) {
            $creationId = $creation->getId();
            $stats = $topStats[$creationId] ?? ['average' => null, 'count' => 0];
            $topRows[] = [
                'creation' => $creation,
                'averageRating' => $stats['average'],
                'ratingCount' => $stats['count'],
            ];
        }

        return $this->render('site/leaderboard-creations.html.twig', [
            'creationRows' => $creationRows,
            'topRows' => $topRows,
            'activeSort' => $sort,
        ]);
    }

    #[Route('/leaderboard/creations/ranking', name: 'app_leaderboard_creations_ranking', methods: ['GET'])]
    public function leaderboardCreationsRanking(CreationRepository $creations, CreationVoteRepository $votes): Response
    {
        $creationItems = $creations->findPublishedRanking();
        $ratingStats = $votes->getStatsByCreation($creationItems);
        $rankingRows = [];

        foreach ($creationItems as $index => $creation) {
            $creationId = $creation->getId();
            $stats = $ratingStats[$creationId] ?? ['average' => null, 'count' => 0];

            $rankingRows[] = [
                'rank' => $index + 1,
                'creation' => $creation,
                'averageRating' => $stats['average'],
                'ratingCount' => $stats['count'],
            ];
        }

        return $this->render('site/leaderboard-creations-ranking.html.twig', [
            'rankingRows' => $rankingRows,
        ]);
    }

    #[Route('/leaderboard/creations/new', name: 'app_leaderboard_creation_new', methods: ['GET', 'POST'])]
    #[IsGranted('ROLE_USER')]
    public function newLeaderboardCreation(Request $request, EntityManagerInterface $entityManager, SluggerInterface $slugger): Response
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException('Authentification requise');
        }

        $this->compressOversizedCreationImage($request, 'creation_user');

        $creation = (new Creation())
            ->setAuthor($user)
            ->setIsPublished(true);
        $form = $this->createForm(CreationUserType::class, $creation);
        $form->handleRequest($request);

        if ($form->isSubmitted()) {
            $this->applyPublicCreationDuration($creation, $form);
            $this->applyPublicCreationTags($creation, $form);
        }

        if ($form->isSubmitted() && $form->isValid()) {
            $this->normalizePublicCreationData($creation);

            if (!$this->handlePublicCreationUploads($creation, $form, $slugger, true)) {
                $this->addFlash('error', 'La création n’a pas été publiée. Vérifie les erreurs du formulaire.');

                return $this->render('site/leaderboard-creation-new.html.twig', [
                    'creation' => $creation,
                    'form' => $form,
                ], new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY));
            }

            $entityManager->persist($creation);
            $entityManager->flush();
            $this->addFlash('success', 'Création publiée avec succès !');

            return $this->redirectToRoute('app_leaderboard_creations');
        }

        if ($form->isSubmitted()) {
            $this->addFlash('error', 'La création n’a pas été publiée. Vérifie les erreurs du formulaire.');
        }

        return $this->render('site/leaderboard-creation-new.html.twig', [
            'creation' => $creation,
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/leaderboard/creations/{id}/vote', name: 'app_leaderboard_creation_vote', requirements: ['id' => '\\d+'], methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function voteLeaderboardCreation(Creation $creation, Request $request, EntityManagerInterface $entityManager, CreationVoteRepository $votes): Response
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException('Authentification requise');
        }

        if (!$creation->isPublished()) {
            throw $this->createNotFoundException('Création introuvable');
        }

        if (!$this->isCsrfTokenValid('vote_creation_' . $creation->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Notation refusée : token CSRF invalide.');
            return $this->redirectToRoute('app_leaderboard_creations');
        }

        $rawRating = $request->request->get('rating');
        if (!is_numeric($rawRating)) {
            $this->addFlash('error', 'Choisis une note avant de confirmer.');
            return $this->redirectToRoute('app_leaderboard_creations');
        }

        $rating = (float) $rawRating;
        if ($rating < 0.5 || $rating > 5.0 || abs(($rating * 2) - round($rating * 2)) > 0.0001) {
            $this->addFlash('error', 'La note doit être comprise entre 0.5 et 5, par pas de 0.5.');
            return $this->redirectToRoute('app_leaderboard_creations');
        }

        $vote = $votes->findUserRating($creation, $user);
        if ($vote === null) {
            $vote = (new CreationVote())
                ->setCreation($creation)
                ->setUser($user);
            $entityManager->persist($vote);
        } else {
            $vote->setUpdatedAt(new \DateTime());
        }

        $vote->setRating($rating);
        $entityManager->flush();
        $this->addFlash('success', sprintf('Note enregistrée : %.1f/5.', $rating));

        return $this->redirectToRoute('app_leaderboard_creations');
    }

    #[Route('/leaderboard/creations/{id}/delete', name: 'app_leaderboard_creation_delete', requirements: ['id' => '\\d+'], methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function deleteLeaderboardCreation(Creation $creation, Request $request, EntityManagerInterface $entityManager): Response
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException('Authentification requise');
        }

        // Only the creation's own author may delete it here (admins have their own tool).
        $author = $creation->getAuthor();
        if ($author === null || $author->getId() !== $user->getId()) {
            throw $this->createAccessDeniedException('Tu ne peux supprimer que tes propres créations.');
        }

        if (!$this->isCsrfTokenValid('delete_creation_' . $creation->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Suppression refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_leaderboard_creations');
        }

        $title = $creation->getTitle();
        $imageFilename = $creation->getImageFilename();
        $fileFilename = $creation->getFileFilename();

        $entityManager->remove($creation);
        $entityManager->flush();

        $this->deleteCreationUploadIfSafe('public/uploads/creations/images', $imageFilename);
        $this->deleteCreationUploadIfSafe('public/uploads/creations/files', $fileFilename);
        $this->addFlash('success', sprintf('Création "%s" supprimée.', $title));

        return $this->redirectToRoute('app_leaderboard_creations');
    }

    /**
     * Deletes an uploaded creation asset only when the filename is a safe basename that
     * resolves inside the expected directory — mirrors the admin-side cleanup helper.
     */
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

    #[Route('/badges', name: 'app_badges', methods: ['GET'])]
    public function badges(BadgeRepository $badges): Response
    {
        return $this->render('site/badges.html.twig', [
            'badges' => $badges->findBy([], ['nom' => 'ASC']),
        ]);
    }

    #[Route('/badges/{id}', name: 'app_badge_detail', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function badgeDetail(Badge $badge, FormationRepository $formations): Response
    {
        $machineAccess = [];
        foreach ($badge->getMachineBadges() as $machineBadge) {
            $machine = $machineBadge->getMachine();
            if ($machine === null) {
                continue;
            }

            $machineAccess[] = [
                'machine' => $machine,
                'requiredForAccess' => $machineBadge->isRequiredForAccess(),
            ];
        }

        return $this->render('site/badge-detail.html.twig', [
            'badge' => $badge,
            'machineAccess' => $machineAccess,
            'earnedVia' => $formations->findBy(['badge' => $badge], ['titre' => 'ASC']),
            'unlockedBadges' => $badge->getUnlockedBadges(),
        ]);
    }

    #[Route('/lab', name: 'app_lab_pages', methods: ['GET'])]
    public function labPages(LabPageRepository $labPages): Response
    {
        return $this->render('site/lab-pages.html.twig', [
            'topLevelPages' => $labPages->findTopLevel(),
        ]);
    }

    #[Route('/lab/{id}', name: 'app_lab_page', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function labPage(LabPage $page): Response
    {
        return $this->render('site/lab-page-detail.html.twig', [
            'page' => $page,
        ]);
    }

    #[Route('/places', name: 'app_places', methods: ['GET'])]
    public function places(PlaceRepository $places): Response
    {
        return $this->render('site/places.html.twig', [
            'places' => $places->findBy([], ['nom' => 'ASC']),
        ]);
    }

    #[Route('/places/{id}', name: 'app_place_detail', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function placeDetail(Place $place, ReservationRepository $reservations): Response
    {
        return $this->render('site/place-detail.html.twig', [
            'place' => $place,
            'reservations' => $reservations->findActiveByPlace($place, ['dateDebut' => 'ASC']),
        ]);
    }

    /**
     * Re-renders the place booking form after a validation error, keeping the
     * user's submitted values so they don't have to retype everything.
     */
    private function renderPlaceBookingError(Place $place, ReservationRepository $reservations, Request $request, string $error): Response
    {
        $response = $this->render('site/place-detail.html.twig', [
            'place' => $place,
            'reservations' => $reservations->findActiveByPlace($place, ['dateDebut' => 'ASC']),
            'bookingError' => $error,
            'submitted' => [
                'date' => (string) $request->request->get('date'),
                'startTime' => (string) $request->request->get('startTime'),
                'endTime' => (string) $request->request->get('endTime'),
                'motif' => (string) $request->request->get('motif'),
            ],
        ]);
        $response->setStatusCode(Response::HTTP_UNPROCESSABLE_ENTITY);

        return $response;
    }

    #[Route('/places/{id}/reserve', name: 'app_place_reserve', requirements: ['id' => '\d+'], methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function reservePlace(Place $place, Request $request, EntityManagerInterface $entityManager, ReservationRepository $reservations, OpeningHoursProvider $openingHours): Response
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException('Authentification requise');
        }

        if (!$this->isCsrfTokenValid('place_reserve_' . $place->getId(), (string) $request->request->get('_token'))) {
            return $this->renderPlaceBookingError($place, $reservations, $request, 'Réservation refusée : token CSRF invalide.');
        }

        $dateInput = (string) $request->request->get('date');
        $startInput = (string) $request->request->get('startTime');
        $endInput = (string) $request->request->get('endTime');
        $motif = trim((string) $request->request->get('motif'));

        try {
            $dateDebut = new \DateTimeImmutable($dateInput . ' ' . $startInput, new \DateTimeZone('Europe/Paris'));
            $dateFin = new \DateTimeImmutable($dateInput . ' ' . $endInput, new \DateTimeZone('Europe/Paris'));
        } catch (\Throwable) {
            return $this->renderPlaceBookingError($place, $reservations, $request, 'Date ou horaire invalide.');
        }

        $now = new \DateTimeImmutable('now', new \DateTimeZone('Europe/Paris'));
        if ($dateDebut <= $now) {
            return $this->renderPlaceBookingError($place, $reservations, $request, 'La date de début doit être dans le futur.');
        }

        if ($dateFin <= $dateDebut) {
            return $this->renderPlaceBookingError($place, $reservations, $request, 'L’heure de fin doit être après l’heure de début.');
        }

        $openingHoursError = $openingHours->validateReservationPeriod($dateDebut, $dateFin);
        if ($openingHoursError !== null) {
            return $this->renderPlaceBookingError($place, $reservations, $request, $openingHoursError);
        }

        if ($motif !== '' && mb_strlen($motif) > 500) {
            return $this->renderPlaceBookingError($place, $reservations, $request, 'Le motif ne doit pas dépasser 500 caractères.');
        }

        if ($reservations->hasOverlapForPlace($place, $dateDebut, $dateFin)) {
            return $this->renderPlaceBookingError($place, $reservations, $request, 'Ce créneau est déjà réservé pour cet espace.');
        }

        $reservation = (new Reservation())
            ->setPlace($place)
            ->setUtilisateur($user)
            ->setDateDebut($dateDebut)
            ->setDateFin($dateFin)
            ->setMotif($motif !== '' ? $motif : null)
            ->setStatut('confirmed');

        $entityManager->persist($reservation);
        $entityManager->flush();
        $this->addFlash('success', 'Réservation confirmée.');

        return $this->redirectToRoute('app_place_detail', ['id' => $place->getId()]);
    }

    #[Route('/events', name: 'app_events', methods: ['GET'])]
    public function events(EventRepository $events): Response
    {
        return $this->render('site/events.html.twig', [
            'events' => $events->findUpcoming(),
        ]);
    }

    #[Route('/events/{id}', name: 'app_event_detail', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function eventDetail(Event $event): Response
    {
        return $this->render('site/event-detail.html.twig', [
            'event' => $event,
        ]);
    }

    #[Route('/locale/{locale}', name: 'app_switch_locale', requirements: ['locale' => 'fr|en|es|de|it'], methods: ['GET'])]
    public function switchLocale(string $locale, Request $request, EntityManagerInterface $entityManager): Response
    {
        $request->getSession()->set('_locale', $locale);

        // Persist the choice on the profile too, so it follows a logged-in user everywhere.
        $user = $this->getUser();
        if ($user instanceof Utilisateur) {
            $user->setLangue($locale);
            $entityManager->flush();
        }

        // Redirect back where the user came from, but only within this site (no open redirect).
        $referer = (string) $request->headers->get('referer');
        $isSameHost = $referer !== '' && str_starts_with($referer, $request->getSchemeAndHttpHost() . '/');

        return $this->redirect($isSameHost ? $referer : $this->generateUrl('app_home'));
    }

    #[Route('/register', name: 'app_register', methods: ['GET', 'POST'])]
    #[Route('/register.html', name: 'app_register_html', methods: ['GET'])]
    public function register(
        Request $request,
        EntityManagerInterface $entityManager,
        UserPasswordHasherInterface $passwordHasher,
        UtilisateurRepository $users,
        RoleRepository $roles,
    ): Response
    {
        if ($this->getUser() instanceof Utilisateur) {
            return $this->redirectToRoute('app_profile');
        }

        $errors = [];
        $formData = [
            'firstName' => '',
            'lastName' => '',
            'email' => '',
            'terms' => false,
        ];

        if ($request->isMethod('POST')) {
            $formData = [
                'firstName' => trim((string) $request->request->get('firstName', '')),
                'lastName' => trim((string) $request->request->get('lastName', '')),
                'email' => mb_strtolower(trim((string) $request->request->get('email', ''))),
                'terms' => $request->request->has('terms'),
            ];
            $password = (string) $request->request->get('password', '');
            $confirmPassword = (string) $request->request->get('confirmPassword', '');

            if (!$this->isCsrfTokenValid('register', (string) $request->request->get('_token'))) {
                $errors[] = 'La demande d’inscription a expiré. Rechargez la page puis réessayez.';
            }

            if ($formData['firstName'] === '') {
                $errors[] = 'Le prénom est obligatoire.';
            }
            if ($formData['lastName'] === '') {
                $errors[] = 'Le nom est obligatoire.';
            }
            if ($formData['email'] === '' || filter_var($formData['email'], FILTER_VALIDATE_EMAIL) === false) {
                $errors[] = 'L’adresse email est invalide.';
            } elseif ($users->findOneBy(['email' => $formData['email']]) instanceof Utilisateur) {
                $errors[] = 'Un compte existe déjà avec cette adresse email.';
            }
            if (mb_strlen($password) < 8) {
                $errors[] = 'Le mot de passe doit contenir au moins 8 caractères.';
            }
            if ($password !== $confirmPassword) {
                $errors[] = 'Les deux mots de passe ne correspondent pas.';
            }
            if (!$formData['terms']) {
                $errors[] = 'Vous devez accepter les conditions d’utilisation.';
            }

            if ($errors === []) {
                $user = (new Utilisateur())
                    ->setFirstName($formData['firstName'])
                    ->setLastName($formData['lastName'])
                    ->setEmail($formData['email'])
                    ->setUsername($this->generateUniqueUsername($formData['email'], $users))
                    ->setPassword('')
                    ->setStatut('actif')
                    ->setIsVerified(true);

                $user->setPassword($passwordHasher->hashPassword($user, $password));

                $role = $roles->findOneBySecurityRole('ROLE_USER');
                if (!$role instanceof Role) {
                    $role = (new Role())->setNom('ROLE_USER');
                    $entityManager->persist($role);
                }

                $userRole = (new UtilisateurRole())
                    ->setUtilisateur($user)
                    ->setRole($role);

                $entityManager->persist($user);
                $entityManager->persist($userRole);
                $entityManager->flush();

                $this->addFlash('success', 'Votre compte a été créé. Vous pouvez maintenant vous connecter.');

                return $this->redirectToRoute('app_login');
            }
        }

        return $this->render('site/register.html.twig', [
            'errors' => $errors,
            'formData' => $formData,
        ]);
    }

    #[Route('/forgot-password', name: 'app_forgot_password', methods: ['GET'])]
    #[Route('/forgot-password.html', name: 'app_forgot_password_html', methods: ['GET'])]
    public function forgotPassword(): Response
    {
        return $this->render('site/forgot-password.html.twig');
    }

    #[Route('/profil', name: 'app_profile', methods: ['GET', 'POST'])]
    #[Route('/profil.html', name: 'app_profile_html', methods: ['GET'])]
    public function profile(
        Request $request,
        EntityManagerInterface $entityManager,
        ProgressionRepository $progressions,
        UtilisateurBadgeRepository $userBadges,
        FormationRepository $formations,
        TrainingQualificationService $qualification,
        ReservationRepository $reservations,
        AccessRfidLogRepository $rfidLogs,
        LogUtilisationRepository $usageLogs,
        SluggerInterface $slugger,
        UtilisateurRepository $users,
        LoanRepository $loans,
        ModuleService $modules,
    ): Response
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException('Utilisateur connecte requis');
        }

        if ($request->isMethod('POST') && $request->request->get('_profile_form') === 'avatar') {
            if (!$this->isCsrfTokenValid('profile_avatar', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'La mise a jour de l avatar a ete refusee. Rechargez la page puis reessayez.');

                return $this->redirectToRoute('app_profile');
            }

            $avatarFile = $request->files->get('avatar');
            if (!$avatarFile instanceof UploadedFile || $avatarFile->getError() === UPLOAD_ERR_NO_FILE) {
                $this->addFlash('error', 'Choisissez une image PNG, JPG, JPEG ou WEBP.');

                return $this->redirectToRoute('app_profile');
            }

            if (!$avatarFile->isValid()) {
                $this->addFlash('error', 'L upload de l image a echoue.');

                return $this->redirectToRoute('app_profile');
            }

            if ($avatarFile->getSize() !== null && $avatarFile->getSize() > 2 * 1024 * 1024) {
                $this->addFlash('error', 'L image ne doit pas depasser 2 Mo.');

                return $this->redirectToRoute('app_profile');
            }

            $mimeType = $avatarFile->getMimeType();
            if (!in_array($mimeType, ['image/png', 'image/jpeg', 'image/webp'], true)) {
                $this->addFlash('error', 'Choisissez une image PNG, JPG, JPEG ou WEBP.');

                return $this->redirectToRoute('app_profile');
            }

            $uploadDir = $this->getParameter('kernel.project_dir') . '/public/uploads/avatars';
            if (!is_dir($uploadDir) && !mkdir($uploadDir, 0775, true) && !is_dir($uploadDir)) {
                $this->addFlash('error', 'Impossible de creer le dossier de destination des avatars.');

                return $this->redirectToRoute('app_profile');
            }

            $baseName = strtolower($slugger->slug($user->getDisplayName() ?: $user->getUsername())->toString());
            if ($baseName === '') {
                $baseName = 'utilisateur';
            }

            $extension = $avatarFile->guessExtension() ?: $avatarFile->getClientOriginalExtension();
            $extension = strtolower($extension ?: 'bin');
            if ($extension === 'jpeg') {
                $extension = 'jpg';
            }

            $fileName = sprintf('%s-%s.%s', $baseName, bin2hex(random_bytes(3)), $extension);
            $previousAvatarFilename = $user->getAvatarFilename();

            try {
                $avatarFile->move($uploadDir, $fileName);
            } catch (FileException) {
                $this->addFlash('error', 'Impossible de copier l image de profil.');

                return $this->redirectToRoute('app_profile');
            }

            $user->setAvatarFilename($fileName);
            $entityManager->flush();
            $this->deleteUnusedPreviousAvatar($previousAvatarFilename, $uploadDir, $users);
            $this->addFlash('success', 'Photo de profil mise a jour.');

            return $this->redirectToRoute('app_profile');
        }

        if ($request->isMethod('POST') && $request->request->get('_profile_form') === 'banner') {
            if (!$this->isCsrfTokenValid('profile_banner', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'La mise a jour de la banniere a ete refusee. Rechargez la page puis reessayez.');

                return $this->redirectToRoute('app_profile');
            }

            $bannerFile = $request->files->get('banner');
            if (!$bannerFile instanceof UploadedFile || $bannerFile->getError() === UPLOAD_ERR_NO_FILE) {
                $this->addFlash('error', 'Choisissez une image PNG, JPG, JPEG, WEBP ou GIF.');

                return $this->redirectToRoute('app_profile');
            }

            if (!$bannerFile->isValid()) {
                $this->addFlash('error', 'L upload de la banniere a echoue.');

                return $this->redirectToRoute('app_profile');
            }

            if ($bannerFile->getSize() !== null && $bannerFile->getSize() > 5 * 1024 * 1024) {
                $this->addFlash('error', 'La banniere ne doit pas depasser 5 Mo.');

                return $this->redirectToRoute('app_profile');
            }

            $mimeType = $bannerFile->getMimeType();
            if (!in_array($mimeType, ['image/png', 'image/jpeg', 'image/webp', 'image/gif'], true)) {
                $this->addFlash('error', 'Choisissez une image PNG, JPG, JPEG, WEBP ou GIF.');

                return $this->redirectToRoute('app_profile');
            }

            $uploadDir = $this->getParameter('kernel.project_dir') . '/public/uploads/profile-banners';
            if (!is_dir($uploadDir) && !mkdir($uploadDir, 0775, true) && !is_dir($uploadDir)) {
                $this->addFlash('error', 'Impossible de creer le dossier de destination des bannieres.');

                return $this->redirectToRoute('app_profile');
            }

            $baseName = strtolower($slugger->slug($user->getDisplayName() ?: $user->getUsername())->toString());
            if ($baseName === '') {
                $baseName = 'utilisateur';
            }

            $extension = match ($mimeType) {
                'image/png' => 'png',
                'image/jpeg' => 'jpg',
                'image/webp' => 'webp',
                'image/gif' => 'gif',
            };

            $fileName = sprintf('%s-banner-%s.%s', $baseName, bin2hex(random_bytes(3)), $extension);
            $previousBannerFilename = $user->getBannerFilename();

            try {
                $bannerFile->move($uploadDir, $fileName);
            } catch (FileException) {
                $this->addFlash('error', 'Impossible de copier la banniere de profil.');

                return $this->redirectToRoute('app_profile');
            }

            $user->setBannerFilename($fileName);
            $entityManager->flush();
            $this->deleteUnusedPreviousBanner($previousBannerFilename, $uploadDir, $users);
            $this->addFlash('success', 'Banniere de profil mise a jour.');

            return $this->redirectToRoute('app_profile');
        }

        if ($request->isMethod('POST') && $request->request->get('_profile_form') === 'delete_banner') {
            if (!$this->isCsrfTokenValid('profile_banner_delete', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'La suppression de la banniere a ete refusee. Rechargez la page puis reessayez.');

                return $this->redirectToRoute('app_profile');
            }

            $previousBannerFilename = $user->getBannerFilename();
            $user->setBannerFilename(null);
            $entityManager->flush();
            $this->deleteUnusedPreviousBanner($previousBannerFilename, $this->getParameter('kernel.project_dir') . '/public/uploads/profile-banners', $users);
            $this->addFlash('success', 'Banniere de profil supprimee. Le fond par defaut est de nouveau utilise.');

            return $this->redirectToRoute('app_profile');
        }

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('profile_preferences', (string) $request->request->get('_token'))) {
                if ($request->isXmlHttpRequest()) {
                    return $this->json([
                        'ok' => false,
                        'message' => 'La sauvegarde des préférences a été refusée. Rechargez la page puis réessayez.',
                    ], Response::HTTP_UNPROCESSABLE_ENTITY);
                }

                $this->addFlash('error', 'La sauvegarde des preferences a ete refusee. Rechargez la page puis reessayez.');

                return $this->redirectToRoute('app_profile');
            }

            $theme = (string) $request->request->get('theme', '');

            if (!in_array($theme, ['light', 'dark', 'system'], true)) {
                if ($request->isXmlHttpRequest()) {
                    return $this->json([
                        'ok' => false,
                        'message' => 'Thème invalide.',
                    ], Response::HTTP_UNPROCESSABLE_ENTITY);
                }

                $this->addFlash('error', 'Theme invalide.');

                return $this->redirectToRoute('app_profile');
            }

            // Le bouton animé du profil enregistre uniquement le thème, sans
            // écraser les autres préférences éventuellement en cours d'édition.
            if ($request->request->getBoolean('_theme_only')) {
                $user->setTheme($theme);
                $entityManager->flush();

                if ($request->isXmlHttpRequest()) {
                    return $this->json([
                        'ok' => true,
                        'theme' => $theme,
                    ]);
                }

                return $this->redirectToRoute('app_profile');
            }

            $langue = (string) $request->request->get('langue', '');

            if (!in_array($langue, ['fr', 'en'], true)) {
                $this->addFlash('error', 'Langue invalide.');

                return $this->redirectToRoute('app_profile');
            }

            $user
                ->setNotificationEmail($request->request->has('notificationEmail'))
                ->setNotificationPush($request->request->has('notificationPush'))
                ->setRappelReservation($request->request->has('rappelReservation'))
                ->setTheme($theme)
                ->setLangue($langue);

            $entityManager->flush();
            $this->addFlash('success', 'Preferences mises a jour.');

            return $this->redirectToRoute('app_profile');
        }

        $userProgressions = $progressions->findVisibleByUser($user);
        $completedProgressions = array_values(array_filter($userProgressions, static fn ($progression): bool => $progression->isCompleted()));

        $qualifiedUserBadges = [];
        foreach ($userBadges->findBy(['utilisateur' => $user], ['dateObtention' => 'DESC']) as $userBadge) {
            $badge = $userBadge->getBadge();
            if ($badge === null) {
                continue;
            }

            $badgeFormation = $formations->findVisibleByBadge($badge);
            if ($badgeFormation === null || $qualification->getStatus($badgeFormation, $user)['badgeUnlocked']) {
                $qualifiedUserBadges[] = $userBadge;
            }
        }

        $userUsageLogs = $usageLogs->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']);
        $userRfidLogs = $rfidLogs->findBy(['utilisateur' => $user], ['createdAt' => 'DESC']);

        $machineIds = [];
        foreach ($userUsageLogs as $usageLog) {
            $machine = $usageLog->getMachine();
            if ($machine?->getId() !== null) {
                $machineIds[$machine->getId()] = true;
            }
        }
        foreach ($userRfidLogs as $rfidLog) {
            $machine = $rfidLog->getMachine();
            if ($machine?->getId() !== null) {
                $machineIds[$machine->getId()] = true;
            }
        }

        return $this->render('site/profil.html.twig', [
            'user' => $user,
            'progressions' => $userProgressions,
            'completedProgressions' => $completedProgressions,
            'userBadges' => $qualifiedUserBadges,
            'reservations' => $reservations->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']),
            'rfidLogs' => $userRfidLogs,
            'usageLogs' => $userUsageLogs,
            'loansEnabled' => $modules->isEnabled('loans'),
            'myLoans' => $loans->findForBorrower($user),
            'profileStats' => [
                'completedFormations' => count($completedProgressions),
                'badges' => count($qualifiedUserBadges),
                'reservations' => $reservations->count(['utilisateur' => $user]),
                'rfidLogs' => count($userRfidLogs),
                'usageLogs' => count($userUsageLogs),
                'machinesUsed' => count($machineIds),
            ],
        ]);
    }


    private function deleteUnusedPreviousAvatar(?string $previousAvatarFilename, string $uploadDir, UtilisateurRepository $users): void
    {
        if ($previousAvatarFilename === null || trim($previousAvatarFilename) === '' || $previousAvatarFilename === 'default-avatar.svg') {
            return;
        }

        $safePreviousAvatarFilename = basename($previousAvatarFilename);
        if ($safePreviousAvatarFilename !== $previousAvatarFilename || $safePreviousAvatarFilename === '' || $safePreviousAvatarFilename === 'default-avatar.svg') {
            return;
        }

        if ($users->count(['avatarFilename' => $safePreviousAvatarFilename]) > 0) {
            return;
        }

        $uploadDirRealPath = realpath($uploadDir);
        if ($uploadDirRealPath === false) {
            return;
        }

        $previousAvatarPath = $uploadDirRealPath . DIRECTORY_SEPARATOR . $safePreviousAvatarFilename;
        if (!is_file($previousAvatarPath)) {
            return;
        }

        $previousAvatarRealPath = realpath($previousAvatarPath);
        if ($previousAvatarRealPath === false) {
            return;
        }

        $uploadDirPrefix = rtrim($uploadDirRealPath, DIRECTORY_SEPARATOR) . DIRECTORY_SEPARATOR;
        if (!str_starts_with($previousAvatarRealPath, $uploadDirPrefix)) {
            return;
        }

        @unlink($previousAvatarRealPath);
    }

    #[Route('/profil/password', name: 'app_profile_password', methods: ['GET', 'POST'])]
    public function profilePassword(
        Request $request,
        EntityManagerInterface $entityManager,
        UserPasswordHasherInterface $passwordHasher,
    ): Response
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur || !$user instanceof PasswordAuthenticatedUserInterface) {
            throw $this->createAccessDeniedException('Utilisateur connecte requis');
        }

        $errors = [];

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('profile_password', (string) $request->request->get('_token'))) {
                $errors[] = 'La demande de changement de mot de passe a ete refusee. Rechargez la page puis reessayez.';
            }

            $currentPassword = (string) $request->request->get('currentPassword', '');
            $newPassword = (string) $request->request->get('newPassword', '');
            $confirmPassword = (string) $request->request->get('confirmPassword', '');

            if ($currentPassword === '') {
                $errors[] = 'Le mot de passe actuel est obligatoire.';
            } elseif (!$passwordHasher->isPasswordValid($user, $currentPassword)) {
                $errors[] = 'Le mot de passe actuel est incorrect.';
            }

            if ($newPassword === '') {
                $errors[] = 'Le nouveau mot de passe est obligatoire.';
            } elseif (mb_strlen($newPassword) < 8) {
                $errors[] = 'Le nouveau mot de passe doit contenir au moins 8 caracteres.';
            }

            if ($confirmPassword === '') {
                $errors[] = 'La confirmation du nouveau mot de passe est obligatoire.';
            } elseif ($newPassword !== $confirmPassword) {
                $errors[] = 'Le nouveau mot de passe et sa confirmation doivent etre identiques.';
            }

            if ($newPassword !== '' && $currentPassword !== '' && $newPassword === $currentPassword) {
                $errors[] = 'Le nouveau mot de passe doit etre different du mot de passe actuel.';
            }

            if ($errors === []) {
                $user->setPassword($passwordHasher->hashPassword($user, $newPassword));
                $entityManager->flush();

                $this->addFlash('success', 'Mot de passe mis a jour.');

                return $this->redirectToRoute('app_profile');
            }
        }

        return $this->render('site/profile-password.html.twig', [
            'user' => $user,
            'errors' => $errors,
        ]);
    }

    #[Route('/search', name: 'app_search', methods: ['GET'])]
    #[Route('/search.html', name: 'app_search_html', methods: ['GET'])]
    public function search(Request $request, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges): Response
    {
        return $this->renderSearchPage($request, $users, $machines, $formations, $badges);
    }

    #[Route('/recherche', name: 'app_recherche', methods: ['GET'])]
    #[Route('/recherche.html', name: 'app_recherche_html', methods: ['GET'])]
    public function recherche(Request $request, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges): Response
    {
        return $this->renderSearchPage($request, $users, $machines, $formations, $badges);
    }

    private function renderSearchPage(Request $request, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges): Response
    {
        $query = trim((string) $request->query->get('q', ''));
        $categories = $this->buildSearchCategories($query, $users, $machines, $formations, $badges);

        return $this->render('site/search.html.twig', [
            'query' => $query,
            'categories' => $categories,
            'totalResults' => array_sum(array_map('count', $categories)),
        ]);
    }

    private function generateUniqueUsername(string $email, UtilisateurRepository $users): string
    {
        $localPart = trim((string) preg_replace('/[^a-z0-9._-]+/i', '-', strstr($email, '@', true) ?: $email), '-._');
        $base = mb_strtolower($localPart !== '' ? $localPart : 'utilisateur');
        $base = mb_substr($base, 0, 220);
        $username = $base;
        $suffix = 2;

        while ($users->findOneBy(['username' => $username]) instanceof Utilisateur) {
            $username = sprintf('%s-%d', $base, $suffix++);
        }

        return $username;
    }

    private function buildSearchCategories(string $query, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges): array
    {
        $categories = [
            'Utilisateurs' => [],
            'Machines' => [],
            'Formations' => [],
            'Badges' => [],
        ];
        $needle = mb_strtolower(trim($query));
        if ($needle === '') {
            return $categories;
        }

        if ($this->isGranted('ROLE_ADMIN')) {
            foreach ($users->findAll() as $user) {
                $haystack = mb_strtolower(implode(' ', [
                    $user->getFirstName() ?? '',
                    $user->getLastName() ?? '',
                    $user->getUsername(),
                    $user->getEmail(),
                    $user->getIdentifiantRfid() ?? '',
                    $user->getNumeroId() ?? '',
                ]));
                if (str_contains($haystack, $needle)) {
                    $categories['Utilisateurs'][] = [
                        'title' => $user->getDisplayName(),
                        'description' => trim($user->getEmail() . ' ' . ($user->getIdentifiantRfid() ? '- RFID ' . $user->getIdentifiantRfid() : '')),
                        'meta' => $user->getUsername(),
                        'url' => $this->generateUrl('app_admin_user_detail', ['id' => $user->getId()]),
                    ];
                }
            }
        }

        foreach ($machines->findAll() as $machine) {
            $haystack = mb_strtolower(implode(' ', [
                $machine->getNom(),
                $machine->getDescription() ?? '',
                $machine->getLocalisation() ?? '',
                $machine->getMachineToken(),
            ]));
            if (str_contains($haystack, $needle)) {
                $categories['Machines'][] = [
                    'title' => $machine->getNom(),
                    'description' => $machine->getDescription() ?: 'Sans description',
                    'meta' => trim(($machine->getLocalisation() ?: 'Localisation non renseignée') . ' - ' . $machine->getStatut()),
                    'url' => $this->generateUrl('app_machine_detail', ['id' => $machine->getId()]),
                ];
            }
        }

        foreach ($formations->findVisible(['id' => 'DESC']) as $formation) {
            $haystack = mb_strtolower($formation->getTitre() . ' ' . ($formation->getDescription() ?? ''));
            if (str_contains($haystack, $needle)) {
                $categories['Formations'][] = [
                    'title' => $formation->getTitre(),
                    'description' => $formation->getDescription() ?: 'Sans description',
                    'meta' => $formation->getBadge()?->getNom() ? 'Badge : ' . $formation->getBadge()->getNom() : 'Aucun badge associé',
                    'url' => $this->generateUrl('app_formation_detail', ['id' => $formation->getId()]),
                ];
            }
        }

        foreach ($badges->findAll() as $badge) {
            $haystack = mb_strtolower(implode(' ', [$badge->getNom(), $badge->getDescription() ?? '', $badge->getIcone() ?? '']));
            if (str_contains($haystack, $needle)) {
                $categories['Badges'][] = [
                    'title' => $badge->getNom(),
                    'description' => $badge->getDescription() ?: 'Sans description',
                    'meta' => $badge->getIcone() ? 'Icône : ' . $badge->getIcone() : 'Icône non renseignée',
                    'url' => $this->generateUrl('app_admin_badges'),
                ];
            }
        }

        return $categories;
    }

    /**
     * @param iterable<Formation> $formations
     * @return array<int, array{type: string, icon: string, src: ?string, label: string}>
     */
    private function buildFormationVisuals(iterable $formations): array
    {
        $visuals = [];
        foreach ($formations as $formation) {
            if ($formation->getId() !== null) {
                $visuals[$formation->getId()] = $this->buildFormationVisual($formation);
            }
        }

        return $visuals;
    }

    /** @return array{type: string, icon: string, src: ?string, label: string} */
    private function buildFormationVisual(Formation $formation): array
    {
        $image = trim((string) $formation->getImage());
        if ($image !== '') {
            if (preg_match('#^(https?://|/|images/|uploads/)#', $image) === 1) {
                return ['type' => 'image', 'icon' => 'generic', 'src' => $image, 'label' => $formation->getTitre()];
            }

            return ['type' => 'icon', 'icon' => $this->normalizeFormationIconSlug($image), 'src' => null, 'label' => $formation->getTitre()];
        }


        return ['type' => 'icon', 'icon' => 'generic', 'src' => null, 'label' => $formation->getTitre()];
    }

    private function inferMachineIconSlug(string $token, string $name, string $fallback): string
    {
        $haystack = mb_strtolower($token . ' ' . $name);
        if (str_contains($haystack, 'laser')) {
            return 'laser';
        }
        if (str_contains($haystack, 'soudure')) {
            return 'soldering';
        }
        if (str_contains($haystack, 'vinyl') || str_contains($haystack, 'vinyle')) {
            return 'vinyl';
        }
        if (str_contains($haystack, 'cnc') || str_contains($haystack, 'fraiseuse')) {
            return 'cnc';
        }
        if (str_contains($haystack, 'oscilloscope')) {
            return 'oscilloscope';
        }
        if (str_contains($haystack, 'brodeuse')) {
            return 'embroidery';
        }

        return $this->normalizeFormationIconSlug($fallback);
    }

    private function normalizeFormationIconSlug(string $slug): string
    {
        return match ($slug) {
            'printer', 'printer-3d', 'impression-3d' => 'printer-3d',
            'decoupe', 'laser-co2' => 'laser',
            'electronique' => 'soldering',
            'textile' => 'embroidery',
            default => $slug,
        };
    }


    private function getLeaderboardRoleLabel(Utilisateur $user): string
    {
        $roles = $user->getRoles();
        if (in_array('ROLE_ADMIN', $roles, true)) {
            return 'Admin';
        }
        if (in_array('ROLE_STAFF', $roles, true)) {
            return 'Staff';
        }

        return 'Utilisateur';
    }

    private function formatMinutes(int $minutes): string
    {
        $minutes = max(0, $minutes);
        $hours = intdiv($minutes, 60);
        $remaining = $minutes % 60;

        if ($hours > 0 && $remaining > 0) {
            return sprintf('%dh%02d', $hours, $remaining);
        }

        if ($hours > 0) {
            return sprintf('%dh', $hours);
        }

        return sprintf('%d min', $remaining);
    }

    private function calculateOverlapMinutes(\DateTimeImmutable $start, \DateTimeImmutable $end, \DateTimeImmutable $rangeStart, \DateTimeImmutable $rangeEnd): int
    {
        $effectiveStart = max($start->getTimestamp(), $rangeStart->getTimestamp());
        $effectiveEnd = min($end->getTimestamp(), $rangeEnd->getTimestamp());
        if ($effectiveEnd <= $effectiveStart) {
            return 0;
        }

        return (int) max(0, round(($effectiveEnd - $effectiveStart) / 60));
    }

    private function isPrintMachine(Machine $machine): bool
    {
        $haystack = mb_strtolower(implode(' ', array_filter([
            $machine->getNom(),
            $machine->getCategorySlug(),
            $machine->getCategoryLabel(),
            $machine->getMachineToken(),
        ], static fn (?string $value): bool => $value !== null && trim($value) !== '')));

        foreach (['impression', 'print', 'printer', 'imprimante', 'ultimaker', 'prusa', 'bambu'] as $keyword) {
            if (str_contains($haystack, $keyword)) {
                return true;
            }
        }

        return false;
    }

    /** @return array{0: ?\DateTimeImmutable, 1: ?\DateTimeImmutable, 2: string} */
    private function resolveLeaderboardPeriod(string $period): array
    {
        $timezone = new \DateTimeZone('Europe/Paris');
        $now = new \DateTimeImmutable('now', $timezone);

        return match ($period) {
            'month' => [$now->modify('first day of this month')->setTime(0, 0), $now->modify('first day of next month')->setTime(0, 0), 'ce mois-ci'],
            'all' => [null, null, 'depuis toujours'],
            default => [$now->modify('monday this week')->setTime(0, 0), $now->modify('monday next week')->setTime(0, 0), 'cette semaine'],
        };
    }

    /**
     * If the uploaded image for the given form field exceeds $maxBytes, replace it in-place
     * (inside the request's file bag, before the form binds to it) with a resized/re-encoded
     * copy that fits under the limit, instead of letting it fail form validation outright.
     */
    private function compressOversizedCreationImage(Request $request, string $formName, int $maxBytes = 3 * 1024 * 1024): void
    {
        $files = $request->files->get($formName);
        if (!is_array($files) || !($files['imageUpload'] ?? null) instanceof UploadedFile) {
            return;
        }

        $original = $files['imageUpload'];
        if (!$original->isValid() || $original->getSize() === false || $original->getSize() <= $maxBytes) {
            return;
        }

        $mimeType = $original->getMimeType();
        if (!in_array($mimeType, ['image/jpeg', 'image/png', 'image/webp'], true)) {
            return;
        }

        $compressed = $this->createCompressedImageCopy($original->getPathname(), $mimeType, $maxBytes);
        if ($compressed === null) {
            return;
        }
        [$compressedPath, $finalMimeType] = $compressed;

        $files['imageUpload'] = new UploadedFile(
            $compressedPath,
            $original->getClientOriginalName(),
            $finalMimeType,
            null,
            true, // "test" mode: allows using a file that isn't a genuine HTTP upload
        );
        $request->files->set($formName, $files);
    }

    /**
     * GD decodes raw pixel data and ignores the EXIF "Orientation" tag that cameras/phones use
     * to say "display this rotated" — so once we re-encode via GD, that correction is lost
     * unless we bake the rotation into the pixels ourselves first.
     */
    private function applyExifOrientation(\GdImage $image, string $sourcePath): \GdImage
    {
        if (!function_exists('exif_read_data')) {
            return $image;
        }

        $exif = @exif_read_data($sourcePath);
        $orientation = is_array($exif) ? ($exif['Orientation'] ?? 1) : 1;

        // imagerotate() angles are counter-clockwise.
        $angle = match ($orientation) {
            3 => 180.0,
            6 => -90.0,
            8 => 90.0,
            default => 0.0,
        };

        if ($angle === 0.0) {
            return $image;
        }

        $rotated = imagerotate($image, $angle, 0);
        if ($rotated === false) {
            return $image;
        }

        imagedestroy($image);

        return $rotated;
    }

    /** @return array{0: string, 1: string}|null [tmpPath, finalMimeType] */
    private function createCompressedImageCopy(string $sourcePath, string $mimeType, int $maxBytes): ?array
    {
        if (!extension_loaded('gd')) {
            return null;
        }

        $image = match ($mimeType) {
            'image/jpeg' => @imagecreatefromjpeg($sourcePath),
            'image/png' => @imagecreatefrompng($sourcePath),
            'image/webp' => @imagecreatefromwebp($sourcePath),
            default => false,
        };

        if ($image === false) {
            return null;
        }

        $image = $this->applyExifOrientation($image, $sourcePath);

        $width = imagesx($image);
        $height = imagesy($image);
        $maxDimension = 2200;

        if ($width > $maxDimension || $height > $maxDimension) {
            $ratio = min($maxDimension / $width, $maxDimension / $height);
            $newWidth = max(1, (int) round($width * $ratio));
            $newHeight = max(1, (int) round($height * $ratio));

            $resized = imagecreatetruecolor($newWidth, $newHeight);
            imagealphablending($resized, false);
            imagesavealpha($resized, true);
            imagecopyresampled($resized, $image, 0, 0, 0, 0, $newWidth, $newHeight, $width, $height);
            imagedestroy($image);
            $image = $resized;
        }

        $result = $this->encodeImageUnderLimit($image, $mimeType, $maxBytes);

        // PNG is lossless: shrinking dimensions alone often isn't enough for photographic
        // content. Fall back to a JPEG re-encode (flattened onto white) as a last resort.
        if ($result === null && $mimeType === 'image/png') {
            $flatWidth = imagesx($image);
            $flatHeight = imagesy($image);
            $flattened = imagecreatetruecolor($flatWidth, $flatHeight);
            imagefill($flattened, 0, 0, imagecolorallocate($flattened, 255, 255, 255));
            imagecopy($flattened, $image, 0, 0, 0, 0, $flatWidth, $flatHeight);
            $result = $this->encodeImageUnderLimit($flattened, 'image/jpeg', $maxBytes);
            imagedestroy($flattened);
        }

        imagedestroy($image);

        return $result;
    }

    /** @return array{0: string, 1: string}|null [tmpPath, mimeType] */
    private function encodeImageUnderLimit(\GdImage $image, string $mimeType, int $maxBytes): ?array
    {
        $tmpPath = tempnam(sys_get_temp_dir(), 'fabos_creation_img_');
        if ($tmpPath === false) {
            return null;
        }

        $quality = 85;
        while (true) {
            $saved = match ($mimeType) {
                'image/png' => imagepng($image, $tmpPath, 6),
                'image/webp' => imagewebp($image, $tmpPath, $quality),
                default => imagejpeg($image, $tmpPath, $quality),
            };

            if (!$saved) {
                @unlink($tmpPath);
                return null;
            }

            clearstatcache(true, $tmpPath);
            $size = filesize($tmpPath);

            // PNG's "quality" arg is compression effort, not visual quality: looping it won't
            // shrink the file further, so a single pass is it (caller may fall back to JPEG).
            if ($mimeType === 'image/png' || ($size !== false && $size <= $maxBytes) || $quality <= 40) {
                break;
            }

            $quality -= 15;
        }

        clearstatcache(true, $tmpPath);
        $finalSize = filesize($tmpPath);
        if ($finalSize === false || $finalSize > $maxBytes) {
            @unlink($tmpPath);
            return null;
        }

        return [$tmpPath, $mimeType];
    }

    /**
     * Normalizes the free-text tags field into a clean comma-separated list on the entity:
     * trimmed, de-duplicated (case-insensitive), max 8 tags of up to 30 chars each.
     * @param FormInterface<Creation> $form
     */
    private function applyPublicCreationTags(Creation $creation, FormInterface $form): void
    {
        $raw = (string) $form->get('tagsInput')->getData();
        $tags = [];
        $seen = [];

        foreach (explode(',', $raw) as $tag) {
            $tag = trim(preg_replace('/\s+/', ' ', $tag) ?? '');
            if ($tag === '' || mb_strlen($tag) > 30) {
                continue;
            }
            $key = mb_strtolower($tag);
            if (isset($seen[$key])) {
                continue;
            }
            $seen[$key] = true;
            $tags[] = $tag;
            if (count($tags) >= 8) {
                break;
            }
        }

        $creation->setTags($tags === [] ? null : implode(', ', $tags));
    }

    /** @param FormInterface<Creation> $form */
    private function applyPublicCreationDuration(Creation $creation, FormInterface $form): void
    {
        $rawDuration = trim((string) $form->get('printDurationFormatted')->getData());

        if ($rawDuration === '') {
            $creation->setPrintDurationMinutes(null);
            return;
        }

        if (preg_match('/^(\d{1,3}):([0-5]\d)$/', $rawDuration, $matches) !== 1) {
            // Invalid format: leave the entity untouched, the form's own Regex
            // constraint on printDurationFormatted will fail validation with a clear message.
            return;
        }

        $creation->setPrintDurationMinutes(((int) $matches[1] * 60) + (int) $matches[2]);
    }

    private function normalizePublicCreationData(Creation $creation): void
    {
        $creation
            ->setTitle(trim($creation->getTitle()))
            ->setDescription($this->nullableString($creation->getDescription()))
            ->setExternalUrl($this->nullableString($creation->getExternalUrl()))
            ->setPrintDurationMinutes($creation->getPrintDurationMinutes() !== null ? max(0, $creation->getPrintDurationMinutes()) : null);
    }

    /** @param FormInterface<Creation> $form */
    private function handlePublicCreationUploads(Creation $creation, FormInterface $form, SluggerInterface $slugger, bool $imageRequired): bool
    {
        return $this->handlePublicCreationImageUpload($creation, $form, $slugger, $imageRequired)
            && $this->handlePublicCreationFileUpload($creation, $form, $slugger);
    }

    /** @param FormInterface<Creation> $form */
    private function handlePublicCreationImageUpload(Creation $creation, FormInterface $form, SluggerInterface $slugger, bool $required): bool
    {
        $uploadedFile = $form->get('imageUpload')->getData();
        if (!$uploadedFile instanceof UploadedFile) {
            if ($required) {
                $form->get('imageUpload')->addError(new FormError('Ajoute une image de ta création.'));
                return false;
            }

            return true;
        }

        $extension = strtolower($uploadedFile->guessExtension() ?: $uploadedFile->getClientOriginalExtension() ?: 'bin');
        if ($extension === 'jpeg') {
            $extension = 'jpg';
        }

        if (!in_array($extension, ['png', 'jpg', 'webp'], true)) {
            $form->get('imageUpload')->addError(new FormError('Choisis une image PNG, JPG, JPEG ou WEBP.'));
            return false;
        }

        $uploadDir = $this->getParameter('kernel.project_dir') . '/public/uploads/creations/images';
        if (!is_dir($uploadDir) && !mkdir($uploadDir, 0775, true) && !is_dir($uploadDir)) {
            $form->get('imageUpload')->addError(new FormError('Impossible de créer le dossier des images de créations.'));
            return false;
        }

        $fileName = $this->buildPublicCreationFileName($creation, $slugger, $extension);
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
    private function handlePublicCreationFileUpload(Creation $creation, FormInterface $form, SluggerInterface $slugger): bool
    {
        $uploadedFile = $form->get('fileUpload')->getData();
        if (!$uploadedFile instanceof UploadedFile) {
            return true;
        }

        $extension = strtolower($uploadedFile->getClientOriginalExtension() ?: $uploadedFile->guessExtension() ?: '');
        $allowedExtensions = ['stl', '3mf', 'obj', 'step', 'pdf', 'zip', 'afdesign'];
        if (!in_array($extension, $allowedExtensions, true)) {
            $form->get('fileUpload')->addError(new FormError('Choisis un fichier STL, 3MF, OBJ, STEP, PDF, ZIP ou AFDESIGN.'));
            return false;
        }

        $uploadDir = $this->getParameter('kernel.project_dir') . '/public/uploads/creations/files';
        if (!is_dir($uploadDir) && !mkdir($uploadDir, 0775, true) && !is_dir($uploadDir)) {
            $form->get('fileUpload')->addError(new FormError('Impossible de créer le dossier des fichiers de créations.'));
            return false;
        }

        $fileName = $this->buildPublicCreationFileName($creation, $slugger, $extension);
        try {
            $uploadedFile->move($uploadDir, $fileName);
        } catch (FileException) {
            $form->get('fileUpload')->addError(new FormError('Impossible de copier le fichier projet.'));
            return false;
        }

        $creation->setFileFilename($fileName);

        return true;
    }

    /** @param iterable<Machine> $machines */
    private function buildCalendarBookingAccess(iterable $machines, MachineQualificationService $machineAccess): array
    {
        $currentUser = $this->getUser();
        $isAdmin = $this->isGranted('ROLE_ADMIN');
        $accessByMachine = [];

        foreach ($machines as $machine) {
            $machineId = $machine->getId();
            if ($machineId === null) {
                continue;
            }

            if ($isAdmin) {
                $accessByMachine[$machineId] = [
                    'canReserve' => true,
                    'reason' => null,
                    'reasonLabel' => null,
                    'physicalTrainingRequired' => false,
                    'physicalTrainingCompleted' => true,
                    'adminBypass' => true,
                ];
                continue;
            }

            if (!$currentUser instanceof Utilisateur) {
                $accessByMachine[$machineId] = [
                    'canReserve' => false,
                    'reason' => 'login_required',
                    'reasonLabel' => 'Connexion nécessaire',
                    'physicalTrainingRequired' => false,
                    'physicalTrainingCompleted' => false,
                    'adminBypass' => false,
                ];
                continue;
            }

            $status = $machineAccess->getStatus($machine, $currentUser);
            $reason = $status['trainingBlockReason'] ?? null;
            $accessByMachine[$machineId] = [
                'canReserve' => (bool) $status['authorized'],
                'reason' => $reason,
                'reasonLabel' => $reason === 'physical_training_required'
                    ? 'Formation pratique non validée'
                    : ($reason === 'training_required' ? 'Formation requise' : null),
                'physicalTrainingRequired' => (bool) ($status['physicalTrainingRequired'] ?? false),
                'physicalTrainingCompleted' => (bool) ($status['physicalTrainingCompleted'] ?? false),
                'adminBypass' => false,
            ];
        }

        return $accessByMachine;
    }



    private function deleteUnusedPreviousBanner(?string $previousBannerFilename, string $uploadDir, UtilisateurRepository $users): void
    {
        if (!$previousBannerFilename) {
            return;
        }

        $stillUsed = $users->findOneBy([
            'bannerFilename' => $previousBannerFilename,
        ]);

        if ($stillUsed) {
            return;
        }

        $previousPath = rtrim($uploadDir, DIRECTORY_SEPARATOR) . DIRECTORY_SEPARATOR . $previousBannerFilename;

        if (is_file($previousPath)) {
            @unlink($previousPath);
        }
    }

    private function buildPublicCreationFileName(Creation $creation, SluggerInterface $slugger, string $extension): string
    {
        $baseName = strtolower($slugger->slug($creation->getTitle())->toString());
        if ($baseName === '') {
            $baseName = 'creation';
        }

        return sprintf('%s-%s.%s', $baseName, bin2hex(random_bytes(4)), $extension);
    }

    private function nullableString(?string $value): ?string
    {
        $value = trim((string) $value);

        return $value === '' ? null : $value;
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
}
