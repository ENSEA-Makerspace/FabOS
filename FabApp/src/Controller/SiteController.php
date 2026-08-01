<?php

namespace App\Controller;

use App\Service\SiteSettingService;
use App\Entity\Badge;
use App\Entity\Creation;
use App\Entity\CreationVote;
use App\Entity\Event;
use App\Entity\LabPage;
use App\Entity\Place;
use App\Form\CreationUserType;
use App\Repository\AccessRfidLogRepository;
use App\Repository\BadgeRepository;
use App\Repository\CreationRepository;
use App\Repository\CreationVoteRepository;
use App\Repository\EventRegistrationRepository;
use App\Repository\EventRepository;
use App\Repository\FormationRepository;
use App\Repository\LabPageRepository;
use App\Repository\LogUtilisationRepository;
use App\Repository\PlaceRepository;
use App\Repository\MachineFavoriteRepository;
use App\Repository\MachineRepository;
use App\Repository\LoanRepository;
use App\Repository\MaintenanceTaskRepository;
use App\Repository\MaterialRepository;
use App\Repository\ProgressionRepository;
use App\Repository\ReservationRepository;
use App\Reservation\ReservableResolver;
use App\Reservation\NextFreeSlotService;
use App\Reservation\ReservableType;
use App\Reservation\ReservationService;
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
use App\Service\BookingIdentityPolicy;
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
use App\Event\EventLocationResolver;
use App\Event\EventRegistrationService;
use App\Mail\NotificationCategory;
use App\Mail\NotificationPreferences;
use App\Feature\SiteFeatureService;
use App\Portal\PortalHome;
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
        PortalHome $portalHome,
    ): Response
    {
        // A portal can open on one of its features instead of the block homepage.
        //
        // ⚠️ **302, and it must stay 302.** This is a redirect the operator can
        // change from a form; a 301 would be cached by every browser that saw it
        // and would keep sending people to the old page long after the setting
        // changed — with nothing on the server left to explain why.
        //
        // A *redirect* rather than rendering the target here: no controller is
        // duplicated, and the address bar ends up honest about which page you are
        // actually on.
        $home = $portalHome->redirectRoute();
        if ($home !== null) {
            return $this->redirectToRoute($home);
        }

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
        if ($visibility['upcoming_events'] ?? false) {
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
        SiteFeatureService $modules,
        ReservableResolver $reservables,
        PlaceRepository $places,
        BookingIdentityPolicy $bookingIdentity,
    ): Response {
        $reservationRows = $reservations->findAllActive(['dateDebut' => 'ASC']);
        $reservables->warm($reservationRows);

        // Each resource layer is drawn only when its module is on. Equipment is
        // no longer special: an events-only or training-only deployment gets a
        // calendar with no equipment column, and FeatureAccessSubscriber 404s the
        // page outright once no layer is left.
        $machineRows = $modules->isEnabled('machines') ? $machines->findBy([], ['nom' => 'ASC']) : [];
        $placeRows = $modules->isEnabled('places') ? $places->findBy([], ['nom' => 'ASC']) : [];

        $resources = $this->buildCalendarResources($machineRows, $placeRows);

        // Grouped here rather than with Twig's `filter`, which returns a lazy
        // iterator that `is not empty` silently reports as empty — the booking
        // picker rendered no options at all until this moved into PHP.
        $resourcesByKind = [];
        foreach ($resources as $resource) {
            $resourcesByKind[$resource['kind']][] = $resource;
        }

        return $this->render('site/calendrier.html.twig', [
            'machines' => $machineRows,
            'resources' => $resources,
            'resourcesByKind' => $resourcesByKind,
            'reservations' => $reservationRows,
            'openingHoursJson' => $openingHours->getOpeningHoursForJson(),
            'calendarStartHour' => $openingHours->getCalendarStartHour(),
            'calendarEndHour' => $openingHours->getCalendarEndHour(),
            'bookingAccess' => $this->buildCalendarResourceAccess($machineRows, $placeRows, $machineAccess),
            'upcomingEvents' => $modules->isEnabled('events') ? $events->findUpcoming(6) : [],
            'showBookerIdentity' => $bookingIdentity->canSeeOthersIdentity(),
            'viewerId' => $bookingIdentity->viewerId(),
        ]);
    }

    /**
     * The calendar's bookable resources, machines and spaces together.
     *
     * Everything downstream keys off `kind:id` rather than a bare id: the two
     * kinds have overlapping id sequences, so machine 2 and space 2 would
     * otherwise be the same row to the filter list, the grid and the access map.
     *
     * @param Machine[] $machines
     * @param Place[]   $places
     *
     * @return array<int, array<string, mixed>>
     */
    private function buildCalendarResources(array $machines, array $places): array
    {
        $resources = [];

        foreach ($machines as $machine) {
            $id = $machine->getId();
            if ($id === null) {
                continue;
            }

            $resources[] = [
                'key' => ReservableType::Machine->value . ':' . $id,
                'kind' => ReservableType::Machine->value,
                'id' => $id,
                'name' => $machine->getNom(),
                'status' => $machine->getStatut(),
                'statusKey' => str_replace(' ', '-', mb_strtolower($machine->getStatut())),
                'category' => $machine->getCategoryLabel(),
            ];
        }

        foreach ($places as $place) {
            $id = $place->getId();
            if ($id === null) {
                continue;
            }

            // Spaces have no operational status of their own, so they borrow the
            // "available" pill rather than inventing a vocabulary for one value.
            $capacity = $place->getCapacite();
            $resources[] = [
                'key' => ReservableType::Place->value . ':' . $id,
                'kind' => ReservableType::Place->value,
                'id' => $id,
                'name' => $place->getNom(),
                'status' => 'Disponible',
                'statusKey' => 'disponible',
                'category' => $capacity !== null ? sprintf('Espace · %d places', $capacity) : 'Espace',
            ];
        }

        return $resources;
    }

    /**
     * Booking access per resource, keyed the same `kind:id` way.
     *
     * Machines carry the certification verdict; spaces are open to any signed-in
     * member, which is the rule the place form already applied — this just says
     * so in the shape the calendar reads.
     *
     * @param Machine[] $machines
     * @param Place[]   $places
     *
     * @return array<string, array<string, mixed>>
     */
    private function buildCalendarResourceAccess(array $machines, array $places, MachineQualificationService $machineAccess): array
    {
        $access = [];

        foreach ($this->buildCalendarBookingAccess($machines, $machineAccess) as $machineId => $row) {
            $access[ReservableType::Machine->value . ':' . $machineId] = $row;
        }

        $isAuthenticated = $this->getUser() instanceof Utilisateur;
        foreach ($places as $place) {
            $id = $place->getId();
            if ($id === null) {
                continue;
            }

            $access[ReservableType::Place->value . ':' . $id] = [
                'canReserve' => $isAuthenticated,
                'reason' => $isAuthenticated ? null : 'login_required',
                'reasonLabel' => $isAuthenticated ? null : 'Connexion nécessaire',
                'physicalTrainingRequired' => false,
                'physicalTrainingCompleted' => true,
                'adminBypass' => false,
            ];
        }

        return $access;
    }

    #[Route('/mes-reservations', name: 'app_my_reservations', methods: ['GET'])]
    #[IsGranted('ROLE_USER')]
    public function myReservations(ReservationRepository $reservations, ReservableResolver $reservables, SiteSettingService $siteSettings): Response
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException('Authentification requise');
        }

        $items = $reservations->findForUser($user, ['dateDebut' => 'DESC']);
        $reservables->warm($items);
        $now = new \DateTimeImmutable('now', $this->labZone($siteSettings));
        $current = [];
        $upcoming = [];
        $past = [];
        $cancelled = [];
        $nextReservation = null;

        foreach ($items as $reservation) {
            // Declined requests group with cancellations — both are bookings the
            // user no longer has, and neither holds its slot any more.
            if (!$reservation->isActive()) {
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
    /**
     * The catalogue, on S59's list shape (2026-08-01).
     *
     * Three things changed and each was a deliberate call:
     *  · Categories are a persistent filter bar, not headings. Grouping the grid
     *    into a section per category left one card and three empty cells, six
     *    times down the page — a group of one is a row of one.
     *  · Filtering is server-side, so the page works without JavaScript and the
     *    URL is shareable. The old page filtered ~250 lines of inline JS over
     *    every card in the DOM.
     *  · Favourites are gone (S75, operator decision).
     *
     * ⚠️ Availability per card is 1 `NextFreeSlotService` call per machine, which
     * is the cost S47 refused and Phase H **S41** exists to fix with one batched
     * query. It is acceptable at this lab's size and it will not stay acceptable
     * — the prototype printed the number so the decision was taken against it.
     */
    public function machines(
        MachineRepository $machines,
        MachineQualificationService $qualification,
        NextFreeSlotService $nextFreeSlot,
        OpeningHoursProvider $hours,
        Request $request,
        SiteSettingService $siteSettings,
    ): Response {
        $user = $this->getUser();
        $user = $user instanceof Utilisateur ? $user : null;

        $search = trim((string) $request->query->get('q', ''));
        $category = trim((string) $request->query->get('cat', ''));

        // ⚠️ "Closed" belongs to the venue, not to the machine. Without this the
        // page renders every machine as "occupée" on a Saturday, which blames
        // eleven machines for the calendar and reads as entirely plausible.
        $now = new \DateTimeImmutable('now', $this->labZone($siteSettings));
        $todayOpen = $hours->getOpenMinutesFor($now);
        $nowMinutes = ((int) $now->format('H')) * 60 + (int) $now->format('i');
        $venueOpenNow = $todayOpen !== null
            && $nowMinutes >= $todayOpen['start'] && $nowMinutes < $todayOpen['end'];

        $rows = $machines->findBy([], ['nom' => 'ASC']);

        $cards = [];
        foreach ($rows as $machine) {
            if ($category !== '' && $machine->getCategorySlug() !== $category) {
                continue;
            }
            if ($search !== '' && stripos($machine->getNom(), $search) === false) {
                continue;
            }

            $status = $qualification->getStatus($machine, $user);
            $authorized = (bool) ($status['authorized'] ?? false);
            $down = \in_array(strtolower($machine->getStatut()), ['maintenance', 'panne'], true);

            // ⚠️ The user is passed only when qualified. With a user the service
            // applies their quotas and the answer means "you may book this"; with
            // null it is opening-hours-and-overlap only and means "the machine is
            // free then". That is what lets an untrained member see a real state
            // without being promised a slot they would be refused (S47).
            $slot = $down ? null : $nextFreeSlot->find(
                $authorized ? $user : null,
                ReservableType::Machine,
                (int) $machine->getId(),
            );

            // "Libre" has to mean free NOW, not "a slot exists this fortnight".
            $freeNow = $venueOpenNow && $slot !== null && $slot['start'] <= $now->modify('+60 minutes');

            $cards[] = [
                'machine' => $machine,
                'authorized' => $authorized,
                // ⚠️ `authorized` is false for an anonymous visitor even on a
                // machine that needs no training at all, so it cannot drive the
                // footer on its own: the first deploy told every logged-out
                // visitor that all eleven machines required a formation. Ask
                // whether the MACHINE has a requirement, separately from whether
                // THIS person meets it.
                'requiresTraining' => ($status['badgeRows'] ?? []) !== [],
                'down' => $down,
                'slot' => $slot,
                'freeNow' => $freeNow,
                'catSlug' => $machine->getCategorySlug(),
                'catLabel' => $machine->getCategoryLabel(),
            ];
        }

        usort($cards, static fn (array $a, array $b): int
            => [$a['catLabel'], $a['machine']->getNom()] <=> [$b['catLabel'], $b['machine']->getNom()]);

        // Tiles are counted over the UNFILTERED set, so picking one category does
        // not blank out the others' counts.
        $tiles = [];
        foreach ($rows as $machine) {
            $slug = $machine->getCategorySlug();
            $tiles[$slug] ??= ['slug' => $slug, 'label' => $machine->getCategoryLabel(), 'total' => 0, 'free' => 0];
            $tiles[$slug]['total']++;
        }
        foreach ($cards as $card) {
            if ($card['freeNow'] && isset($tiles[$card['catSlug']])) {
                $tiles[$card['catSlug']]['free']++;
            }
        }
        usort($tiles, static fn (array $a, array $b): int => $a['label'] <=> $b['label']);

        return $this->render('site/machines.html.twig', [
            'cards' => $cards,
            'tiles' => $tiles,
            'search' => $search,
            'category' => $category,
            'venueOpenNow' => $venueOpenNow,
            'totalCount' => \count($cards),
            'allCount' => \count($rows),
            'freeCount' => \count(array_filter($cards, static fn (array $c): bool => $c['freeNow'])),
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
        MaintenanceTaskRepository $maintenanceTasks,
        SiteFeatureService $modules,
        NextFreeSlotService $nextFreeSlot,
        ?int $id = null,
    ): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        $machine = $machines->find($id);
        if (!$machine) {
            throw $this->createNotFoundException('Machine introuvable');
        }

        $maintenanceEnabled = $modules->isEnabled('maintenance');
        $openMaintenance = $maintenanceEnabled ? $maintenanceTasks->findOpenForMachine($machine) : [];
        $maintenanceHealth = 'ok';
        foreach ($openMaintenance as $task) {
            if ($task->getEffectiveStatus() === 'overdue') { $maintenanceHealth = 'overdue'; break; }
            if ($task->getEffectiveStatus() === 'due_soon') { $maintenanceHealth = 'due_soon'; }
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

        // The answer the visitor came for, computed rather than made them hunt for
        // it in a grid (S47). Only worth asking when they could actually act on
        // it — a machine they are not cleared for gets the certification message
        // instead, and offering it a slot would be a promise `book()` refuses.
        $nextSlot = $hasRequiredBadge
            ? $nextFreeSlot->find(
                $currentUser instanceof Utilisateur ? $currentUser : null,
                ReservableType::Machine,
                $machine->getId(),
            )
            : null;

        return $this->render('site/machine-detail.html.twig', [
            'machine' => $machine,
            'nextSlot' => $nextSlot,
            'requiredBadgeRows' => $requiredBadgeRows,
            'hasRequiredBadge' => $hasRequiredBadge,
            'authorizationStatus' => $authorizationStatus,
            'rfidLogCount' => $rfidLogs->count(['machine' => $machine]),
            'usageLogCount' => $usageLogs->count(['machine' => $machine]),
            'reservationCount' => $reservations->countForReservable(ReservableType::Machine, $machine->getId()),
            'favoritesEnabled' => $favoritesEnabled,
            'isFavorite' => $isFavorite,
            'materialsEnabled' => $modules->isEnabled('materials'),
            'machineMaterials' => $materials->findByMachine($machine->getId()),
            'maintenanceEnabled' => $maintenanceEnabled,
            'openMaintenance' => $openMaintenance,
            'maintenanceHealth' => $maintenanceHealth,
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
        BookingIdentityPolicy $bookingIdentity,
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
            'reservations' => $reservations->findActiveForReservable(ReservableType::Machine, $machine->getId()),
            'openingHoursJson' => $openingHours->getOpeningHoursForJson(),
            'calendarStartHour' => $openingHours->getCalendarStartHour(),
            'calendarEndHour' => $openingHours->getCalendarEndHour(),
            'bookingAccess' => $bookingAccessByMachine[$machine->getId()] ?? null,
            'showBookerIdentity' => $bookingIdentity->canSeeOthersIdentity(),
            'viewerId' => $bookingIdentity->viewerId(),
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
            'reservations' => $reservations->findForReservable(ReservableType::Machine, $machine->getId()),
        ]);
    }

    #[Route('/formations', name: 'app_formations', methods: ['GET'])]
    #[Route('/formations.html', name: 'app_formations_html', methods: ['GET'])]
    public function formations(
        FormationRepository $formations,
        ProgressionRepository $progressions,
        TrainingPolicyService $trainingPolicy,
        Request $request,
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

        // ── S59 catalogue shape ──────────────────────────────────────────
        // ⚠️ `progressionStats` above is aggregated over ALL users; the card
        // needs THIS member's own progression, which is a different question.
        $user = $this->getUser();
        $user = $user instanceof Utilisateur ? $user : null;
        $mine = [];
        if ($user !== null) {
            foreach ($progressions->findBy(['utilisateur' => $user]) as $row) {
                $f = $row->getFormation();
                if ($f !== null) {
                    $mine[$f->getId()] = $row;
                }
            }
        }

        $search = trim((string) $request->query->get('q', ''));
        $category = trim((string) $request->query->get('cat', ''));

        $cards = [];
        foreach ($formationItems as $formation) {
            if ($category !== '' && ($formation->getCategorie() ?? '') !== $category) { continue; }
            if ($search !== '' && stripos($formation->getTitre(), $search) === false) { continue; }
            $p = $mine[$formation->getId()] ?? null;
            $cards[] = [
                'formation' => $formation,
                'started' => $p !== null,
                'completed' => $p !== null && $p->isCompleted(),
            ];
        }
        usort($cards, static fn (array $a, array $b): int
            => [$a['formation']->getCategorie() ?? '', $a['formation']->getTitre()]
            <=> [$b['formation']->getCategorie() ?? '', $b['formation']->getTitre()]);

        $tiles = [];
        foreach ($formationItems as $formation) {
            $slug = $formation->getCategorie() ?: '';
            if ($slug === '') { continue; }
            $tiles[$slug] ??= ['slug' => $slug, 'label' => $slug, 'total' => 0, 'free' => 0];
            $tiles[$slug]['total']++;
        }
        usort($tiles, static fn (array $a, array $b): int => $a['label'] <=> $b['label']);

        return $this->render('site/formations.html.twig', [
            'cards' => $cards,
            'tiles' => $tiles,
            'search' => $search,
            'category' => $category,
            'signedIn' => $user !== null,
            'totalCount' => \count($cards),
            'allCount' => \count($formationItems),
            'doneCount' => \count(array_filter($cards, static fn (array $c): bool => $c['completed'])),
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
        SiteSettingService $siteSettings,
    ): Response
    {
        $activeTab = in_array($request->query->get('tab'), ['presence', 'prints'], true) ? (string) $request->query->get('tab') : 'presence';
        $activePeriod = in_array($request->query->get('period'), ['week', 'month', 'all'], true) ? (string) $request->query->get('period') : 'week';
        [$periodStart, $periodEnd, $periodLabel] = $this->resolveLeaderboardPeriod($activePeriod, $siteSettings);

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
            // A `stats` array carrying a user count, a machine count and a
            // published-creation count used to be built here. The template never
            // rendered any of it, so all it did was make a page that ranks
            // *people* query the equipment and project tables — two modules it
            // has nothing to do with — on every request. Removed with the split.
        ]);
    }

    /**
     * The gallery lived under `/leaderboard/creations*` until S22, which is a
     * path named after a feature a deployment may well have disabled. These are
     * public, linkable URLs that people may have bookmarked or shared, so the
     * old paths keep answering — permanently, and as a redirect rather than a
     * second route onto the same action, so the address bar tells the truth and
     * search engines are told which URL is the real one.
     *
     * The named routes are `app_creation_legacy_*`, which puts them under the
     * gallery's own module gate: with the gallery off these 404 like everything
     * else it owns, instead of redirecting to a page that then 404s.
     *
     * GET pages only. The vote and delete endpoints moved without redirects —
     * they are POST targets on our own forms, never a URL anyone holds, and a
     * 301 is not reliably re-POSTed anyway.
     */
    #[Route('/leaderboard/creations', name: 'app_creation_legacy_gallery', methods: ['GET'])]
    public function creationGalleryLegacy(): Response
    {
        return $this->redirectToRoute('app_creations', [], Response::HTTP_MOVED_PERMANENTLY);
    }

    #[Route('/leaderboard/creations/ranking', name: 'app_creation_legacy_ranking', methods: ['GET'])]
    public function creationsRankingLegacy(): Response
    {
        return $this->redirectToRoute('app_creations_ranking', [], Response::HTTP_MOVED_PERMANENTLY);
    }

    #[Route('/leaderboard/creations/new', name: 'app_creation_legacy_new', methods: ['GET'])]
    public function newCreationLegacy(): Response
    {
        return $this->redirectToRoute('app_creation_new', [], Response::HTTP_MOVED_PERMANENTLY);
    }

    #[Route('/creations', name: 'app_creations', methods: ['GET'])]
    public function creationGallery(Request $request, CreationRepository $creations, CreationVoteRepository $votes): Response
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

        return $this->render('site/creations.html.twig', [
            'creationRows' => $creationRows,
            'topRows' => $topRows,
            'activeSort' => $sort,
        ]);
    }

    #[Route('/creations/ranking', name: 'app_creations_ranking', methods: ['GET'])]
    public function creationsRanking(CreationRepository $creations, CreationVoteRepository $votes): Response
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

        return $this->render('site/creations-ranking.html.twig', [
            'rankingRows' => $rankingRows,
        ]);
    }

    #[Route('/creations/new', name: 'app_creation_new', methods: ['GET', 'POST'])]
    #[IsGranted('ROLE_USER')]
    public function newCreation(Request $request, EntityManagerInterface $entityManager, SluggerInterface $slugger): Response
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

                return $this->render('site/creation-new.html.twig', [
                    'creation' => $creation,
                    'form' => $form,
                ], new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY));
            }

            $entityManager->persist($creation);
            $entityManager->flush();
            $this->addFlash('success', 'Création publiée avec succès !');

            return $this->redirectToRoute('app_creations');
        }

        if ($form->isSubmitted()) {
            $this->addFlash('error', 'La création n’a pas été publiée. Vérifie les erreurs du formulaire.');
        }

        return $this->render('site/creation-new.html.twig', [
            'creation' => $creation,
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/creations/{id}/vote', name: 'app_creation_vote', requirements: ['id' => '\\d+'], methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function voteCreation(Creation $creation, Request $request, EntityManagerInterface $entityManager, CreationVoteRepository $votes): Response
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
            return $this->redirectToRoute('app_creations');
        }

        $rawRating = $request->request->get('rating');
        if (!is_numeric($rawRating)) {
            $this->addFlash('error', 'Choisis une note avant de confirmer.');
            return $this->redirectToRoute('app_creations');
        }

        $rating = (float) $rawRating;
        if ($rating < 0.5 || $rating > 5.0 || abs(($rating * 2) - round($rating * 2)) > 0.0001) {
            $this->addFlash('error', 'La note doit être comprise entre 0.5 et 5, par pas de 0.5.');
            return $this->redirectToRoute('app_creations');
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

        return $this->redirectToRoute('app_creations');
    }

    #[Route('/creations/{id}/delete', name: 'app_creation_delete', requirements: ['id' => '\\d+'], methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function deleteCreation(Creation $creation, Request $request, EntityManagerInterface $entityManager): Response
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

            return $this->redirectToRoute('app_creations');
        }

        $title = $creation->getTitle();
        $imageFilename = $creation->getImageFilename();
        $fileFilename = $creation->getFileFilename();

        $entityManager->remove($creation);
        $entityManager->flush();

        $this->deleteCreationUploadIfSafe('public/uploads/creations/images', $imageFilename);
        $this->deleteCreationUploadIfSafe('public/uploads/creations/files', $fileFilename);
        $this->addFlash('success', sprintf('Création "%s" supprimée.', $title));

        return $this->redirectToRoute('app_creations');
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
    /**
     * Badges, on the shared catalogue shells.
     *
     * ⚠️ A badge is the odd one in the set: it has no availability whatsoever,
     * and the only fact worth leading with is whether YOU hold it. So the state
     * slot stays empty and the footer — the slot that carries "what is true
     * about you acting on this thing" — does all the work. What the badge
     * unlocks goes in the meta line, which is the question a member browsing
     * this page is actually asking.
     */
    public function badges(BadgeRepository $badges, UtilisateurBadgeRepository $held, Request $request): Response
    {
        $user = $this->getUser();
        $user = $user instanceof Utilisateur ? $user : null;
        $search = trim((string) $request->query->get('q', ''));

        $rows = $badges->findBy([], ['nom' => 'ASC']);
        $cards = [];
        foreach ($rows as $badge) {
            if ($search !== '' && stripos($badge->getNom(), $search) === false) {
                continue;
            }
            // ⚠️ One query per badge. Same shape as the per-card availability on
            // /machines and the same answer: fine at this size, and Phase H's
            // S41 is where it stops being fine.
            $owned = $user !== null && $held->findOneBy(['utilisateur' => $user, 'badge' => $badge]) !== null;
            $cards[] = [
                'badge' => $badge,
                'owned' => $owned,
                'unlocks' => \count($badge->getMachineBadges()),
            ];
        }

        return $this->render('site/badges.html.twig', [
            'cards' => $cards,
            'search' => $search,
            'signedIn' => $user !== null,
            'totalCount' => \count($cards),
            'allCount' => \count($rows),
            'ownedCount' => \count(array_filter($cards, static fn (array $c): bool => $c['owned'])),
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
    /**
     * Espaces, on the same S59 catalogue shells as `/machines`.
     *
     * ⚠️ A room is NOT a machine and the differences are the interesting part:
     * no categories (so no tile bar), no badge requirement (so no permission
     * footer), and capacity in the footer instead. The shells take all three as
     * absence rather than as a special case — which is the test of whether they
     * are shells at all.
     */
    public function places(
        PlaceRepository $places,
        NextFreeSlotService $nextFreeSlot,
        OpeningHoursProvider $hours,
        Request $request,
        SiteSettingService $siteSettings,
    ): Response {
        $user = $this->getUser();
        $user = $user instanceof Utilisateur ? $user : null;
        $search = trim((string) $request->query->get('q', ''));

        $now = new \DateTimeImmutable('now', $this->labZone($siteSettings));
        $todayOpen = $hours->getOpenMinutesFor($now);
        $nowMinutes = ((int) $now->format('H')) * 60 + (int) $now->format('i');
        $venueOpenNow = $todayOpen !== null
            && $nowMinutes >= $todayOpen['start'] && $nowMinutes < $todayOpen['end'];

        $rows = $places->findBy([], ['nom' => 'ASC']);
        $cards = [];
        foreach ($rows as $place) {
            if ($search !== '' && stripos($place->getNom(), $search) === false) {
                continue;
            }
            $slot = $nextFreeSlot->find($user, ReservableType::Place, (int) $place->getId());
            $cards[] = [
                'place' => $place,
                'slot' => $slot,
                'freeNow' => $venueOpenNow && $slot !== null && $slot['start'] <= $now->modify('+60 minutes'),
            ];
        }

        return $this->render('site/places.html.twig', [
            'cards' => $cards,
            'search' => $search,
            'venueOpenNow' => $venueOpenNow,
            'totalCount' => \count($cards),
            'allCount' => \count($rows),
            'freeCount' => \count(array_filter($cards, static fn (array $c): bool => $c['freeNow'])),
        ]);
    }

    #[Route('/places/{id}', name: 'app_place_detail', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function placeDetail(Place $place, ReservationRepository $reservations, NextFreeSlotService $nextFreeSlot): Response
    {
        $currentUser = $this->getUser();

        // Booking a space meant typing a date and two times from nothing, on a
        // page that already knows the opening hours, the existing bookings and
        // the minimum notice (S47). The form opens on the next slot this person
        // could actually book instead.
        //
        // ⚠️ A suggestion, never a constraint: all three inputs stay editable,
        // and `ReservationService::book()` still validates — a slot pre-filled
        // here and taken since must still be refused there, and is.
        $suggestedSlot = $nextFreeSlot->find(
            $currentUser instanceof Utilisateur ? $currentUser : null,
            ReservableType::Place,
            $place->getId(),
        );

        return $this->render('site/place-detail.html.twig', [
            'place' => $place,
            'reservations' => $reservations->findActiveForReservable(ReservableType::Place, $place->getId()),
            'suggestedSlot' => $suggestedSlot,
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
            'reservations' => $reservations->findActiveForReservable(ReservableType::Place, $place->getId()),
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
    public function reservePlace(Place $place, Request $request, ReservationRepository $reservations, ReservationService $booking, SiteSettingService $siteSettings): Response
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

        try {
            $dateDebut = new \DateTimeImmutable($dateInput . ' ' . $startInput, $this->labZone($siteSettings));
            $dateFin = new \DateTimeImmutable($dateInput . ' ' . $endInput, $this->labZone($siteSettings));
        } catch (\Throwable) {
            return $this->renderPlaceBookingError($place, $reservations, $request, 'Date ou horaire invalide.');
        }

        $result = $booking->book(
            ReservableType::Place,
            $place->getId(),
            $user,
            $dateDebut,
            $dateFin,
            (string) $request->request->get('motif'),
        );

        if (!$result->ok) {
            return $this->renderPlaceBookingError($place, $reservations, $request, $result->message);
        }

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
    public function eventDetail(
        Event $event,
        EventRegistrationRepository $registrations,
        EventRegistrationService $registrationService,
        EventLocationResolver $locations,
    ): Response {
        $user = $this->getUser();

        return $this->render('site/event-detail.html.twig', [
            'event' => $event,
            'location' => $locations->resolve($event),
            'seatsTaken' => $registrations->countSeatsTaken($event),
            'seatsRemaining' => $registrationService->seatsRemaining($event),
            'waitlistCount' => $registrations->countWaitlisted($event),
            // Only a signed-in member's own registration is resolvable here; a
            // guest's place lives with their address and their signed mail link.
            'myRegistration' => $user instanceof Utilisateur
                ? $registrations->findOneForContact($event, $user->getEmail())
                : null,
            'registrationOpen' => $event->isRegistrationOpen(),
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
        SiteFeatureService $modules,
        NotificationPreferences $notificationPreferences,
        EventRegistrationRepository $eventRegistrations,
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
                ->setTheme($theme)
                ->setLangue($langue);

            // Per-category mail preferences live in their own table, not on the
            // user row — see NotificationPreferences for why they're opt-out rows.
            if (($userId = $user->getId()) !== null) {
                $accepted = [];
                foreach (NotificationCategory::OPTOUTABLE as $category) {
                    $accepted[$category] = $request->request->has('notify_' . $category);
                }

                $notificationPreferences->save($userId, $accepted);
            }

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
            'eventsEnabled' => $modules->isEnabled('events'),
            'myEventRegistrations' => $eventRegistrations->findForUser($user),
            'notificationCategories' => $user->getId() !== null
                ? $notificationPreferences->forUser($user->getId())
                : array_fill_keys(NotificationCategory::OPTOUTABLE, true),
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
    public function search(Request $request, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges, SiteFeatureService $modules): Response
    {
        return $this->renderSearchPage($request, $users, $machines, $formations, $badges, $modules);
    }

    #[Route('/recherche', name: 'app_recherche', methods: ['GET'])]
    #[Route('/recherche.html', name: 'app_recherche_html', methods: ['GET'])]
    public function recherche(Request $request, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges, SiteFeatureService $modules): Response
    {
        return $this->renderSearchPage($request, $users, $machines, $formations, $badges, $modules);
    }

    private function renderSearchPage(Request $request, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges, SiteFeatureService $modules): Response
    {
        $query = trim((string) $request->query->get('q', ''));
        $categories = $this->buildSearchCategories($query, $users, $machines, $formations, $badges, $modules);

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

    private function buildSearchCategories(string $query, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges, SiteFeatureService $modules): array
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

        // Equipment pages 404 when the module is off, so offering them as search
        // hits would hand the user a link straight into a dead end.
        foreach ($modules->isEnabled('machines') ? $machines->findAll() : [] as $machine) {
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
    private function resolveLeaderboardPeriod(string $period, SiteSettingService $siteSettings): array
    {
        $timezone = $this->labZone($siteSettings);
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

    /**
     * The lab's wall-clock zone, from the operator's setting.
     *
     * ⚠️ Human-entered times are parsed **and stored** in this zone, so the naive
     * string in the database keeps the time the person actually typed (the audit is
     * S38b in docs/HISTORY.md). Machine timestamps follow the opposite rule and are
     * stored UTC — those are converted at display time by the `|lab_date` filter,
     * never here.
     */
    private function labZone(SiteSettingService $siteSettings): \DateTimeZone
    {
        return new \DateTimeZone($siteSettings->getTimezone());
    }

}
