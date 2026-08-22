<?php

namespace App\Controller;

use App\Service\SiteSettingService;
use App\Entity\Reservation;
use App\Entity\Progression;
use App\Entity\Utilisateur;
use App\Repository\AccessRfidLogRepository;
use App\Repository\BadgeRepository;
use App\Repository\FormationRepository;
use App\Repository\LogUtilisationRepository;
use App\Repository\MachineRepository;
use App\Repository\ProgressionRepository;
use App\Repository\ReservationRepository;
use App\Reservation\BookingResult;
use App\Reservation\ReservableResolver;
use App\Reservation\ReservableType;
use App\Reservation\ReservationService;
use App\Reservation\Verb\BookingVerb;
use App\Reservation\Verb\VerbContext;
use App\Repository\SectionRepository;
use App\Repository\QuizRepository;
use App\Repository\QuestionRepository;
use App\Repository\ChoixRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Repository\UtilisateurRepository;
use App\Service\MachineQualificationService;
use App\Schedule\ScheduleResolver;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\JsonResponse;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

#[Route('/api')]
final class ApiController extends AbstractController
{
    public function __construct(private readonly ReservableResolver $reservables)
    {
    }

    #[Route('/navigation', name: 'api_navigation', methods: ['GET'])]
    public function navigation(): JsonResponse
    {
        return new JsonResponse([
            ['label' => 'Accueil', 'url' => $this->generateUrl('app_home')],
            ['label' => 'Calendrier', 'url' => $this->generateUrl('app_calendar')],
            ['label' => 'Machines', 'url' => $this->generateUrl('app_machines')],
            ['label' => 'Formations', 'url' => $this->generateUrl('app_formations')],
            ['label' => 'Leaderboard', 'url' => $this->generateUrl('app_leaderboard')],
        ]);
    }


    #[Route('/opening-hours', name: 'api_opening_hours', methods: ['GET'])]
    #[Route('/horaires', name: 'api_horaires', methods: ['GET'])]
    public function openingHours(ScheduleResolver $schedule): JsonResponse
    {
        // ⚠️ The default venue's week. This endpoint has no location
        // parameter, and inventing one here would be a public API change
        // made in passing; it belongs with the rest of S134e.
        return new JsonResponse($schedule->forJson(null));
    }

    #[Route('/machines', name: 'api_machines', methods: ['GET'])]
    public function machines(MachineRepository $machines, MachineQualificationService $machineAccess): JsonResponse
    {
        $user = $this->getUser();
        return new JsonResponse(array_map(
            fn ($machine) => $this->machineToArray(
                $machine,
                $user instanceof Utilisateur ? $user : null,
                $machineAccess,
            ),
            // ⚠️ S134b — an API that lists machines lists the live ones.
            $machines->findLive(),
        ));
    }

    #[Route('/machines/{id}', name: 'api_machine_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function machine(int $id, MachineRepository $machines, MachineQualificationService $machineAccess): JsonResponse
    {
        $machine = $machines->find($id);
        if (!$machine) {
            return new JsonResponse(['error' => 'Machine introuvable'], 404);
        }

        $user = $this->getUser();
        return new JsonResponse($this->machineToArray(
            $machine,
            $user instanceof Utilisateur ? $user : null,
            $machineAccess,
        ));
    }

    #[Route('/calendar', name: 'api_calendar', methods: ['GET'])]
    public function calendar(ReservationRepository $reservations): JsonResponse
    {
        $rows = $reservations->findAllActive(['dateDebut' => 'ASC']);
        $this->reservables->warm($rows);

        // ⚠️ This one appears in neither S38's list nor /api-docs, and it was the last
        // anonymous leak standing: every active booking with the booker's real name and
        // their free-text `motif`. `templates/site/calendrier.html.twig` does not read
        // it — that page server-renders its own rows — so nothing in the repo depended
        // on the identity fields. `pending` survives because the calendar draws a
        // request-awaiting-a-person differently from a confirmed booking (S38).
        return new JsonResponse(array_map(fn (Reservation $reservation): array => $this->reservationToOccupancyArray($reservation), $rows));
    }

    #[Route('/formations', name: 'api_formations', methods: ['GET'])]
    public function formations(FormationRepository $formations): JsonResponse
    {
        return new JsonResponse(array_map(fn ($formation): array => $this->formationToArray($formation), $formations->findAll()));
    }

    #[Route('/progressions', name: 'api_progressions', methods: ['GET'])]
    public function progressions(ProgressionRepository $progressions): JsonResponse
    {
        // Named people paired with their training scores. Undocumented in /api-docs,
        // so no public contract is broken by closing it. Gated rather than narrowed:
        // strip the identity and nothing useful is left (S38).
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return new JsonResponse(array_map(static function ($progression): array {
            $user = $progression->getUtilisateur();
            $formation = $progression->getFormation();

            return [
                'id' => $progression->getId(),
                'userId' => $user?->getId(),
                'userName' => $user?->getDisplayName(),
                'formationId' => $formation?->getId(),
                'formationTitle' => $formation?->getTitre(),
                'score' => $progression->getScore(),
                'completed' => $progression->isCompleted(),
                'dateDebut' => $progression->getDateDebut()->format(DATE_ATOM),
                'dateEnd' => $progression->getDateEnd()?->format(DATE_ATOM),
            ];
        }, $progressions->findBy([], ['dateDebut' => 'DESC'])));
    }

    #[Route('/progressions', name: 'api_progression_save', methods: ['POST'])]
    public function saveProgression(
        Request $request,
        FormationRepository $formations,
        ProgressionRepository $progressions,
        UtilisateurBadgeRepository $userBadges,
        EntityManagerInterface $em,
    ): JsonResponse {
        $payload = $this->decodeJsonPayload($request);
        if ($payload instanceof JsonResponse) {
            return $payload;
        }

        $formationId = $payload['formationId'] ?? null;
        if (!$this->isPositiveIntegerValue($formationId)) {
            return new JsonResponse(['error' => 'formationId obligatoire', 'code' => 'FORMATION_ID_REQUIRED'], 400);
        }

        $formation = $formations->find((int) $formationId);
        if (!$formation) {
            return new JsonResponse(['error' => 'Formation introuvable', 'code' => 'FORMATION_NOT_FOUND'], 404);
        }

        return $this->saveCurrentUserProgression($payload, $formation, $progressions, $userBadges, $em);
    }

    #[Route('/leaderboard', name: 'api_leaderboard', methods: ['GET'])]
    public function leaderboard(UtilisateurRepository $users): JsonResponse
    {
        $items = $users->findBy([], ['tempsPresenceTotal' => 'DESC'], 20);

        return new JsonResponse(array_map(static fn ($user, $rank) => [
            'rank' => $rank + 1,
            'id' => $user->getId(),
            'username' => $user->getUsername(),
            'displayName' => $user->getDisplayName(),
            'tempsPresenceTotal' => $user->getTempsPresenceTotal(),
            'statut' => $user->getStatut(),
            // No badge UID here. This endpoint is documented as public in /api-docs
            // and the contract is fine — the field was not. A badge UID is the
            // credential the door and machine readers trust, so publishing it beside
            // a real name was a badge-cloning kit. Narrowed rather than gated so that
            // a later permissions mistake cannot re-expose it (S38).
        ], $items, array_keys($items)));
    }

    #[Route('/users', name: 'api_users', methods: ['GET'])]
    #[Route('/utilisateurs', name: 'api_utilisateurs', methods: ['GET'])]
    public function users(UtilisateurRepository $users): JsonResponse
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return new JsonResponse(array_map(fn ($user): array => $this->userToArray($user), $users->findBy([], ['createdAt' => 'DESC'])));
    }

    #[Route('/users/{id}', name: 'api_user_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/utilisateurs/{id}', name: 'api_utilisateur_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function user(
        int $id,
        UtilisateurRepository $users,
        AccessRfidLogRepository $logs,
        ProgressionRepository $progressions,
        ReservationRepository $reservations,
        LogUtilisationRepository $usageLogs,
    ): JsonResponse {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $user = $users->find($id);
        if (!$user) {
            return new JsonResponse(['error' => 'Utilisateur introuvable'], 404);
        }

        return new JsonResponse($this->userToArray($user) + [
            'rfidLogs' => array_map(fn ($log): array => $this->rfidLogToArray($log), $logs->findBy(['utilisateur' => $user], ['createdAt' => 'DESC'])),
            'progressions' => array_map(fn ($progression): array => $this->progressionToArray($progression), $progressions->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC'])),
            'reservations' => array_map(fn ($reservation): array => $this->reservationToArray($reservation), $reservations->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC'])),
            'usageLogs' => array_map(fn ($usageLog): array => $this->usageLogToArray($usageLog), $usageLogs->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC'])),
        ]);
    }

    #[Route('/formations/{id}/suivi', name: 'api_formation_follow', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function formationFollow(
        int $id,
        FormationRepository $formations,
        ProgressionRepository $progressions,
        SectionRepository $sections,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        ChoixRepository $choices,
    ): JsonResponse {
        $formation = $formations->find($id);
        if (!$formation) {
            return new JsonResponse(['error' => 'Formation introuvable'], 404);
        }

        $quizRows = [];
        foreach ($quizzes->findBy(['formation' => $formation], ['id' => 'ASC']) as $quiz) {
            $questionRows = [];
            foreach ($questions->findBy(['quiz' => $quiz], ['ordre' => 'ASC']) as $question) {
                $questionRows[] = [
                    'id' => $question->getId(),
                    'texte' => $question->getTexte(),
                    'type' => $question->getType(),
                    'ordre' => $question->getOrdre(),
                    'choices' => array_map(static fn ($choice): array => [
                        'id' => $choice->getId(),
                        'texte' => $choice->getTexte(),
                        'estCorrect' => $choice->isEstCorrect(),
                        'ordre' => $choice->getOrdre(),
                    ], $choices->findBy(['question' => $question], ['ordre' => 'ASC'])),
                ];
            }

            $quizRows[] = [
                'id' => $quiz->getId(),
                'sectionId' => $quiz->getSection()?->getId(),
                'noteMinimale' => $quiz->getNoteMinimale(),
                'questions' => $questionRows,
            ];
        }

        return new JsonResponse([
            'formation' => $this->formationToArray($formation),
            'progressions' => array_map(fn ($progression): array => $this->progressionToArray($progression), $progressions->findBy(['formation' => $formation], ['dateDebut' => 'DESC'])),
            'sections' => array_map(static fn ($section): array => [
                'id' => $section->getId(),
                'titre' => $section->getTitre(),
                'contenu' => $section->getContenu(),
                'videoUrl' => $section->getVideoUrl(),
                'ordre' => $section->getOrdre(),
                'createdAt' => $section->getCreatedAt()->format(DATE_ATOM),
            ], $sections->findJourneySections($formation)),
            'quizzes' => $quizRows,
        ]);
    }

    #[Route('/formations/{id}/suivi', name: 'api_formation_follow_save', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function saveFormationFollow(
        int $id,
        Request $request,
        FormationRepository $formations,
        ProgressionRepository $progressions,
        UtilisateurBadgeRepository $userBadges,
        EntityManagerInterface $em,
    ): JsonResponse {
        $payload = $this->decodeJsonPayload($request);
        if ($payload instanceof JsonResponse) {
            return $payload;
        }

        $formation = $formations->find($id);
        if (!$formation) {
            return new JsonResponse(['error' => 'Formation introuvable', 'code' => 'FORMATION_NOT_FOUND'], 404);
        }

        return $this->saveCurrentUserProgression($payload, $formation, $progressions, $userBadges, $em);
    }

    #[Route('/badges', name: 'api_badges', methods: ['GET'])]
    public function badges(BadgeRepository $badges, UtilisateurBadgeRepository $userBadges, FormationRepository $formations): JsonResponse
    {
        return new JsonResponse(array_map(static fn ($badge) => [
            'id' => $badge->getId(),
            'nom' => $badge->getNom(),
            'description' => $badge->getDescription(),
            'icone' => $badge->getIcone(),
            'createdAt' => $badge->getCreatedAt()->format(DATE_ATOM),
            'userCount' => $userBadges->count(['badge' => $badge]),
            'formationCount' => $formations->count(['badge' => $badge]),
        ], $badges->findBy([], ['createdAt' => 'DESC'])));
    }

    #[Route('/access-rfid-logs', name: 'api_access_rfid_logs', methods: ['GET'])]
    public function accessRfidLogs(AccessRfidLogRepository $logs): JsonResponse
    {
        // Badge UIDs paired with who used which machine when — a movement history of
        // real people, 100 rows at a time. ⚠️ The JSON key is `badgeUid`, not `rfid`,
        // which is why grepping for the field name reported this endpoint as clean.
        // Undocumented in /api-docs, so nothing public depended on it (S38).
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return new JsonResponse(array_map(static function ($log): array {
            $user = $log->getUtilisateur();
            $machine = $log->getMachine();

            return [
                'id' => $log->getId(),
                'badgeUid' => $log->getBadgeUid(),
                'userId' => $user?->getId(),
                'userFirstName' => $user?->getFirstName(),
                'userName' => $user?->getDisplayName(),
                'machineId' => $machine?->getId(),
                'machineName' => $machine?->getNom(),
                'authorized' => $log->isAuthorized(),
                'status' => $log->getStatus(),
                'reason' => $log->getReason(),
                'message' => $log->getMessage(),
                'color' => $log->getColor(),
                'createdAt' => $log->getCreatedAt()->format(DATE_ATOM),
            ];
        }, $logs->findBy([], ['createdAt' => 'DESC'], 100)));
    }

    #[Route('/search', name: 'api_search', methods: ['GET'])]
    public function search(Request $request, MachineRepository $machines, FormationRepository $formations, UtilisateurRepository $users, BadgeRepository $badges): JsonResponse
    {
        $query = trim((string) $request->query->get('q', ''));
        $needle = mb_strtolower($query);
        $categories = [
            'utilisateurs' => [],
            'machines' => [],
            'formations' => [],
            'badges' => [],
        ];

        if ($needle !== '') {
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
                        $categories['utilisateurs'][] = [
                            'type' => 'utilisateur',
                            'id' => $user->getId(),
                            'title' => $user->getDisplayName(),
                            'description' => $user->getEmail(),
                            'url' => $this->generateUrl('app_admin_user_detail', ['id' => $user->getId()]),
                        ];
                    }
                }
            }

            foreach ($machines->findLive() as $machine) {
                $haystack = mb_strtolower(implode(' ', [
                    $machine->getNom(),
                    $machine->getDescription() ?? '',
                    $machine->getLocalisation() ?? '',
                    $machine->getMachineToken(),
                ]));
                if (str_contains($haystack, $needle)) {
                    $categories['machines'][] = [
                        'type' => 'machine',
                        'id' => $machine->getId(),
                        'title' => $machine->getNom(),
                        'description' => $machine->getDescription(),
                        'url' => $this->generateUrl('app_machine_detail', ['id' => $machine->getId()]),
                    ];
                }
            }

            foreach ($formations->findAll() as $formation) {
                $haystack = mb_strtolower($formation->getTitre() . ' ' . ($formation->getDescription() ?? ''));
                if (str_contains($haystack, $needle)) {
                    $categories['formations'][] = [
                        'type' => 'formation',
                        'id' => $formation->getId(),
                        'title' => $formation->getTitre(),
                        'description' => $formation->getDescription(),
                        'url' => $this->generateUrl('app_formation_detail', ['id' => $formation->getId()]),
                    ];
                }
            }

            foreach ($badges->findAll() as $badge) {
                $haystack = mb_strtolower(implode(' ', [$badge->getNom(), $badge->getDescription() ?? '', $badge->getIcone() ?? '']));
                if (str_contains($haystack, $needle)) {
                    $categories['badges'][] = [
                        'type' => 'badge',
                        'id' => $badge->getId(),
                        'title' => $badge->getNom(),
                        'description' => $badge->getDescription(),
                        'url' => $this->generateUrl('app_admin_badges'),
                    ];
                }
            }
        }

        return new JsonResponse([
            'query' => $query,
            'categories' => $categories,
            'results' => array_merge(...array_values($categories)),
        ]);
    }


    #[Route('/reservations', name: 'api_reservations', methods: ['GET'])]
    public function reservations(ReservationRepository $reservations): JsonResponse
    {
        // Every reservation of every user: real names, what they booked, when, and the
        // free-text `motif`. /api-docs already files this under "Réservations
        // connectées" — it was simply never enforced, so gating it restores the
        // documented contract rather than breaking one. ROLE_ADMIN and not merely
        // "signed in", because the payload is all users, not the caller's own (S38).
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        return new JsonResponse(array_map(fn ($reservation): array => $this->reservationToArray($reservation), $reservations->findBy([], ['dateDebut' => 'ASC'])));
    }

    #[Route('/reservations/{id}', name: 'api_reservation_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function reservation(int $id, ReservationRepository $reservations): JsonResponse
    {
        // Same payload as the list above, one row at a time — and enumerable by id,
        // so leaving it open would have made gating the list cosmetic (S38).
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $reservation = $reservations->find($id);
        if (!$reservation) {
            return new JsonResponse(['error' => 'Réservation introuvable'], 404);
        }

        return new JsonResponse($this->reservationToArray($reservation));
    }

    #[Route('/machines/{id}/reservations', name: 'api_machine_reservations', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function machineReservations(int $id, MachineRepository $machines, ReservationRepository $reservations): JsonResponse
    {
        $machine = $machines->find($id);
        if (!$machine) {
            return new JsonResponse(['error' => 'Machine introuvable'], 404);
        }

        // ⚠️ Narrowed, not gated. Knowing a machine is busy from 14:00 to 16:00 is
        // what a calendar needs; knowing *who* booked it is not. Gating this would
        // have taken occupancy away from any public calendar with it, so this one
        // keeps the slots and drops the identity (S38).
        return new JsonResponse(array_map(fn ($reservation): array => $this->reservationToOccupancyArray($reservation), $reservations->findForReservable(ReservableType::Machine, $machine->getId(), ['dateDebut' => 'ASC'])));
    }

    // ⚠️ This one was not in S38's list and leaks the same badge UIDs as
    // /api/access-rfid-logs, one machine at a time and enumerable by id. Found by
    // fetching every endpoint and reading the payloads, which is the only check that
    // works here — the entity property is `identifiantRfid`, the keys are `rfid` and
    // `badgeUid`, and no single grep covers all three.
    #[Route('/machines/{id}/historique', name: 'api_machine_history', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function machineHistory(int $id, MachineRepository $machines, AccessRfidLogRepository $rfidLogs, LogUtilisationRepository $usageLogs, ReservationRepository $reservations): JsonResponse
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $machine = $machines->find($id);
        if (!$machine) {
            return new JsonResponse(['error' => 'Machine introuvable'], 404);
        }

        return new JsonResponse([
            'machine' => $this->machineToArray($machine),
            'rfidLogs' => array_map(fn ($log): array => $this->rfidLogToArray($log), $rfidLogs->findBy(['machine' => $machine], ['createdAt' => 'DESC'])),
            'usageLogs' => array_map(fn ($usageLog): array => $this->usageLogToArray($usageLog), $usageLogs->findBy(['machine' => $machine], ['dateDebut' => 'DESC'])),
            'reservations' => array_map(fn ($reservation): array => $this->reservationToArray($reservation), $reservations->findForReservable(ReservableType::Machine, $machine->getId())),
        ]);
    }

    #[Route('/reservations', name: 'api_reservation_create', methods: ['POST'])]
    public function createReservation(Request $request, ReservationService $booking, SiteSettingService $siteSettings): JsonResponse
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            return new JsonResponse(['error' => 'Authentification requise', 'code' => 'AUTH_REQUIRED'], 401);
        }

        $payload = json_decode($request->getContent(), true);
        if (!is_array($payload)) {
            return new JsonResponse(['error' => 'JSON invalide', 'code' => 'INVALID_JSON'], 400);
        }

        // Polymorphic payload, falling back to the machine-only shape older
        // clients still send.
        $type = ReservableType::tryParse(is_string($payload['reservableType'] ?? null) ? $payload['reservableType'] : null);
        $id = $payload['reservableId'] ?? null;
        if ($type === null) {
            $type = ReservableType::Machine;
            $id = $payload['machineId'] ?? null;
        }

        if (!$this->isPositiveIntegerValue($id)) {
            return new JsonResponse([
                'error' => $type === ReservableType::Machine ? 'machineId obligatoire' : 'reservableId obligatoire',
                'code' => $type === ReservableType::Machine ? 'MACHINE_ID_REQUIRED' : 'RESERVABLE_ID_REQUIRED',
            ], 400);
        }

        $dateDebutInput = $payload['dateDebut'] ?? $payload['startAt'] ?? null;
        if (!is_string($dateDebutInput) || trim($dateDebutInput) === '') {
            return new JsonResponse(['error' => 'dateDebut obligatoire', 'code' => 'DATE_DEBUT_REQUIRED'], 400);
        }

        $dateFinInput = $payload['dateFin'] ?? $payload['endAt'] ?? null;
        if (!is_string($dateFinInput) || trim($dateFinInput) === '') {
            return new JsonResponse(['error' => 'dateFin obligatoire', 'code' => 'DATE_FIN_REQUIRED'], 400);
        }

        try {
            $dateDebut = $this->parseReservationDate($dateDebutInput, $siteSettings);
            $dateFin = $this->parseReservationDate($dateFinInput, $siteSettings);
        } catch (\Throwable) {
            return new JsonResponse(['error' => 'Dates invalides', 'code' => 'INVALID_DATES'], 400);
        }

        $motif = $payload['motif'] ?? $payload['comment'] ?? null;
        if ($motif !== null && !is_string($motif)) {
            return new JsonResponse(['error' => 'motif invalide', 'code' => 'INVALID_MOTIF'], 400);
        }

        $result = $booking->book($type, (int) $id, $user, $dateDebut, $dateFin, $motif);
        if (!$result->ok) {
            return new JsonResponse(
                ['error' => $result->message, 'code' => $result->code] + $result->context,
                $result->status,
            );
        }

        return new JsonResponse([
            'status' => 'created',
            'reservation' => $this->reservationToArray($result->reservation),
        ], 201);
    }

    /*
     * The four verbs on an existing booking (S77). They are four routes rather
     * than one PATCH because they are four different acts with four different
     * permission answers — "I'm done early" is always allowed, "move me to
     * Thursday" is a fresh booking in disguise — and a single endpoint switching
     * on a body field would hide that behind a shape.
     *
     * ⚠️ None of them decides anything. Each parses its payload and hands off to
     * ReservationService, which asks BookingVerbService. The rule lives in one
     * place so the page drawing the button and the endpoint honouring it cannot
     * drift apart — which is exactly what had happened to cancel.
     */

    #[Route('/reservations/{id}/cancel', name: 'api_reservation_cancel', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function cancelReservation(int $id, Request $request, ReservationRepository $reservations, ReservationService $booking): JsonResponse|\Symfony\Component\HttpFoundation\RedirectResponse
    {
        return $this->runReservationVerb(
            $request,
            $reservations,
            $id,
            BookingVerb::Cancel,
            fn (Reservation $reservation, Utilisateur $user): BookingResult => $booking->cancel($reservation, $user),
            // ⚠️ The undo offer rides on the redirect, so it survives the full
            // page reload a no-JS form post causes. A toast would not.
            undoable: true,
        );
    }

    /**
     * The staff cancel — the same verb, from a surface that is allowed to correct
     * the record.
     *
     * ⚠️ **A separate route rather than a role check inside the member one**, and
     * that is the whole point. An admin on `/mes-reservations` is a member
     * looking at their own bookings and must see what any member sees; an admin
     * on `/admin/reservations` is doing records management. Deciding by role
     * alone put a "cancel a booking from last March" button on a personal page,
     * which is what an operator noticed and asked about.
     *
     * ⚠️ `IsGranted` here is the real gate — `VerbContext::Staff` is a statement
     * about the surface, not a permission, and is unreachable except through
     * this route.
     *
     * ⚠️ **Unaudited.** It confines a power that was previously ambient rather
     * than adding one, but attributing it to a named person is S63, and staff
     * acting on behalf of a member properly belongs to S62.
     */
    #[Route('/staff/reservations/{id}/cancel', name: 'api_staff_reservation_cancel', requirements: ['id' => '\\d+'], methods: ['POST'])]
    #[IsGranted('ROLE_ADMIN')]
    public function staffCancelReservation(int $id, Request $request, ReservationRepository $reservations, ReservationService $booking): JsonResponse|\Symfony\Component\HttpFoundation\RedirectResponse
    {
        return $this->runReservationVerb(
            $request,
            $reservations,
            $id,
            BookingVerb::Cancel,
            fn (Reservation $reservation, Utilisateur $user): BookingResult => $booking->cancel($reservation, $user, null, VerbContext::Staff),
        );
    }

    #[Route('/reservations/{id}/restore', name: 'api_reservation_restore', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function restoreReservation(int $id, Request $request, ReservationRepository $reservations, ReservationService $booking): JsonResponse|\Symfony\Component\HttpFoundation\RedirectResponse
    {
        return $this->runReservationVerb(
            $request,
            $reservations,
            $id,
            BookingVerb::Restore,
            fn (Reservation $reservation, Utilisateur $user): BookingResult => $booking->restore($reservation, $user),
            successMessage: 'Réservation rétablie.',
        );
    }

    #[Route('/reservations/{id}/end-now', name: 'api_reservation_end_now', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function endReservationNow(int $id, Request $request, ReservationRepository $reservations, ReservationService $booking): JsonResponse|\Symfony\Component\HttpFoundation\RedirectResponse
    {
        return $this->runReservationVerb(
            $request,
            $reservations,
            $id,
            BookingVerb::EndNow,
            fn (Reservation $reservation, Utilisateur $user): BookingResult => $booking->endNow($reservation, $user),
            successMessage: 'Réservation terminée. Le créneau restant est libéré.',
        );
    }

    #[Route('/reservations/{id}/reschedule', name: 'api_reservation_reschedule', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function rescheduleReservation(int $id, Request $request, ReservationRepository $reservations, ReservationService $booking, SiteSettingService $siteSettings): JsonResponse|\Symfony\Component\HttpFoundation\RedirectResponse
    {
        // Both entry shapes, like every other booking route: the form posts
        // fields, the API posts JSON.
        $payload = $request->request->has('_token')
            ? $request->request->all()
            : (json_decode($request->getContent(), true) ?: []);

        if (!is_array($payload)) {
            return $this->reservationVerbResponse($request, ['error' => 'JSON invalide', 'code' => 'INVALID_JSON'], 400);
        }

        $startInput = $payload['dateDebut'] ?? $payload['startAt'] ?? null;
        $endInput = $payload['dateFin'] ?? $payload['endAt'] ?? null;

        if (!is_string($startInput) || trim($startInput) === '' || !is_string($endInput) || trim($endInput) === '') {
            return $this->reservationVerbResponse($request, ['error' => 'Nouvelles dates obligatoires', 'code' => 'DATES_REQUIRED'], 400);
        }

        try {
            $start = $this->parseReservationDate($startInput, $siteSettings);
            $end = $this->parseReservationDate($endInput, $siteSettings);
        } catch (\Throwable) {
            return $this->reservationVerbResponse($request, ['error' => 'Dates invalides', 'code' => 'INVALID_DATES'], 400);
        }

        $motif = $payload['motif'] ?? $payload['comment'] ?? null;
        if ($motif !== null && !is_string($motif)) {
            return $this->reservationVerbResponse($request, ['error' => 'motif invalide', 'code' => 'INVALID_MOTIF'], 400);
        }

        return $this->runReservationVerb(
            $request,
            $reservations,
            $id,
            BookingVerb::Reschedule,
            fn (Reservation $reservation, Utilisateur $user): BookingResult => $booking->reschedule($reservation, $user, $start, $end, $motif),
            successMessage: 'Réservation déplacée.',
        );
    }

    /**
     * Load, check CSRF, run the verb, answer in whichever dialect asked.
     *
     * @param callable(Reservation, Utilisateur): BookingResult $run
     */
    private function runReservationVerb(
        Request $request,
        ReservationRepository $reservations,
        int $id,
        BookingVerb $verb,
        callable $run,
        string $successMessage = 'Réservation annulée.',
        bool $undoable = false,
    ): JsonResponse|\Symfony\Component\HttpFoundation\RedirectResponse {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            return $this->reservationVerbResponse($request, ['error' => 'Authentification requise', 'code' => 'AUTH_REQUIRED'], 401);
        }

        $reservation = $reservations->find($id);
        if (!$reservation) {
            return $this->reservationVerbResponse($request, ['error' => 'Réservation introuvable', 'code' => 'RESERVATION_NOT_FOUND'], 404);
        }

        // ⚠️ Only form posts carry a token, and only form posts are checked —
        // the JSON API is stateless and authenticates otherwise. That asymmetry
        // is pre-existing; it is preserved here rather than quietly changed,
        // because tightening it would break every JSON client in the same deploy
        // that adds three new verbs.
        if ($request->request->has('_token')
            && !$this->isCsrfTokenValid($verb->csrfToken((int) $reservation->getId()), (string) $request->request->get('_token'))) {
            return $this->reservationVerbResponse($request, ['error' => 'Token CSRF invalide', 'code' => 'INVALID_CSRF_TOKEN'], 400);
        }

        $result = $run($reservation, $user);
        if (!$result->ok) {
            return $this->reservationVerbResponse(
                $request,
                ['error' => $result->message, 'code' => $result->code] + $result->context,
                $result->status,
            );
        }

        return $this->reservationVerbResponse($request, [
            'status' => 'ok',
            'message' => $successMessage,
            'reservation' => $this->reservationToArray($reservation),
        ], 200, $undoable ? (int) $reservation->getId() : null);
    }


    private function isPositiveIntegerValue(mixed $value): bool
    {
        if (is_int($value)) {
            return $value > 0;
        }

        return is_string($value) && preg_match('/^[1-9]\d*$/', $value) === 1;
    }

    /**
     * @return array<string, mixed>|JsonResponse
     */
    private function decodeJsonPayload(Request $request): array|JsonResponse
    {
        $payload = json_decode($request->getContent(), true);
        if (!is_array($payload)) {
            return new JsonResponse(['error' => 'JSON invalide', 'code' => 'INVALID_JSON'], 400);
        }

        return $payload;
    }

    /**
     * @param array<string, mixed> $payload
     */
    private function saveCurrentUserProgression(
        array $payload,
        $formation,
        ProgressionRepository $progressions,
        UtilisateurBadgeRepository $userBadges,
        EntityManagerInterface $em,
    ): JsonResponse {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            return new JsonResponse(['error' => 'Authentification requise', 'code' => 'AUTH_REQUIRED'], 401);
        }

        $completed = $payload['completed'] ?? false;
        if (!is_bool($completed)) {
            return new JsonResponse(['error' => 'completed doit être un booléen', 'code' => 'INVALID_COMPLETED'], 400);
        }

        $score = $payload['score'] ?? 0;
        if (!is_int($score) && !(is_string($score) && preg_match('/^\d+$/', $score) === 1)) {
            return new JsonResponse(['error' => 'score invalide', 'code' => 'INVALID_SCORE'], 400);
        }

        $score = max(0, min(100, (int) $score));
        $progression = $progressions->findOneBy([
            'utilisateur' => $user,
            'formation' => $formation,
        ]);
        $created = false;

        if (!$progression instanceof Progression) {
            $progression = (new Progression())
                ->setUtilisateur($user)
                ->setFormation($formation);
            $em->persist($progression);
            $created = true;
        }

        $badge = $formation->getBadge();
        $hadBadge = $badge !== null && $userBadges->findOneBy([
            'utilisateur' => $user,
            'badge' => $badge,
        ]) !== null;

        $dateEnd = null;
        if ($completed) {
            $now = new \DateTimeImmutable();
            $minimumDateEnd = $progression->getDateDebut()->modify('+1 second');
            $dateEnd = $now > $minimumDateEnd ? $now : $minimumDateEnd;
        }

        $progression
            ->setScore($score)
            ->setCompleted($completed)
            ->setDateEnd($dateEnd);

        $em->flush();

        $hasBadge = $badge !== null && $userBadges->findOneBy([
            'utilisateur' => $user,
            'badge' => $badge,
        ]) !== null;

        return new JsonResponse([
            'status' => $created ? 'created' : 'updated',
            'progression' => $this->progressionToArray($progression),
            'badgeAwarded' => $completed && !$hadBadge && $hasBadge,
            'badgeAlreadyOwned' => $completed && $hadBadge,
            'badge' => $badge ? [
                'id' => $badge->getId(),
                'nom' => $badge->getNom(),
                'description' => $badge->getDescription(),
                'icone' => $badge->getIcone(),
            ] : null,
        ], $created ? 201 : 200);
    }

    /**
     * One answer, two dialects: JSON for the API, flash-and-redirect for a form
     * post. The token's presence is what tells them apart — a browser form
     * always carries one, a JSON client never does.
     *
     * $undoId turns the redirect into an offer rather than a notice. It is a
     * query parameter and not a flash because the undo has to survive the member
     * reloading, sorting or filtering the page they land on: a flash is consumed
     * by the first render, and an undo that vanishes when you blink is worse
     * than no undo, because you have already relaxed about the mistake.
     *
     * @param array<string, mixed> $payload
     */
    private function reservationVerbResponse(Request $request, array $payload, int $statusCode = 200, ?int $undoId = null): JsonResponse|\Symfony\Component\HttpFoundation\RedirectResponse
    {
        if (!$request->request->has('_token')) {
            return new JsonResponse($payload, $statusCode);
        }

        $this->addFlash(
            $statusCode >= 400 ? 'error' : 'success',
            $payload['error'] ?? $payload['message'] ?? 'Réservation annulée.',
        );

        // ⚠️ Reduced to path + query before being redirected to. `Referer` is a
        // client-supplied header, so handing it to redirect() whole is an open
        // redirect — pre-existing here, and closed on the way past rather than
        // left in place next to three new routes that would inherit it.
        $parts = parse_url((string) $request->headers->get('referer'));
        $path = is_array($parts) && ($parts['path'] ?? '') !== '' ? $parts['path'] : null;

        $query = [];
        parse_str(is_array($parts) ? ($parts['query'] ?? '') : '', $query);

        if ($undoId !== null && $statusCode < 400) {
            // ⚠️ Merged into the existing query, not assigned over it: the member
            // may have arrived from a filtered or searched list
            // (`?etat=upcoming&q=laser`), and dropping those would answer their
            // cancellation by silently resetting their view.
            $query['undo'] = $undoId;
        }

        $target = $path ?? $this->generateUrl('app_my_reservations');

        return $this->redirect($query === [] ? $target : $target . '?' . http_build_query($query));
    }

    private function parseReservationDate(string $value, SiteSettingService $siteSettings): \DateTimeImmutable
    {
        $value = trim($value);
        if (!preg_match('/^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}(?::\d{2})?(?:Z|[+-]\d{2}:?\d{2})?$/', $value)) {
            throw new \InvalidArgumentException('Format de date invalide');
        }

        return new \DateTimeImmutable($value, $this->labZone($siteSettings));
    }

    private function userToArray($user): array
    {
        return [
            'id' => $user->getId(),
            'email' => $user->getEmail(),
            'username' => $user->getUsername(),
            'firstName' => $user->getFirstName(),
            'lastName' => $user->getLastName(),
            'numeroId' => $user->getNumeroId(),
            'statut' => $user->getStatut(),
            'identifiantRfid' => $user->getIdentifiantRfid(),
            'tempsPresenceTotal' => $user->getTempsPresenceTotal(),
            'isVerified' => $user->isVerified(),
            'createdAt' => $user->getCreatedAt()->format(DATE_ATOM),
            'derniereConnexion' => $user->getDerniereConnexion()?->format(DATE_ATOM),
            'notificationEmail' => $user->isNotificationEmail(),
            'notificationPush' => $user->isNotificationPush(),
            'rappelReservation' => $user->isRappelReservation(),
            'theme' => $user->getTheme(),
            'langue' => $user->getLangue(),
        ];
    }

    private function rfidLogToArray($log): array
    {
        $machine = $log->getMachine();
        $user = $log->getUtilisateur();

        return [
            'id' => $log->getId(),
            'badgeUid' => $log->getBadgeUid(),
            'userId' => $user?->getId(),
            'userName' => $user?->getDisplayName(),
            'machineId' => $machine?->getId(),
            'machineName' => $machine?->getNom(),
            'authorized' => $log->isAuthorized(),
            'status' => $log->getStatus(),
            'reason' => $log->getReason(),
            'message' => $log->getMessage(),
            'color' => $log->getColor(),
            'createdAt' => $log->getCreatedAt()->format(DATE_ATOM),
        ];
    }

    private function formationToArray($formation): array
    {
        $badge = $formation->getBadge();

        return [
            'id' => $formation->getId(),
            'titre' => $formation->getTitre(),
            'description' => $formation->getDescription(),
            'image' => $formation->getImage(),
            'categorie' => $formation->getCategorie(),
            'niveau' => $formation->getNiveau(),
            'duree' => $formation->getDuree(),
            'formateur' => $formation->getFormateur(),
            'placesTotales' => $formation->getPlacesTotales(),
            'objectifs' => $formation->getObjectifs(),
            'prerequis' => $formation->getPrerequis(),
            'materielFourni' => $formation->getMaterielFourni(),
            'badge' => $badge ? [
                'id' => $badge->getId(),
                'nom' => $badge->getNom(),
                'icone' => $badge->getIcone(),
            ] : null,
        ];
    }

    private function progressionToArray($progression): array
    {
        $formation = $progression->getFormation();
        $user = $progression->getUtilisateur();

        return [
            'id' => $progression->getId(),
            'userId' => $user?->getId(),
            'userName' => $user?->getDisplayName(),
            'formationId' => $formation?->getId(),
            'formationTitle' => $formation?->getTitre(),
            'score' => $progression->getScore(),
            'completed' => $progression->isCompleted(),
            'dateDebut' => $progression->getDateDebut()->format(DATE_ATOM),
            'dateEnd' => $progression->getDateEnd()?->format(DATE_ATOM),
        ];
    }

    private function reservationToArray($reservation): array
    {
        $user = $reservation->getUtilisateur();
        $resource = $this->reservables->resolve($reservation);
        $isMachine = $resource->type === ReservableType::Machine;

        return [
            'id' => $reservation->getId(),
            'reservableType' => $resource->type?->value,
            'reservableId' => $resource->id,
            'reservableName' => $resource->name,
            // Kept for existing API consumers; null for anything but a machine.
            'machineId' => $isMachine ? $resource->id : null,
            'machineName' => $isMachine ? $resource->name : null,
            'userId' => $user?->getId(),
            'userName' => $user?->getDisplayName(),
            'dateDebut' => $reservation->getDateDebut()->format(DATE_ATOM),
            'dateFin' => $reservation->getDateFin()->format(DATE_ATOM),
            'motif' => $reservation->getMotif(),
            'statut' => $reservation->getStatut(),
            'status' => $reservation->getStatut(),
            'cancelled' => $reservation->isCancelled(),
            'pending' => $reservation->isPending(),
            'declined' => $reservation->isDeclined(),
            'created' => $reservation->getCreated()->format(DATE_ATOM),
        ];
    }

    /**
     * A reservation as an anonymous caller may see it: when the resource is taken,
     * never by whom.
     *
     * ⚠️ Deliberately a second serialiser rather than a flag on reservationToArray().
     * That one is shared with the ROLE_ADMIN endpoints, where the name and the motif
     * are exactly what the caller is entitled to — narrowing it in place would have
     * silently emptied the admin views. And a `$includeIdentity = false` parameter
     * puts the safe outcome one forgotten argument away; a separate function makes
     * leaking identity require writing the wrong function name (S38).
     */
    private function reservationToOccupancyArray($reservation): array
    {
        $resource = $this->reservables->resolve($reservation);
        $isMachine = $resource->type === ReservableType::Machine;

        return [
            'id' => $reservation->getId(),
            'reservableType' => $resource->type?->value,
            'reservableId' => $resource->id,
            'reservableName' => $resource->name,
            'machineId' => $isMachine ? $resource->id : null,
            'machineName' => $isMachine ? $resource->name : null,
            'dateDebut' => $reservation->getDateDebut()->format(DATE_ATOM),
            'dateFin' => $reservation->getDateFin()->format(DATE_ATOM),
            'statut' => $reservation->getStatut(),
            'status' => $reservation->getStatut(),
            'cancelled' => $reservation->isCancelled(),
            'pending' => $reservation->isPending(),
            'declined' => $reservation->isDeclined(),
            // No userId, no userName, no motif — the motif is free text somebody typed
            // about their own project, and it has no business on an open endpoint.
        ];
    }

    private function usageLogToArray($usageLog): array
    {
        $machine = $usageLog->getMachine();
        $user = $usageLog->getUtilisateur();

        return [
            'id' => $usageLog->getId(),
            'machineId' => $machine?->getId(),
            'machineName' => $machine?->getNom(),
            'userId' => $user?->getId(),
            'userName' => $user?->getDisplayName(),
            'dateDebut' => $usageLog->getDateDebut()->format(DATE_ATOM),
            'dateFin' => $usageLog->getDateFin()?->format(DATE_ATOM),
            'duree' => $usageLog->getDuree(),
            'source' => $usageLog->getSource(),
        ];
    }

    private function machineToArray(
        $machine,
        ?Utilisateur $currentUser = null,
        ?MachineQualificationService $machineAccess = null,
    ): array
    {
        $requiredBadges = [];
        $authorizationStatus = null;

        if ($currentUser instanceof Utilisateur && $machineAccess !== null) {
            $accessStatus = $machineAccess->getStatus($machine, $currentUser);
            foreach ($accessStatus['badgeRows'] as $row) {
                $badge = $row['badge'];
                $requiredBadges[] = [
                    'id' => $badge->getId(),
                    'nom' => $badge->getNom(),
                    'description' => $badge->getDescription(),
                    'icone' => $badge->getIcone(),
                    'requiredForAccess' => $row['machineBadge']->isRequiredForAccess(),
                    'owned' => $row['owned'],
                ];
            }
            $authorizationStatus = $accessStatus['authorizationStatus'];
        } else {
            foreach ($machine->getRequiredMachineBadges() as $machineBadge) {
                $badge = $machineBadge->getBadge();
                if ($badge === null) {
                    continue;
                }
                $requiredBadges[] = [
                    'id' => $badge->getId(),
                    'nom' => $badge->getNom(),
                    'description' => $badge->getDescription(),
                    'icone' => $badge->getIcone(),
                    'requiredForAccess' => $machineBadge->isRequiredForAccess(),
                    'owned' => null,
                ];
            }
        }

        return [
            'id' => $machine->getId(),
            'nom' => $machine->getNom(),
            'description' => $machine->getDescription(),
            'localisation' => $machine->getLocalisation(),
            'photo' => $machine->getPhoto(),
            'statut' => $machine->getStatut(),
            'granularite' => $machine->getGranulariteMinutes(),
            'granulariteLabel' => $machine->getGranulariteLabel(),
            'categorySlug' => $machine->getCategorySlug(),
            'categoryLabel' => $machine->getCategoryLabel(),
            'levelSlug' => $machine->getLevelSlug(),
            'levelLabel' => $machine->getLevelLabel(),
            'iconSlug' => $machine->getIconSlug(),
            'materials' => $machine->getMaterials(),
            'features' => $machine->getFeatures(),
            'requirementTitle' => $machine->getRequirementTitle(),
            'requirementDescription' => $machine->getRequirementDescription(),
            'popularity' => $machine->getPopularity(),
            'limiteReservations' => $machine->getLimiteReservations(),
            'machineToken' => $machine->getMachineToken(),
            'createdAt' => $machine->getCreatedAt()->format(DATE_ATOM),
            'updated' => $machine->getUpdated()->format(DATE_ATOM),
            'lastAuthorizationTime' => $machine->getLastAuthorizationTime()?->format(DATE_ATOM),
            'badgesRequis' => $requiredBadges,
            'authorizationStatus' => $authorizationStatus,
        ];
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
