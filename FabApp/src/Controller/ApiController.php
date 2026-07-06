<?php

namespace App\Controller;

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
use App\Repository\SectionRepository;
use App\Repository\QuizRepository;
use App\Repository\QuestionRepository;
use App\Repository\ChoixRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Repository\UtilisateurRepository;
use App\Service\OpeningHoursProvider;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\JsonResponse;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\Routing\Attribute\Route;

#[Route('/api')]
final class ApiController extends AbstractController
{
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
    public function openingHours(OpeningHoursProvider $openingHours): JsonResponse
    {
        return new JsonResponse($openingHours->getOpeningHoursForJson());
    }

    #[Route('/machines', name: 'api_machines', methods: ['GET'])]
    public function machines(MachineRepository $machines, UtilisateurBadgeRepository $userBadges): JsonResponse
    {
        return new JsonResponse(array_map(fn ($machine) => $this->machineToArray($machine, $this->getUser(), $userBadges), $machines->findAll()));
    }

    #[Route('/machines/{id}', name: 'api_machine_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function machine(int $id, MachineRepository $machines, UtilisateurBadgeRepository $userBadges): JsonResponse
    {
        $machine = $machines->find($id);
        if (!$machine) {
            return new JsonResponse(['error' => 'Machine introuvable'], 404);
        }

        return new JsonResponse($this->machineToArray($machine, $this->getUser(), $userBadges));
    }

    #[Route('/calendar', name: 'api_calendar', methods: ['GET'])]
    public function calendar(ReservationRepository $reservations): JsonResponse
    {
        return new JsonResponse(array_map(function (Reservation $reservation): array {
            $user = $reservation->getUtilisateur();
            $machine = $reservation->getMachine();

            return [
                'id' => $reservation->getId(),
                'machineId' => $machine?->getId(),
                'machineName' => $machine?->getNom(),
                'userId' => $user?->getId(),
                'userName' => $user?->getDisplayName(),
                'dateDebut' => $reservation->getDateDebut()->format(DATE_ATOM),
                'dateFin' => $reservation->getDateFin()->format(DATE_ATOM),
                'motif' => $reservation->getMotif(),
                'created' => $reservation->getCreated()->format(DATE_ATOM),
            ];
        }, $reservations->findAllActive(['dateDebut' => 'ASC'])));
    }

    #[Route('/formations', name: 'api_formations', methods: ['GET'])]
    public function formations(FormationRepository $formations): JsonResponse
    {
        return new JsonResponse(array_map(fn ($formation): array => $this->formationToArray($formation), $formations->findAll()));
    }

    #[Route('/progressions', name: 'api_progressions', methods: ['GET'])]
    public function progressions(ProgressionRepository $progressions): JsonResponse
    {
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
            'rfid' => $user->getIdentifiantRfid(),
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
            ], $sections->findBy(['formation' => $formation], ['ordre' => 'ASC'])),
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

            foreach ($machines->findAll() as $machine) {
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
        return new JsonResponse(array_map(fn ($reservation): array => $this->reservationToArray($reservation), $reservations->findBy([], ['dateDebut' => 'ASC'])));
    }

    #[Route('/reservations/{id}', name: 'api_reservation_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function reservation(int $id, ReservationRepository $reservations): JsonResponse
    {
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

        return new JsonResponse(array_map(fn ($reservation): array => $this->reservationToArray($reservation), $reservations->findBy(['machine' => $machine], ['dateDebut' => 'ASC'])));
    }

    #[Route('/machines/{id}/historique', name: 'api_machine_history', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function machineHistory(int $id, MachineRepository $machines, AccessRfidLogRepository $rfidLogs, LogUtilisationRepository $usageLogs, ReservationRepository $reservations): JsonResponse
    {
        $machine = $machines->find($id);
        if (!$machine) {
            return new JsonResponse(['error' => 'Machine introuvable'], 404);
        }

        return new JsonResponse([
            'machine' => $this->machineToArray($machine),
            'rfidLogs' => array_map(fn ($log): array => $this->rfidLogToArray($log), $rfidLogs->findBy(['machine' => $machine], ['createdAt' => 'DESC'])),
            'usageLogs' => array_map(fn ($usageLog): array => $this->usageLogToArray($usageLog), $usageLogs->findBy(['machine' => $machine], ['dateDebut' => 'DESC'])),
            'reservations' => array_map(fn ($reservation): array => $this->reservationToArray($reservation), $reservations->findBy(['machine' => $machine], ['dateDebut' => 'DESC'])),
        ]);
    }

    #[Route('/reservations', name: 'api_reservation_create', methods: ['POST'])]
    public function createReservation(Request $request, MachineRepository $machines, ReservationRepository $reservations, EntityManagerInterface $em, OpeningHoursProvider $openingHours): JsonResponse
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            return new JsonResponse(['error' => 'Authentification requise', 'code' => 'AUTH_REQUIRED'], 401);
        }

        $payload = json_decode($request->getContent(), true);
        if (!is_array($payload)) {
            return new JsonResponse(['error' => 'JSON invalide', 'code' => 'INVALID_JSON'], 400);
        }

        $machineId = $payload['machineId'] ?? null;
        if (!$this->isPositiveIntegerValue($machineId)) {
            return new JsonResponse(['error' => 'machineId obligatoire', 'code' => 'MACHINE_ID_REQUIRED'], 400);
        }

        $dateDebutInput = $payload['dateDebut'] ?? $payload['startAt'] ?? null;
        if (!is_string($dateDebutInput) || trim($dateDebutInput) === '') {
            return new JsonResponse(['error' => 'dateDebut obligatoire', 'code' => 'DATE_DEBUT_REQUIRED'], 400);
        }

        $dateFinInput = $payload['dateFin'] ?? $payload['endAt'] ?? null;
        if (!is_string($dateFinInput) || trim($dateFinInput) === '') {
            return new JsonResponse(['error' => 'dateFin obligatoire', 'code' => 'DATE_FIN_REQUIRED'], 400);
        }

        $machine = $machines->find((int) $machineId);
        if (!$machine) {
            return new JsonResponse(['error' => 'Machine introuvable', 'code' => 'MACHINE_NOT_FOUND'], 404);
        }

        try {
            $dateDebut = $this->parseReservationDate($dateDebutInput);
            $dateFin = $this->parseReservationDate($dateFinInput);
        } catch (\Throwable) {
            return new JsonResponse(['error' => 'Dates invalides', 'code' => 'INVALID_DATES'], 400);
        }

        $now = new \DateTimeImmutable('now', new \DateTimeZone('Europe/Paris'));
        if ($dateDebut <= $now) {
            return new JsonResponse(['error' => 'dateDebut doit être dans le futur', 'code' => 'DATE_DEBUT_PAST'], 400);
        }

        if ($dateFin <= $dateDebut) {
            return new JsonResponse(['error' => 'dateFin doit être après dateDebut', 'code' => 'DATE_FIN_BEFORE_START'], 400);
        }

        $openingHoursError = $openingHours->validateReservationPeriod($dateDebut, $dateFin);
        if ($openingHoursError !== null) {
            return new JsonResponse(['error' => $openingHoursError, 'code' => 'FABLAB_CLOSED'], 400);
        }

        $motif = $payload['motif'] ?? $payload['comment'] ?? null;
        if ($motif !== null && !is_string($motif)) {
            return new JsonResponse(['error' => 'motif invalide', 'code' => 'INVALID_MOTIF'], 400);
        }

        $motif = is_string($motif) ? trim($motif) : null;
        if ($motif === '') {
            $motif = null;
        }
        if ($motif !== null && mb_strlen($motif) > 500) {
            return new JsonResponse(['error' => 'motif trop long (500 caractères maximum)', 'code' => 'MOTIF_TOO_LONG'], 400);
        }

        if ($reservations->hasOverlap($machine, $dateDebut, $dateFin)) {
            return new JsonResponse(['error' => 'Créneau déjà réservé pour cette machine', 'code' => 'RESERVATION_OVERLAP'], 409);
        }

        $reservation = (new Reservation())
            ->setMachine($machine)
            ->setUtilisateur($user)
            ->setDateDebut($dateDebut)
            ->setDateFin($dateFin)
            ->setMotif($motif)
            ->setStatut('confirmed');

        $em->persist($reservation);
        $em->flush();

        return new JsonResponse([
            'status' => 'created',
            'reservation' => $this->reservationToArray($reservation),
        ], 201);
    }

    #[Route('/reservations/{id}/cancel', name: 'api_reservation_cancel', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function cancelReservation(int $id, Request $request, ReservationRepository $reservations, EntityManagerInterface $em): JsonResponse|\Symfony\Component\HttpFoundation\RedirectResponse
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            return $this->reservationCancelResponse($request, ['error' => 'Authentification requise', 'code' => 'AUTH_REQUIRED'], 401);
        }

        $reservation = $reservations->find($id);
        if (!$reservation) {
            return $this->reservationCancelResponse($request, ['error' => 'Réservation introuvable', 'code' => 'RESERVATION_NOT_FOUND'], 404);
        }

        if ($request->request->has('_token') && !$this->isCsrfTokenValid('cancel_reservation_' . $reservation->getId(), (string) $request->request->get('_token'))) {
            return $this->reservationCancelResponse($request, ['error' => 'Token CSRF invalide', 'code' => 'INVALID_CSRF_TOKEN'], 400);
        }

        $isAdmin = $this->isGranted('ROLE_ADMIN');
        if (!$isAdmin && $reservation->getUtilisateur()?->getId() !== $user->getId()) {
            return $this->reservationCancelResponse($request, ['error' => 'Accès refusé', 'code' => 'RESERVATION_FORBIDDEN'], 403);
        }

        if ($reservation->isCancelled()) {
            return $this->reservationCancelResponse($request, ['error' => 'Réservation déjà annulée', 'code' => 'RESERVATION_ALREADY_CANCELLED'], 409);
        }

        $now = new \DateTimeImmutable('now', new \DateTimeZone('Europe/Paris'));
        if (!$isAdmin && $reservation->getDateDebut() <= $now) {
            return $this->reservationCancelResponse($request, ['error' => 'Une réservation passée ne peut pas être annulée', 'code' => 'RESERVATION_NOT_FUTURE'], 409);
        }

        $reservation->cancel();
        $em->flush();

        return $this->reservationCancelResponse($request, [
            'status' => 'cancelled',
            'reservation' => $this->reservationToArray($reservation),
        ]);
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

    private function reservationCancelResponse(Request $request, array $payload, int $statusCode = 200): JsonResponse|\Symfony\Component\HttpFoundation\RedirectResponse
    {
        if (!$request->request->has('_token')) {
            return new JsonResponse($payload, $statusCode);
        }

        $this->addFlash($statusCode >= 400 ? 'error' : 'success', $payload['error'] ?? 'Réservation annulée.');

        return $this->redirect($request->headers->get('referer') ?: $this->generateUrl('app_profile'));
    }

    private function parseReservationDate(string $value): \DateTimeImmutable
    {
        $value = trim($value);
        if (!preg_match('/^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}(?::\d{2})?(?:Z|[+-]\d{2}:?\d{2})?$/', $value)) {
            throw new \InvalidArgumentException('Format de date invalide');
        }

        return new \DateTimeImmutable($value, new \DateTimeZone('Europe/Paris'));
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
        $machine = $reservation->getMachine();
        $user = $reservation->getUtilisateur();

        return [
            'id' => $reservation->getId(),
            'machineId' => $machine?->getId(),
            'machineName' => $machine?->getNom(),
            'userId' => $user?->getId(),
            'userName' => $user?->getDisplayName(),
            'dateDebut' => $reservation->getDateDebut()->format(DATE_ATOM),
            'dateFin' => $reservation->getDateFin()->format(DATE_ATOM),
            'motif' => $reservation->getMotif(),
            'statut' => $reservation->getStatut(),
            'status' => $reservation->getStatut(),
            'cancelled' => $reservation->isCancelled(),
            'created' => $reservation->getCreated()->format(DATE_ATOM),
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

    private function machineToArray($machine, ?Utilisateur $currentUser = null, ?UtilisateurBadgeRepository $userBadges = null): array
    {

        $requiredBadges = [];
        $hasRequiredBadge = false;
        foreach ($machine->getRequiredMachineBadges() as $machineBadge) {
            $badge = $machineBadge->getBadge();
            if (!$badge) {
                continue;
            }

            $owned = $currentUser instanceof Utilisateur && $userBadges !== null && $userBadges->findOneBy([
                'utilisateur' => $currentUser,
                'badge' => $badge,
            ]) !== null;
            $hasRequiredBadge = $hasRequiredBadge || $owned;

            $requiredBadges[] = [
                'id' => $badge->getId(),
                'nom' => $badge->getNom(),
                'description' => $badge->getDescription(),
                'icone' => $badge->getIcone(),
                'requiredForAccess' => $machineBadge->isRequiredForAccess(),
                'owned' => $currentUser instanceof Utilisateur ? $owned : null,
            ];
        }

        $authorizationStatus = $requiredBadges === [] ? 'no_badge_required' : ($hasRequiredBadge ? 'authorized' : 'missing_badge');

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
            'authorizationStatus' => $currentUser instanceof Utilisateur ? $authorizationStatus : null,
        ];
    }
}
