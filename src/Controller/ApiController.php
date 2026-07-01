<?php

namespace App\Controller;

use App\Entity\Reservation;
use App\Repository\BadgeRepository;
use App\Repository\FormationRepository;
use App\Repository\MachineRepository;
use App\Repository\ReservationRepository;
use App\Repository\UtilisateurRepository;
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

    #[Route('/machines', name: 'api_machines', methods: ['GET'])]
    public function machines(MachineRepository $machines): JsonResponse
    {
        return new JsonResponse(array_map(fn ($machine) => $this->machineToArray($machine), $machines->findAll()));
    }

    #[Route('/machines/{id}', name: 'api_machine_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function machine(int $id, MachineRepository $machines): JsonResponse
    {
        $machine = $machines->find($id);
        if (!$machine) {
            return new JsonResponse(['error' => 'Machine introuvable'], 404);
        }

        return new JsonResponse($this->machineToArray($machine));
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
        }, $reservations->findBy([], ['dateDebut' => 'ASC'])));
    }

    #[Route('/formations', name: 'api_formations', methods: ['GET'])]
    public function formations(FormationRepository $formations): JsonResponse
    {
        return new JsonResponse(array_map(static fn ($formation) => [
            'id' => $formation->getId(),
            'titre' => $formation->getTitre(),
            'description' => $formation->getDescription(),
            'image' => $formation->getImage(),
            'badge' => $formation->getBadge() ? [
                'id' => $formation->getBadge()->getId(),
                'nom' => $formation->getBadge()->getNom(),
                'icone' => $formation->getBadge()->getIcone(),
            ] : null,
        ], $formations->findAll()));
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

    #[Route('/badges', name: 'api_badges', methods: ['GET'])]
    public function badges(BadgeRepository $badges): JsonResponse
    {
        return new JsonResponse(array_map(static fn ($badge) => [
            'id' => $badge->getId(),
            'nom' => $badge->getNom(),
            'description' => $badge->getDescription(),
            'icone' => $badge->getIcone(),
            'createdAt' => $badge->getCreatedAt()->format(DATE_ATOM),
        ], $badges->findAll()));
    }

    #[Route('/search', name: 'api_search', methods: ['GET'])]
    public function search(Request $request, MachineRepository $machines, FormationRepository $formations, UtilisateurRepository $users): JsonResponse
    {
        $q = mb_strtolower(trim((string) $request->query->get('q', '')));
        if ($q === '') {
            return new JsonResponse(['query' => '', 'results' => []]);
        }

        $results = [];
        foreach ($machines->findAll() as $machine) {
            if (str_contains(mb_strtolower($machine->getNom() . ' ' . ($machine->getDescription() ?? '') . ' ' . ($machine->getLocalisation() ?? '')), $q)) {
                $results[] = ['type' => 'machine', 'id' => $machine->getId(), 'title' => $machine->getNom(), 'url' => $this->generateUrl('app_machine_detail', ['id' => $machine->getId()])];
            }
        }
        foreach ($formations->findAll() as $formation) {
            if (str_contains(mb_strtolower($formation->getTitre() . ' ' . ($formation->getDescription() ?? '')), $q)) {
                $results[] = ['type' => 'formation', 'id' => $formation->getId(), 'title' => $formation->getTitre(), 'url' => $this->generateUrl('app_formation_detail', ['id' => $formation->getId()])];
            }
        }
        foreach ($users->findAll() as $user) {
            if (str_contains(mb_strtolower($user->getUsername() . ' ' . $user->getDisplayName() . ' ' . $user->getEmail()), $q)) {
                $results[] = ['type' => 'utilisateur', 'id' => $user->getId(), 'title' => $user->getDisplayName(), 'url' => $this->generateUrl('app_profile')];
            }
        }

        return new JsonResponse(['query' => $q, 'results' => $results]);
    }

    #[Route('/reservations', name: 'api_reservation_create', methods: ['POST'])]
    public function createReservation(Request $request, MachineRepository $machines, UtilisateurRepository $users, EntityManagerInterface $em): JsonResponse
    {
        $payload = json_decode($request->getContent(), true);
        if (!is_array($payload)) {
            return new JsonResponse(['error' => 'JSON invalide'], 400);
        }

        $machine = $machines->find((int) ($payload['machineId'] ?? 0));
        if (!$machine) {
            return new JsonResponse(['error' => 'Machine introuvable'], 404);
        }

        $user = $users->find((int) ($payload['userId'] ?? 0));
        if (!$user) {
            return new JsonResponse(['error' => 'Utilisateur requis: envoie userId, car RESERVATION.userId est NOT NULL dans le SQL.'], 400);
        }

        try {
            $dateDebut = new \DateTimeImmutable((string) ($payload['dateDebut'] ?? $payload['startAt'] ?? ''));
            $dateFin = new \DateTimeImmutable((string) ($payload['dateFin'] ?? $payload['endAt'] ?? ''));
        } catch (\Throwable) {
            return new JsonResponse(['error' => 'Dates invalides'], 400);
        }

        if ($dateFin <= $dateDebut) {
            return new JsonResponse(['error' => 'La fin doit être après le début'], 400);
        }

        $reservation = (new Reservation())
            ->setMachine($machine)
            ->setUtilisateur($user)
            ->setDateDebut($dateDebut)
            ->setDateFin($dateFin)
            ->setMotif($payload['motif'] ?? $payload['comment'] ?? null);

        $em->persist($reservation);
        $em->flush();

        return new JsonResponse(['id' => $reservation->getId(), 'status' => 'created'], 201);
    }

    private function machineToArray($machine): array
    {
        return [
            'id' => $machine->getId(),
            'nom' => $machine->getNom(),
            'description' => $machine->getDescription(),
            'localisation' => $machine->getLocalisation(),
            'photo' => $machine->getPhoto(),
            'statut' => $machine->getStatut(),
            'granularite' => $machine->getGranularite(),
            'limiteReservations' => $machine->getLimiteReservations(),
            'machineToken' => $machine->getMachineToken(),
            'createdAt' => $machine->getCreatedAt()->format(DATE_ATOM),
            'updated' => $machine->getUpdated()->format(DATE_ATOM),
            'lastAuthorizationTime' => $machine->getLastAuthorizationTime()?->format(DATE_ATOM),
        ];
    }
}
