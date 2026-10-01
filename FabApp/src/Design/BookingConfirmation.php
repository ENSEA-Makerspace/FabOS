<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\Utilisateur;
use App\Repository\AccessPointRepository;
use App\Repository\PlaceRepository;
use App\Repository\ReservationRepository;
use App\Reservation\Policy\BookingPolicyService;
use App\Reservation\ReservableType;
use App\Rfid\DoorAccessDecision;
use Symfony\Component\HttpFoundation\Request;

/**
 * « Parcours de réservation » (proposition du 2026-10-01, planche
 * `espaces/03-parcours-reservation.png`) : l'ÉTAT « créneau choisi » d'un espace,
 * pour un créneau fictif lisible (`?espace=&jour=&debut=&fin=`, défaut : le premier
 * espace réservable, demain 10:00–12:00).
 *
 * Rien n'est inventé : les réservations sont celles de la fiche
 * (`findActiveForReservable`), le délai d'annulation vient de `BookingPolicyService`
 * (la même règle que le verbe « annuler »), les portes d'`AccessPointRepository`
 * et la marge d'ouverture de `DoorAccessDecision::MARGIN_MINUTES`.
 */
final class BookingConfirmation
{
    public function __construct(
        private readonly PlaceRepository $places,
        private readonly ReservationRepository $reservations,
        private readonly AccessPointRepository $doors,
        private readonly BookingPolicyService $policies,
    ) {
    }

    /** @return array<string, mixed>|null null : aucun espace à montrer */
    public function build(Request $request, ?Utilisateur $user): ?array
    {
        $live = $this->places->findLive([], ['nom' => 'ASC']);
        $place = null;
        $id = $request->query->getInt('espace');
        foreach ($live as $candidate) {
            if ($id > 0 ? $candidate->getId() === $id : true) {
                $place = $candidate;
                break;
            }
        }
        if ($place === null) {
            return null;
        }

        $day = $this->parseDay((string) $request->query->get('jour', ''));
        $start = $this->parseTime($day, (string) $request->query->get('debut', ''), '10:00');
        $end = $this->parseTime($day, (string) $request->query->get('fin', ''), '12:00');
        if ($end <= $start) {
            $end = $start->modify('+2 hours');
        }
        $minutes = (int) (($end->getTimestamp() - $start->getTimestamp()) / 60);

        $doors = array_map(static fn ($d): string => (string) $d->getNom(), $this->doors->findForPlace((int) $place->getId()));

        // « Annulable jusqu'à » : seulement si la politique en fixe un (null = libre).
        $deadline = $user === null ? null : $this->policies->changeDeadlineFor($user, ReservableType::Place, $start);

        // La journée, heure par heure : occupé / choisi / libre. Les réservations
        // sont lues telles que la fiche les lit (heures saisies, sans fuseau).
        $busy = $this->reservations->findActiveForReservable(ReservableType::Place, (int) $place->getId());
        $from = min(8, (int) $start->format('G'));
        $to = max(19, (int) $end->format('G') + ($end->format('i') === '00' ? 0 : 1));
        $hours = [];
        for ($h = $from; $h < $to; ++$h) {
            $slotStart = $day->setTime($h, 0);
            $slotEnd = $slotStart->modify('+1 hour');
            $state = 'free';
            foreach ($busy as $reservation) {
                if ($reservation->getDateDebut() < $slotEnd && $reservation->getDateFin() > $slotStart) {
                    $state = 'busy';
                    break;
                }
            }
            if ($slotStart < $end && $slotEnd > $start) {
                $state = $state === 'busy' ? 'clash' : 'chosen';
            }
            $hours[] = ['label' => $slotStart->format('H:i'), 'state' => $state];
        }

        return [
            'place' => $place,
            'day' => $day,
            'start' => $start,
            'end' => $end,
            'minutes' => $minutes,
            'doors' => $doors,
            'doorFrom' => $doors === [] ? null : $start->modify('-' . DoorAccessDecision::MARGIN_MINUTES . ' minutes'),
            'doorUntil' => $doors === [] ? null : $end->modify('+' . DoorAccessDecision::MARGIN_MINUTES . ' minutes'),
            'deadline' => $deadline,
            'hours' => $hours,
            'clash' => \in_array('clash', array_column($hours, 'state'), true),
            'choices' => $live,
            'member' => $user,
        ];
    }

    private function parseDay(string $raw): \DateTimeImmutable
    {
        if (preg_match('/^\d{4}-\d{2}-\d{2}$/', $raw) === 1) {
            $parsed = \DateTimeImmutable::createFromFormat('!Y-m-d', $raw);
            if ($parsed !== false) {
                return $parsed;
            }
        }

        return new \DateTimeImmutable('tomorrow');
    }

    private function parseTime(\DateTimeImmutable $day, string $raw, string $default): \DateTimeImmutable
    {
        $time = preg_match('/^([01]\d|2[0-3]):([0-5]\d)$/', $raw) === 1 ? $raw : $default;
        [$h, $m] = array_map('intval', explode(':', $time));

        return $day->setTime($h, $m);
    }
}
