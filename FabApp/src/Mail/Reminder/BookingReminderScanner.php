<?php

namespace App\Mail\Reminder;

use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Mail\ReminderSettings;
use App\Repository\ReservationRepository;
use App\Repository\UtilisateurRepository;
use App\Reservation\ReservableType;

/**
 * "Your booking starts tomorrow."
 *
 * Sweeps every booking starting between now and the configured lead time rather
 * than trying to hit a precise moment. A booking made *inside* the window still
 * gets its reminder on the next tick, and a timer that misses a run doesn't
 * silently drop a day's worth of reminders — both of which a
 * fire-at-exactly-T-minus-24h design gets wrong.
 *
 * Pending requests are included: an unanswered request is the booking people
 * most want reminding about.
 */
final class BookingReminderScanner implements ReminderScanner
{
    public function __construct(
        private readonly ReservationRepository $reservations,
        private readonly UtilisateurRepository $people,
        private readonly ReminderSettings $settings,
    ) {
    }

    public function kind(): string
    {
        return ReminderSettings::BOOKING;
    }

    public function scan(\DateTimeImmutable $now): array
    {
        $horizon = $now->modify(sprintf('+%d hours', $this->settings->getBookingLeadHours()));

        $candidates = [];
        foreach ($this->reservations->findStartingBetween($now, $horizon) as $reservation) {
            $id = $reservation->getId();
            if ($id === null) {
                continue;
            }

            $context = $this->context($reservation);

            $booker = $reservation->getUtilisateur();
            if ($booker !== null) {
                $candidates[] = ReminderCandidate::forUser(
                    sprintf('booking:%d:start', $id),
                    $booker,
                    'reminder_booking',
                    $context,
                );
            }

            // When the resource is somebody's time, the appointment is on their
            // calendar too — they get their own reminder, keyed separately.
            $person = $this->bookedPerson($reservation);
            if ($person !== null && $person->getId() !== $booker?->getId()) {
                $candidates[] = ReminderCandidate::forUser(
                    sprintf('booking:%d:start:person', $id),
                    $person,
                    'reminder_booking',
                    $context,
                );
            }
        }

        return $candidates;
    }

    /** @return array<string, mixed> */
    private function context(Reservation $reservation): array
    {
        $booker = $reservation->getUtilisateur();

        return [
            // ISO strings, like the booking mails: the context is stored as JSON
            // and re-rendered by the worker in the recipient's own language.
            'start' => $reservation->getDateDebut()->format(\DATE_ATOM),
            'end' => $reservation->getDateFin()->format(\DATE_ATOM),
            'resource' => $reservation->getReservableLabel() ?? '',
            'kind' => $reservation->getReservableType()?->value ?? '',
            'motif' => $reservation->getMotif(),
            'booker' => $booker !== null ? ($booker->getDisplayName() ?: $booker->getEmail()) : '',
            'pending' => $reservation->isPending(),
        ];
    }

    private function bookedPerson(Reservation $reservation): ?Utilisateur
    {
        if ($reservation->getReservableType() !== ReservableType::User) {
            return null;
        }

        $id = $reservation->getReservableId();

        return $id === null ? null : $this->people->find($id);
    }
}
