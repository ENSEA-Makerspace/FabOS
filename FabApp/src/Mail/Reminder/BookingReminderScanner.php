<?php

namespace App\Mail\Reminder;

use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Mail\ReminderSettings;
use App\Repository\ReservationRepository;
use App\Repository\UtilisateurRepository;
use App\Reservation\ReservableType;
use App\Service\ModuleService;

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
 *
 * Unlike the other scanners this one cannot be switched off wholesale by a
 * module, because it serves every resource layer at once. It filters per
 * booking instead: a lab that has turned equipment off must stop mailing about
 * equipment slots while still reminding people about their rooms.
 */
final class BookingReminderScanner implements ReminderScanner
{
    public function __construct(
        private readonly ReservationRepository $reservations,
        private readonly UtilisateurRepository $people,
        private readonly ReminderSettings $settings,
        private readonly ModuleService $modules,
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
            if ($id === null || !$this->layerIsEnabled($reservation)) {
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

    /**
     * A disabled module goes quiet everywhere, background jobs included — the
     * booking's pages are 404 by now, so a mail about it would point nowhere.
     * The rows survive untouched, so re-enabling resumes the reminders.
     */
    private function layerIsEnabled(Reservation $reservation): bool
    {
        $type = $reservation->getReservableType();

        // Same map the booking chokepoint uses, so "we stopped taking these" and
        // "we stopped mailing about these" can never answer differently.
        return $type === null || $this->modules->allowsReservable($type);
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
