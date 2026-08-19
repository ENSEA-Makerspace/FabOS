<?php

namespace App\Reservation;

use App\Service\SiteSettingService;
use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Repository\ReservationRepository;
use App\Reservation\Policy\BookingPolicyService;
use App\Schedule\ScheduleResolver;

/**
 * The first slot a given person could actually book on a given resource.
 *
 * This exists so a page can answer "when is this free?" without the member
 * having to open a calendar and hunt — the app already knows the opening hours,
 * the existing bookings and the quota rules, so making somebody read a grid to
 * recover an answer the server can compute is a tax on the common case (S47).
 *
 * ⚠️ **It validates candidates *through* `BookingPolicyService`, it does not
 * reimplement the rules.** Duplicating min-notice or slot alignment here would
 * create a second copy that drifts, and the copy would be the one members see —
 * so this walks candidate slots and asks the real checker about each, taking the
 * first it does not refuse. Slower, and correct by construction.
 *
 * ⚠️ **A suggestion, never a permission.** Nothing here grants anything: the
 * booking still goes through `ReservationService::book()`, which re-runs the
 * access gate, the opening hours, the overlap check and these same quotas. A
 * slot suggested here and taken a second later must still be refused there, and
 * is.
 *
 * ⚠️ **Anonymous visitors get opening-hours-and-overlap only.** Quotas are per
 * person, so with no user there is nothing to check them against. The answer is
 * then "the resource is free then", not "you may book it" — which is exactly
 * what an anonymous visitor should be told.
 */
final class NextFreeSlotService
{
    /** Bookings are laid out on this grid when the policy does not say otherwise. */
    private const DEFAULT_SLOT_MINUTES = 30;

    /** How long a suggested booking runs when the policy sets no minimum. */
    private const DEFAULT_DURATION_MINUTES = 60;

    /** How far ahead to look before giving up and saying nothing. */
    private const DEFAULT_SEARCH_DAYS = 14;

    public function __construct(
        private readonly ScheduleResolver $schedule,
        private readonly ReservableResolver $reservables,
        private readonly ReservationRepository $reservations,
        private readonly BookingPolicyService $policies,
        private readonly SiteSettingService $siteSettings,
    ) {
    }

    /**
     * @return array{start: \DateTimeImmutable, end: \DateTimeImmutable}|null
     *         null when nothing is bookable within the search window — a closed
     *         venue, a fully-booked resource, or a person who has hit a quota.
     */
    public function find(
        ?Utilisateur $user,
        ReservableType $type,
        int $id,
        ?\DateTimeImmutable $now = null,
        int $searchDays = self::DEFAULT_SEARCH_DAYS,
    ): ?array {
        // ⚠️ The box runs UTC and the booking flow is Europe/Paris. Pinning the
        // zone here matters twice over: the slot grid is built from wall-clock
        // opening hours, and the result is rendered straight into the page.
        $zone = new \DateTimeZone($this->siteSettings->getTimezone());
        $now = ($now ?? new \DateTimeImmutable('now'))->setTimezone($zone);
        $venueId = $this->reservables->venueIdFor($type, $id);

        $slotMinutes = self::DEFAULT_SLOT_MINUTES;
        $durationMinutes = self::DEFAULT_DURATION_MINUTES;

        $busy = $this->busyPeriods($type, $id, $now);

        for ($dayOffset = 0; $dayOffset <= $searchDays; $dayOffset++) {
            $day = $now->modify(sprintf('+%d days', $dayOffset))->setTime(0, 0);

            // ⚠️ The location of the resource whose next slot is being
            // proposed. Offering "free tomorrow at 09:00" from another
            // location's week produces a slot the booking chokepoint will
            // then refuse — the worst kind of wrong answer, because it looks
            // like an invitation.
            $open = $this->schedule->openMinutesFor($venueId, $day);
            if ($open === null) {
                continue; // closed that day
            }

            for ($minute = $open['start']; $minute + $durationMinutes <= $open['end']; $minute += $slotMinutes) {
                $start = $day->setTime(intdiv($minute, 60), $minute % 60);
                $end = $start->modify(sprintf('+%d minutes', $durationMinutes));

                if ($start <= $now) {
                    continue;
                }

                if ($this->overlapsBusy($start, $end, $busy)) {
                    continue;
                }

                // With no user there are no quotas to apply — see the class note.
                if ($user !== null && $this->policies->check($user, $type, $id, $start, $end, $now) !== null) {
                    continue;
                }

                return ['start' => $start, 'end' => $end];
            }
        }

        return null;
    }

    /**
     * Existing bookings that could collide, as [start, end] timestamp pairs.
     *
     * Cancelled ones are skipped: a cancelled booking is exactly the slot that
     * has just become free again, and treating it as busy would make cancelling
     * fail to release anything.
     *
     * @return list<array{0: int, 1: int}>
     */
    private function busyPeriods(ReservableType $type, int $id, \DateTimeImmutable $now): array
    {
        $periods = [];
        foreach ($this->reservations->findForReservable($type, $id) as $reservation) {
            if (!$reservation instanceof Reservation || $reservation->isCancelled()) {
                continue;
            }

            $end = $reservation->getDateFin();
            $start = $reservation->getDateDebut();
            if ($start === null || $end === null || $end < $now) {
                continue; // already over, cannot collide with anything ahead
            }

            $periods[] = [$start->getTimestamp(), $end->getTimestamp()];
        }

        return $periods;
    }

    /** @param list<array{0: int, 1: int}> $busy */
    private function overlapsBusy(\DateTimeImmutable $start, \DateTimeImmutable $end, array $busy): bool
    {
        $from = $start->getTimestamp();
        $to = $end->getTimestamp();

        foreach ($busy as [$busyFrom, $busyTo]) {
            // Touching at the boundary is not overlapping: a booking that ends at
            // 14:00 leaves 14:00 free.
            if ($from < $busyTo && $to > $busyFrom) {
                return true;
            }
        }

        return false;
    }
}
