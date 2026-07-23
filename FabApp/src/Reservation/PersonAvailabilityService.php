<?php

namespace App\Reservation;

use App\Entity\Reservation;
use App\Entity\UserAvailability;
use App\Entity\Utilisateur;
use App\Repository\ReservationRepository;
use App\Repository\UserAvailabilityRepository;
use App\Service\OpeningHoursProvider;

/**
 * Turns a person's weekly availability windows into the concrete list of start
 * times someone can actually click. A slot is offered only where all four agree:
 * the person said they're available that weekday, the lab is open, nothing else
 * is booked, and it isn't in the past.
 *
 * This is a *display* concern — nothing here decides whether a booking is
 * allowed. ReservationService re-checks coverage on POST (via covers()), because
 * a slot list rendered thirty seconds ago is not an authorisation.
 */
final class PersonAvailabilityService
{
    /** How far ahead the booking page offers slots. */
    public const HORIZON_DAYS = 21;

    private const TIMEZONE = 'Europe/Paris';

    public function __construct(
        private readonly UserAvailabilityRepository $availability,
        private readonly ReservationRepository $reservations,
        private readonly OpeningHoursProvider $openingHours,
    ) {
    }

    /**
     * Offered start times per day, for the next $days days.
     *
     * @return list<array{date: \DateTimeImmutable, dayLabel: string, slots: list<array{start: \DateTimeImmutable, end: \DateTimeImmutable}>}>
     */
    public function dailySlots(Utilisateur $person, int $durationMinutes, int $days = self::HORIZON_DAYS): array
    {
        $windowsByDay = $this->availability->findForUserByDay($person);
        if ($windowsByDay === []) {
            return [];
        }

        $now = new \DateTimeImmutable('now', new \DateTimeZone(self::TIMEZONE));
        $booked = $this->bookedRanges($person, $now->modify(sprintf('+%d days', $days)));

        $result = [];
        for ($offset = 0; $offset < $days; ++$offset) {
            $date = $now->modify(sprintf('+%d days', $offset))->setTime(0, 0);
            $slots = $this->slotsOn($date, $windowsByDay[(int) $date->format('N')] ?? [], $durationMinutes, $booked, $now);
            if ($slots !== []) {
                $result[] = ['date' => $date, 'dayLabel' => $date->format('N'), 'slots' => $slots];
            }
        }

        return $result;
    }

    /**
     * Does the person's weekly schedule cover this exact range? False means the
     * range is only bookable as a request the person has to accept.
     */
    public function covers(Utilisateur $person, \DateTimeImmutable $start, \DateTimeImmutable $end): bool
    {
        if ($start->format('Y-m-d') !== $end->format('Y-m-d')) {
            return false;
        }

        $dayOfWeek = (int) $start->format('N');
        $startMinutes = $this->minutes($start);
        $endMinutes = $this->minutes($end);

        foreach ($this->availability->findForUser($person) as $window) {
            if ($window->covers($dayOfWeek, $startMinutes, $endMinutes)) {
                return true;
            }
        }

        return false;
    }

    /** The weekly schedule as the person's page renders it. @return array<int, UserAvailability[]> */
    public function weeklyWindows(Utilisateur $person): array
    {
        return $this->availability->findForUserByDay($person);
    }

    /**
     * @param UserAvailability[] $windows
     * @param list<array{start: int, end: int}> $booked minute-precision timestamps
     *
     * @return list<array{start: \DateTimeImmutable, end: \DateTimeImmutable}>
     */
    private function slotsOn(\DateTimeImmutable $date, array $windows, int $durationMinutes, array $booked, \DateTimeImmutable $now): array
    {
        if ($windows === [] || $durationMinutes <= 0) {
            return [];
        }

        $open = $this->openingHours->getOpenMinutesFor($date);
        if ($open === null) {
            return [];
        }

        $slots = [];
        foreach ($windows as $window) {
            $from = max($window->getStartMinutes(), $open['start']);
            $until = min($window->getEndMinutes(), $open['end']);

            for ($minute = $from; $minute + $durationMinutes <= $until; $minute += $durationMinutes) {
                $start = $date->setTime(intdiv($minute, 60), $minute % 60);
                $end = $start->modify(sprintf('+%d minutes', $durationMinutes));

                if ($start <= $now || $this->overlapsBooked($start, $end, $booked)) {
                    continue;
                }

                $slots[$start->getTimestamp()] = ['start' => $start, 'end' => $end];
            }
        }

        ksort($slots);

        return array_values($slots);
    }

    /**
     * This person's still-live bookings inside the horizon, as timestamp pairs —
     * one query, so a three-week slot grid doesn't issue a query per day.
     *
     * @return list<array{start: int, end: int}>
     */
    private function bookedRanges(Utilisateur $person, \DateTimeImmutable $until): array
    {
        $personId = $person->getId();
        if ($personId === null) {
            return [];
        }

        $ranges = [];
        foreach ($this->reservations->findUpcomingForReservable(ReservableType::User, $personId) as $reservation) {
            /** @var Reservation $reservation */
            if ($reservation->getDateDebut() > $until) {
                continue;
            }

            $ranges[] = [
                'start' => $reservation->getDateDebut()->getTimestamp(),
                'end' => $reservation->getDateFin()->getTimestamp(),
            ];
        }

        return $ranges;
    }

    /** @param list<array{start: int, end: int}> $booked */
    private function overlapsBooked(\DateTimeImmutable $start, \DateTimeImmutable $end, array $booked): bool
    {
        $startTs = $start->getTimestamp();
        $endTs = $end->getTimestamp();

        foreach ($booked as $range) {
            if ($range['start'] < $endTs && $range['end'] > $startTs) {
                return true;
            }
        }

        return false;
    }

    private function minutes(\DateTimeImmutable $moment): int
    {
        return (int) $moment->format('H') * 60 + (int) $moment->format('i');
    }
}
