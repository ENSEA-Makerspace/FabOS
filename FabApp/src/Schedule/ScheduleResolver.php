<?php

declare(strict_types=1);

namespace App\Schedule;

use App\Entity\OpeningHour;
use App\Repository\OpeningHourRepository;
use App\Repository\ScheduleExceptionRepository;
use App\Repository\VenueRepository;
use Psr\Log\LoggerInterface;

/**
 * "Is this location open at that moment?" — asked in exactly one place (S145a).
 *
 * 🔴 **The fault this class exists for.** `OpeningHoursProvider` resolved the
 * lab's hours by calling `VenueRepository::findDefault()` — the venue whose slug
 * is literally `default` — and read nothing else. `ReservationService` then
 * validated **every** booking through it without passing a location at all. On a
 * multi-location install that meant a machine at the second location was checked
 * against the first location's opening hours, and an install with no
 * `slug = 'default'` venue had every booking checked against seven hard-coded
 * rows in a PHP constant.
 *
 * ⚠️ It is the same shape as the venue-scoped grant fault of S134b, one
 * subsystem along: the schema had the dimension, the screen wrote it, and the
 * reader never asked. That is now twice. **When a table carries `venueId`, the
 * question to ask of every reader is not "does it join the column" but "does the
 * CALLER supply a value".**
 *
 * ⚠️ **The fallback ladder is deliberate and preserves today's behaviour.** A
 * location with its own rows uses them; a location with none falls back to the
 * default venue's — which is what *every* location silently got before — and only
 * then to the built-in week. So the change can only ever make an answer more
 * correct, never take a lab offline because somebody has not filled a screen in
 * yet.
 *
 * ⚠️ **Null means "no location in the question", not "no restriction".** A
 * booking with a person rather than a machine has no location, and it stays
 * constrained by the default venue's hours exactly as before. Opening hours are
 * a restriction, so making null mean "unconstrained" would quietly let bookings
 * through the closed hours of a lab that had never asked for that.
 *
 * ⚠️ **S134d added the two things the model could not say.** A weekday may hold
 * SEVERAL ranges (`UNIQ_OPENING_HOUR_VENUE_DAY` is dropped), and a DATE may
 * override the week entirely (`SCHEDULE_EXCEPTION`). The primitive is therefore
 * `openIntervalsFor()` — a list, possibly empty, never a single pair — and every
 * other answer is derived from it so the envelope and the ranges cannot disagree.
 *
 * ⚠️ **A booking must fit inside ONE interval.** Spanning a lunch break is not
 * "mostly open"; it is booking the hour the lab is shut. `refusalFor()` therefore
 * looks for a containing interval rather than testing the outer envelope.
 *
 * ⚠️ **Attachable scope — per workspace, per resource — is still NOT built.**
 * The signatures take a venue and nothing narrower. Adding a scope later means
 * widening the parameter, not rewriting the callers.
 */
final class ScheduleResolver
{
    private const DEFAULT_ROWS = [
        1 => ['label' => 'Lundi', 'isClosed' => false, 'openTime' => '08:00', 'closeTime' => '20:00'],
        2 => ['label' => 'Mardi', 'isClosed' => false, 'openTime' => '08:00', 'closeTime' => '20:00'],
        3 => ['label' => 'Mercredi', 'isClosed' => false, 'openTime' => '08:00', 'closeTime' => '20:00'],
        4 => ['label' => 'Jeudi', 'isClosed' => false, 'openTime' => '08:00', 'closeTime' => '20:00'],
        5 => ['label' => 'Vendredi', 'isClosed' => false, 'openTime' => '08:00', 'closeTime' => '22:00'],
        6 => ['label' => 'Samedi', 'isClosed' => false, 'openTime' => '09:00', 'closeTime' => '18:00'],
        7 => ['label' => 'Dimanche', 'isClosed' => true, 'openTime' => null, 'closeTime' => null],
    ];

    /** @var array<string, list<OpeningHour>> memoised per request; a calendar asks once per rendered day */
    private array $memo = [];

    /** @var array<string, list<\App\Entity\ScheduleException>> one lookup per (venue, date) per request */
    private array $exceptionMemo = [];

    public function __construct(
        private readonly OpeningHourRepository $openingHours,
        private readonly VenueRepository $venues,
        private readonly ScheduleExceptionRepository $exceptions,
        private readonly LoggerInterface $logger,
    ) {
    }

    /**
     * The week of one location.
     *
     * @return list<OpeningHour>
     */
    public function rowsFor(?int $venueId): array
    {
        $key = $venueId === null ? 'default' : (string) $venueId;
        if (isset($this->memo[$key])) {
            return $this->memo[$key];
        }

        $rows = [];
        if ($venueId !== null) {
            $venue = $this->venues->find($venueId);
            $rows = $venue === null ? [] : $this->openingHours->findOrdered($venue);
        }

        // 🔴 **This was `count($rows) !== 7`, and S134d makes that wrong.** A
        // location with a lunch break on Tuesday has eight rows, and the old test
        // would have thrown its whole week away and served the built-in one — the
        // feature silently undoing itself the moment anybody used it. What
        // actually means "unconfigured" is having no rows at all.
        if ($rows === []) {
            $rows = $this->defaultVenueRows($venueId);
        }

        return $this->memo[$key] = array_values($rows);
    }

    /**
     * Why this booking cannot happen, or null.
     *
     * ⚠️ Returns the sentence rather than a boolean because the member has to be
     * told *which* rule refused them, and "the lab is closed then" is a different
     * answer from "a booking cannot span two days" — or from "closed: public
     * holiday", which the exception can now name.
     *
     * 🔴 **It looks for a CONTAINING interval, not the outer envelope.** With a
     * lunch break, 11:00–15:00 sits inside the envelope 09:00–18:00 and is still
     * an hour of booking the shut lab. Testing the envelope would make the
     * feature decorative on the first day somebody used it.
     */
    public function refusalFor(?int $venueId, \DateTimeImmutable $from, \DateTimeImmutable $until): ?string
    {
        if ($from->format('Y-m-d') !== $until->format('Y-m-d')) {
            return 'Une réservation ne peut pas traverser deux jours différents.';
        }

        $closed = 'Le FabLab est fermé sur ce créneau.';
        $reason = $this->closureReasonFor($venueId, $from);
        if ($reason !== null) {
            $closed .= ' (' . $reason . ')';
        }

        $intervals = $this->openIntervalsFor($venueId, $from);
        if ($intervals === []) {
            return $closed;
        }

        $startMinute = ((int) $from->format('H')) * 60 + (int) $from->format('i');
        $endMinute = ((int) $until->format('H')) * 60 + (int) $until->format('i');
        foreach ($intervals as $interval) {
            if ($startMinute >= $interval['start'] && $endMinute <= $interval['end']) {
                return null;
            }
        }

        return $closed;
    }

    /**
     * Every open range on a date, in minutes since local midnight.
     *
     * ⚠️ **The single primitive.** Everything else here is derived from it, so
     * the envelope a calendar lays out and the ranges a booking is judged against
     * can never come from two different readings of the table.
     *
     * ⚠️ **A dated exception REPLACES the weekday.** If any row exists for the
     * date, that date's opening is exactly what those rows say — a closure, or a
     * special opening. Merging with the weekly grid would make "open 09:00–12:00
     * only today" impossible to express.
     *
     * @return list<array{start: int, end: int}> sorted, merged, never overlapping
     */
    public function openIntervalsFor(?int $venueId, \DateTimeInterface $date): array
    {
        $exceptions = $this->exceptionsFor($venueId, $date);
        if ($exceptions !== []) {
            $intervals = [];
            foreach ($exceptions as $exception) {
                if ($exception->opensTheDay()) {
                    $intervals[] = $this->minutesOf($exception->getOpenTime(), $exception->getCloseTime());
                }
            }

            return $this->normalise($intervals);
        }

        $intervals = [];
        foreach ($this->rowsFor($venueId) as $row) {
            if (!$row->appliesTo($date) || $row->isClosed() || $row->getOpenTime() === null || $row->getCloseTime() === null) {
                continue;
            }
            $intervals[] = $this->minutesOf($row->getOpenTime(), $row->getCloseTime());
        }

        return $this->normalise($intervals);
    }

    /**
     * Why the location is shut on this date, when an exception says so.
     *
     * ⚠️ "Closed" leaves a member wondering whether the lab is broken or whether
     * they misread the page. "Closed — public holiday" ends the question, and it
     * is the only part of an exception a member ever sees.
     */
    public function closureReasonFor(?int $venueId, \DateTimeInterface $date): ?string
    {
        if ($this->openIntervalsFor($venueId, $date) !== []) {
            return null;
        }

        foreach ($this->exceptionsFor($venueId, $date) as $exception) {
            if ($exception->getReason() !== null) {
                return $exception->getReason();
            }
        }

        return null;
    }

    /**
     * The outer envelope of a date — earliest opening to latest closing.
     *
     * ⚠️ **For LAYOUT, never for permission.** A calendar needs to know which
     * rows of the grid to draw; whether a given slot is bookable is
     * `isOpenAt()`'s question and must not be answered from here, or a lunch
     * break becomes bookable again.
     *
     * @return array{start: int, end: int}|null
     */
    public function openMinutesFor(?int $venueId, \DateTimeInterface $date): ?array
    {
        $intervals = $this->openIntervalsFor($venueId, $date);
        if ($intervals === []) {
            return null;
        }

        return [
            'start' => $intervals[0]['start'],
            'end' => $intervals[count($intervals) - 1]['end'],
        ];
    }

    public function isOpenAt(?int $venueId, \DateTimeInterface $moment): bool
    {
        $minute = ((int) $moment->format('H')) * 60 + (int) $moment->format('i');
        foreach ($this->openIntervalsFor($venueId, $moment) as $interval) {
            if ($minute >= $interval['start'] && $minute < $interval['end']) {
                return true;
            }
        }

        return false;
    }

    public function calendarStartHour(?int $venueId): int
    {
        $open = $this->openRows($venueId, 'getOpenTime');

        return $open === [] ? 8 : min(array_map(
            static fn (OpeningHour $row): int => (int) $row->getOpenTime()->format('G'),
            $open,
        ));
    }

    public function calendarEndHour(?int $venueId): int
    {
        $open = $this->openRows($venueId, 'getCloseTime');

        return $open === [] ? 20 : max(array_map(
            static fn (OpeningHour $row): int => (int) ceil((((int) $row->getCloseTime()->format('H')) * 60 + (int) $row->getCloseTime()->format('i')) / 60),
            $open,
        ));
    }

    /**
     * The week as a calendar's JavaScript reads it.
     *
     * 🔴 **One entry per DAY, not per row** (S134d). Both calendars build their
     * lookup with `hours[row.dayIndex] = …`, so the moment a day held two rows the
     * second silently overwrote the first and the lab's afternoon disappeared from
     * the grid. The entry now carries `ranges`, and `openTime`/`closeTime` are
     * kept as the day's ENVELOPE so the existing layout maths is unchanged.
     * ⚠️ Anything deciding whether a slot is bookable must read `ranges`; the
     * envelope spans the lunch break by construction.
     *
     * @return list<array<string, mixed>>
     */
    public function forJson(?int $venueId): array
    {
        $byDay = [];
        foreach ($this->rowsFor($venueId) as $row) {
            $day = $row->getDayOfWeek();
            $byDay[$day] ??= ['label' => $row->getLabel(), 'rows' => []];
            $byDay[$day]['rows'][] = $row;
        }
        ksort($byDay);

        $out = [];
        foreach ($byDay as $day => $data) {
            $intervals = $this->normalise(array_values(array_map(
                fn (OpeningHour $row): array => $this->minutesOf($row->getOpenTime(), $row->getCloseTime()),
                array_values(array_filter(
                    $data['rows'],
                    static fn (OpeningHour $row): bool => !$row->isClosed() && $row->getOpenTime() !== null && $row->getCloseTime() !== null,
                )),
            )));

            $out[] = [
                'dayOfWeek' => $day,
                'dayIndex' => $day - 1,
                'label' => $data['label'],
                'isClosed' => $intervals === [],
                'openTime' => $intervals === [] ? null : self::clock($intervals[0]['start']),
                'closeTime' => $intervals === [] ? null : self::clock($intervals[count($intervals) - 1]['end']),
                'ranges' => array_map(
                    static fn (array $i): array => [self::clock($i['start']), self::clock($i['end'])],
                    $intervals,
                ),
                'displayLabel' => $intervals === []
                    ? $data['rows'][0]->getDisplayLabel()
                    : implode(' · ', array_map(
                        static fn (array $i): string => self::clock($i['start']) . ' – ' . self::clock($i['end']),
                        $intervals,
                    )),
            ];
        }

        return $out;
    }

    private static function clock(int $minutes): string
    {
        return sprintf('%02d:%02d', intdiv($minutes, 60), $minutes % 60);
    }

    /** @return array{start: int, end: int} */
    private function minutesOf(\DateTimeInterface $open, \DateTimeInterface $close): array
    {
        return [
            'start' => ((int) $open->format('H')) * 60 + (int) $open->format('i'),
            'end' => ((int) $close->format('H')) * 60 + (int) $close->format('i'),
        ];
    }

    /**
     * Sort, drop the empty, and merge what touches.
     *
     * ⚠️ Two ranges written as 09:00–12:00 and 12:00–14:00 are one opening from
     * nine to two, and a booking from 11:00 to 13:00 must not be refused for
     * crossing a boundary that exists only in the operator's data entry. The same
     * merge `GrantWindowSet` does for package windows, for the same reason.
     *
     * @param list<array{start: int, end: int}> $intervals
     * @return list<array{start: int, end: int}>
     */
    private function normalise(array $intervals): array
    {
        $intervals = array_values(array_filter($intervals, static fn (array $i): bool => $i['end'] > $i['start']));
        usort($intervals, static fn (array $a, array $b): int => $a['start'] <=> $b['start']);

        $merged = [];
        foreach ($intervals as $interval) {
            $last = $merged === [] ? null : count($merged) - 1;
            if ($last !== null && $interval['start'] <= $merged[$last]['end']) {
                $merged[$last]['end'] = max($merged[$last]['end'], $interval['end']);
                continue;
            }
            $merged[] = $interval;
        }

        return $merged;
    }

    /**
     * ⚠️ Resolved through the VENUE the rows belong to, not through the asked
     * id: a location falling back to the default venue's week must fall back to
     * its exceptions too, or it would be open on a holiday the lab has closed.
     *
     * @return list<\App\Entity\ScheduleException>
     */
    private function exceptionsFor(?int $venueId, \DateTimeInterface $date): array
    {
        $venue = $venueId !== null ? $this->venues->find($venueId) : null;
        $venue ??= $this->venues->findDefault();
        if ($venue === null) {
            return [];
        }

        $key = $venue->getId() . '@' . $date->format('Y-m-d');
        if (!isset($this->exceptionMemo[$key])) {
            $this->exceptionMemo[$key] = $this->exceptions->forDate($venue, $date);
        }

        return $this->exceptionMemo[$key];
    }

    /** @return list<OpeningHour> */
    private function openRows(?int $venueId, string $accessor): array
    {
        return array_values(array_filter(
            $this->rowsFor($venueId),
            static fn (OpeningHour $row): bool => !$row->isClosed() && $row->{$accessor}() !== null,
        ));
    }

    /** @return list<OpeningHour> */
    private function defaultVenueRows(?int $askedVenueId): array
    {
        $venue = $this->venues->findDefault();
        $rows = $venue === null ? [] : $this->openingHours->findOrdered($venue);

        // 🔴 **This was `count($rows) === 7` too, and missing it here was worse
        // than missing it in `rowsFor()`.** The moment the default venue gained a
        // lunch break it had eight rows, so every caller that asks without a
        // location — `rowsFor(null)`: the homepage, the aggregated calendar, the
        // public API, the seeding of a new location's week, and the availability
        // of every PERSON booking — silently got the built-in 08:00–20:00 week
        // instead of the lab's real hours. Found by the log line this method
        // emits, in the middle of a self-test whose assertions all passed:
        // the assertions asked the default venue by id and took the other branch.
        if ($rows !== []) {
            return array_values($rows);
        }

        $this->logger->warning('OPENING_HOUR is empty for the default venue; falling back to the built-in week.', [
            'asked' => $askedVenueId,
        ]);

        return $this->builtInWeek();
    }

    /** @return list<OpeningHour> */
    private function builtInWeek(): array
    {
        $rows = [];
        foreach (self::DEFAULT_ROWS as $dayOfWeek => $data) {
            $rows[] = (new OpeningHour())
                ->setDayOfWeek($dayOfWeek)
                ->setLabel($data['label'])
                ->setIsClosed($data['isClosed'])
                ->setOpenTime($data['openTime'] !== null ? \DateTime::createFromFormat('H:i', $data['openTime']) : null)
                ->setCloseTime($data['closeTime'] !== null ? \DateTime::createFromFormat('H:i', $data['closeTime']) : null)
                ->setSortOrder($dayOfWeek);
        }

        return $rows;
    }
}
