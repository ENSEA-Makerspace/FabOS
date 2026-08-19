<?php

declare(strict_types=1);

namespace App\Schedule;

use App\Entity\OpeningHour;
use App\Repository\OpeningHourRepository;
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
 * ⚠️ **This is the model half only.** Several ranges per day, attachable scopes
 * and dated exceptions are S134d/e and need a migration —
 * `UNIQ_OPENING_HOUR_VENUE_DAY` still forbids a lunch break. What is fixed here
 * is *which* location answers, which needs no schema change and was wrong today.
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

    public function __construct(
        private readonly OpeningHourRepository $openingHours,
        private readonly VenueRepository $venues,
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

        if (count($rows) !== 7) {
            // ⚠️ Not an error for a location nobody has configured yet — it is
            // what every location was already using. Logged at info for the
            // default venue's own gap, which IS worth noticing.
            $rows = $this->defaultVenueRows($venueId);
        }

        return $this->memo[$key] = array_values($rows);
    }

    /**
     * Why this booking cannot happen, or null.
     *
     * ⚠️ Returns the sentence rather than a boolean because the member has to be
     * told *which* rule refused them, and "the lab is closed then" is a different
     * answer from "a booking cannot span two days".
     */
    public function refusalFor(?int $venueId, \DateTimeImmutable $from, \DateTimeImmutable $until): ?string
    {
        if ($from->format('Y-m-d') !== $until->format('Y-m-d')) {
            return 'Une réservation ne peut pas traverser deux jours différents.';
        }

        $open = $this->openMinutesFor($venueId, $from);
        if ($open === null) {
            return 'Le FabLab est fermé sur ce créneau.';
        }

        $startMinute = ((int) $from->format('H')) * 60 + (int) $from->format('i');
        $endMinute = ((int) $until->format('H')) * 60 + (int) $until->format('i');
        if ($startMinute < $open['start'] || $endMinute > $open['end']) {
            return 'Le FabLab est fermé sur ce créneau.';
        }

        return null;
    }

    /**
     * The open window on a date, as minutes since local midnight, or null when
     * the location is closed. The slot engine intersects a person's availability
     * with this rather than re-reading rows itself.
     *
     * @return array{start: int, end: int}|null
     */
    public function openMinutesFor(?int $venueId, \DateTimeInterface $date): ?array
    {
        $row = $this->rowForDate($venueId, $date);
        if ($row === null || $row->isClosed() || $row->getOpenTime() === null || $row->getCloseTime() === null) {
            return null;
        }

        return [
            'start' => ((int) $row->getOpenTime()->format('H')) * 60 + (int) $row->getOpenTime()->format('i'),
            'end' => ((int) $row->getCloseTime()->format('H')) * 60 + (int) $row->getCloseTime()->format('i'),
        ];
    }

    public function isOpenAt(?int $venueId, \DateTimeInterface $moment): bool
    {
        $open = $this->openMinutesFor($venueId, $moment);
        if ($open === null) {
            return false;
        }

        $minute = ((int) $moment->format('H')) * 60 + (int) $moment->format('i');

        return $minute >= $open['start'] && $minute < $open['end'];
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

    /** @return array<int, array<string, mixed>> */
    public function forJson(?int $venueId): array
    {
        return array_map(static fn (OpeningHour $row): array => [
            'dayOfWeek' => $row->getDayOfWeek(),
            'dayIndex' => $row->getDayOfWeek() - 1,
            'label' => $row->getLabel(),
            'isClosed' => $row->isClosed(),
            'openTime' => $row->getOpenTime()?->format('H:i'),
            'closeTime' => $row->getCloseTime()?->format('H:i'),
            'displayLabel' => $row->getDisplayLabel(),
        ], $this->rowsFor($venueId));
    }

    /** @return list<OpeningHour> */
    private function openRows(?int $venueId, string $accessor): array
    {
        return array_values(array_filter(
            $this->rowsFor($venueId),
            static fn (OpeningHour $row): bool => !$row->isClosed() && $row->{$accessor}() !== null,
        ));
    }

    private function rowForDate(?int $venueId, \DateTimeInterface $date): ?OpeningHour
    {
        foreach ($this->rowsFor($venueId) as $row) {
            if ($row->appliesTo($date)) {
                return $row;
            }
        }

        return null;
    }

    /** @return list<OpeningHour> */
    private function defaultVenueRows(?int $askedVenueId): array
    {
        $venue = $this->venues->findDefault();
        $rows = $venue === null ? [] : $this->openingHours->findOrdered($venue);
        if (count($rows) === 7) {
            return array_values($rows);
        }

        $this->logger->warning('OPENING_HOUR is incomplete for the default venue; falling back to the built-in week.', [
            'asked' => $askedVenueId,
            'count' => count($rows),
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
