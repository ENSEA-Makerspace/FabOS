<?php

namespace App\Reservation\Policy;

use App\Reservation\ReservableType;

/**
 * The limits that apply to one (resource kind × tier) pair.
 *
 * **Every field is nullable, and null always means "no limit".** That is the
 * whole ergonomic bet of this class: a lab that has never opened the policy
 * screen has no rows, every lookup returns an all-null policy, and booking
 * behaves exactly as it did before quotas existed. Adding a limit is opt-in,
 * one field at a time, and clearing a box is how you remove it — there is no
 * separate "enabled" flag to fall out of sync with the values.
 *
 * Cancellation/reschedule notice is part of the same complete policy path and
 * is consumed centrally by BookingVerbService through changeDeadlineFor().
 */
final readonly class BookingPolicy
{
    public function __construct(
        public ReservableType $type,
        public BookingTier $tier,
        /** How soon before the start a booking may still be made. */
        public ?int $minNoticeMinutes = null,
        /** How far into the future bookings may reach. */
        public ?int $maxHorizonDays = null,
        /** Start times and durations must land on this many minutes. */
        public ?int $slotIncrementMinutes = null,
        public ?int $minDurationMinutes = null,
        public ?int $maxDurationMinutes = null,
        /** Upcoming, still-active bookings this person may hold at once. */
        public ?int $maxActiveReservations = null,
        public ?int $maxPerDay = null,
        public ?int $maxPerWeek = null,
        /** Gap required between two bookings of this resource by this person. */
        public ?int $bufferMinutes = null,
        /** How long before start a member may still cancel or move the booking. */
        public ?int $cancellationNoticeMinutes = null,
    ) {
    }

    /** The do-nothing policy every unconfigured lookup falls back to. */
    public static function unrestricted(ReservableType $type, BookingTier $tier): self
    {
        return new self($type, $tier);
    }

    /** @param array<string, mixed> $row */
    public static function fromRow(array $row): ?self
    {
        $type = ReservableType::tryParse((string) ($row['reservableType'] ?? ''));
        $tier = BookingTier::tryFromValue(isset($row['tier']) ? (string) $row['tier'] : null);

        if ($type === null || $tier === null) {
            return null;
        }

        $int = static fn (string $key): ?int => isset($row[$key]) && $row[$key] !== null && $row[$key] !== ''
            ? max(0, (int) $row[$key])
            : null;

        return new self(
            $type,
            $tier,
            $int('minNoticeMinutes'),
            $int('maxHorizonDays'),
            $int('slotIncrementMinutes'),
            $int('minDurationMinutes'),
            $int('maxDurationMinutes'),
            $int('maxActiveReservations'),
            $int('maxPerDay'),
            $int('maxPerWeek'),
            $int('bufferMinutes'),
            $int('cancellationNoticeMinutes'),
        );
    }

    /** Whether this policy constrains anything at all. */
    public function isUnrestricted(): bool
    {
        return $this->minNoticeMinutes === null
            && $this->maxHorizonDays === null
            && $this->slotIncrementMinutes === null
            && $this->minDurationMinutes === null
            && $this->maxDurationMinutes === null
            && $this->maxActiveReservations === null
            && $this->maxPerDay === null
            && $this->maxPerWeek === null
            && $this->bufferMinutes === null
            && $this->cancellationNoticeMinutes === null;
    }

    /** @return array<string, int|null> field => value, for the admin form. */
    public function toFormValues(): array
    {
        return [
            'minNoticeMinutes' => $this->minNoticeMinutes,
            'maxHorizonDays' => $this->maxHorizonDays,
            'slotIncrementMinutes' => $this->slotIncrementMinutes,
            'minDurationMinutes' => $this->minDurationMinutes,
            'maxDurationMinutes' => $this->maxDurationMinutes,
            'maxActiveReservations' => $this->maxActiveReservations,
            'maxPerDay' => $this->maxPerDay,
            'maxPerWeek' => $this->maxPerWeek,
            'bufferMinutes' => $this->bufferMinutes,
            'cancellationNoticeMinutes' => $this->cancellationNoticeMinutes,
        ];
    }

    /** The editable fields, in the order the admin screen shows them. */
    public const FIELDS = [
        'minNoticeMinutes',
        'maxHorizonDays',
        'slotIncrementMinutes',
        'minDurationMinutes',
        'maxDurationMinutes',
        'maxActiveReservations',
        'maxPerDay',
        'maxPerWeek',
        'bufferMinutes',
        'cancellationNoticeMinutes',
    ];
}
