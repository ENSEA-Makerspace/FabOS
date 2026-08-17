<?php

declare(strict_types=1);

namespace App\UsageRights;

/**
 * How much a package lets you have — "10 hours of machine time a week" (S144c).
 *
 * A grant answers *may you*; an allowance answers *how much*. They are separate
 * rows for the same reason certification and quotas are separate layers in the
 * booking path: a lab that sells more hours must not accidentally widen what a
 * package covers, and a lab that widens coverage must not accidentally hand out
 * more hours.
 *
 * ⚠️ **Allowances are cumulative, like everything else in this model.** The
 * roadmap's rule is "packages cumulatifs, fermés par défaut ; aucun package ne
 * retire un droit" — so a member holding a 5 h package and a 10 h package has
 * 15 h, and the two are summed per (unit, period) before anything is compared.
 * Taking the *highest* instead would mean buying a second package could give you
 * nothing, and taking the *lowest* would mean buying one could take hours away.
 *
 * ⚠️ **No allowance means no ceiling, never a ceiling of zero.** A member whose
 * packages carry no allowance row books as they always have. This is what makes
 * the feature opt-in per package and the migration safe on a live lab.
 */
final readonly class UsageAllowance
{
    /** Minutes of booked time; the unit a lab thinks in when it sells machine hours. */
    public const UNIT_MINUTES = 'minutes';
    /** Whole bookings, for a lab that rations sessions rather than hours. */
    public const UNIT_BOOKINGS = 'bookings';

    public const UNITS = [self::UNIT_MINUTES, self::UNIT_BOOKINGS];

    /**
     * ⚠️ `total` is not a period, it is the absence of one: the allowance is spent
     * once and never resets. That is what sells a prepaid block of hours, and it
     * is why it cannot be expressed as "per year".
     */
    public const PERIODS = ['day', 'week', 'month', 'total'];

    public function __construct(
        public int $id,
        public int $packageId,
        /** The capability's feature key, or null for everything the package covers. */
        public ?string $featureKey,
        /** `ReservableType->value`, or null for any kind of resource. */
        public ?string $reservableType,
        public string $unit,
        public int $amount,
        public string $period,
    ) {
    }

    /** @param array<string, mixed> $row */
    public static function fromRow(array $row): self
    {
        return new self(
            (int) $row['id'],
            (int) $row['packageId'],
            $row['featureKey'] !== null ? (string) $row['featureKey'] : null,
            $row['reservableType'] !== null ? (string) $row['reservableType'] : null,
            (string) $row['unit'],
            (int) $row['amount'],
            (string) $row['period'],
        );
    }

    public static function isValidUnit(string $unit): bool
    {
        return in_array($unit, self::UNITS, true);
    }

    public static function isValidPeriod(string $period): bool
    {
        return in_array($period, self::PERIODS, true);
    }

    /** Two allowances share a budget when they measure the same thing over the same span. */
    public function budgetKey(): string
    {
        return $this->unit . '/' . $this->period . '/' . ($this->reservableType ?? '*') . '/' . ($this->featureKey ?? '*');
    }

    /**
     * Does this allowance have anything to say about this booking?
     *
     * ⚠️ Null is "not restricted to", exactly as it is on a grant — an allowance
     * with no resource kind meters every booking the package covers, and one that
     * names `machine` says nothing at all about booking a room.
     */
    public function applies(?string $featureKey, ?string $reservableType): bool
    {
        return ($this->featureKey === null || $this->featureKey === $featureKey)
            && ($this->reservableType === null || $this->reservableType === $reservableType);
    }

    /**
     * The window this allowance is counted over, containing `$moment`.
     *
     * ⚠️ **Weeks start on Monday** — the same Monday `BookingPolicyService` uses
     * for `maxPerWeek` and the same one the opening hours are read on. Two
     * different week boundaries in one booking path would be a fault nobody could
     * see until a Sunday.
     *
     * @return array{0: ?\DateTimeImmutable, 1: ?\DateTimeImmutable} null bounds mean "since ever" / "for ever"
     */
    public function windowAround(\DateTimeImmutable $moment): array
    {
        return match ($this->period) {
            'day' => [$moment->setTime(0, 0), $moment->setTime(0, 0)->modify('+1 day')],
            'week' => [
                $moment->modify('monday this week')->setTime(0, 0),
                $moment->modify('monday this week')->setTime(0, 0)->modify('+7 days'),
            ],
            'month' => [
                $moment->modify('first day of this month')->setTime(0, 0),
                $moment->modify('first day of this month')->setTime(0, 0)->modify('+1 month'),
            ],
            default => [null, null],
        };
    }
}
