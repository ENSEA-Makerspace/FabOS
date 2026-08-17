<?php

declare(strict_types=1);

namespace App\UsageRights;

/**
 * One slice of the week a grant is awake for — "Monday 14:00 to 18:00" (S144b).
 *
 * The operator's example was "this package lets you 3D print on Monday
 * afternoons", and until this existed the model could say *what* a package
 * covered and *where*, but never *when* — so a lab selling off-peak access had
 * to either sell full access or not sell it.
 *
 * ⚠️ **Minutes from local midnight, ISO day numbers.** `dayOfWeek` is PHP's `N`:
 * 1 = Monday … 7 = Sunday. Storing wall-clock minutes rather than instants is the
 * whole point — "Monday afternoon" is a fact about the lab's clock, so it must
 * survive a timezone change the way the opening hours do, not shift by an hour in
 * summer.
 *
 * ⚠️ `endMinute` may be 1440, which is midnight at the end of the day. It is
 * deliberately not 0: a window ending at 0 would be empty, and a booking that
 * runs to midnight is the one a lab most wants to allow.
 */
final readonly class GrantWindow
{
    public const MINUTES_IN_DAY = 1440;

    public function __construct(
        public int $dayOfWeek,
        public int $startMinute,
        public int $endMinute,
    ) {
    }

    /** @param array<string, mixed> $row */
    public static function fromRow(array $row): self
    {
        return new self((int) $row['dayOfWeek'], (int) $row['startMinute'], (int) $row['endMinute']);
    }

    /**
     * ⚠️ **"00:00" as an END is midnight at the END of the day.** `24:00` is what
     * this model means and `00:00` is what a browser's `<input type="time">`
     * produces when somebody types the end of the day. Read as minute zero it
     * would be a backwards window, `isValid()` would reject it, and an operator
     * would be told their opening hours are invalid for writing exactly what they
     * meant. Only the END is reinterpreted — a window starting at midnight starts
     * at minute zero, which is also what it says.
     */
    public static function fromClock(int $dayOfWeek, string $start, string $end): self
    {
        $endMinute = self::minutes($end);

        return new self($dayOfWeek, self::minutes($start), $endMinute === 0 ? self::MINUTES_IN_DAY : $endMinute);
    }

    private static function minutes(string $clock): int
    {
        $parts = explode(':', trim($clock));
        $minutes = ((int) ($parts[0] ?? 0)) * 60 + ((int) ($parts[1] ?? 0));

        return max(0, min(self::MINUTES_IN_DAY, $minutes));
    }

    public function isValid(): bool
    {
        return $this->dayOfWeek >= 1
            && $this->dayOfWeek <= 7
            && $this->startMinute >= 0
            && $this->endMinute <= self::MINUTES_IN_DAY
            && $this->endMinute > $this->startMinute;
    }

    public static function clock(int $minutes): string
    {
        return sprintf('%02d:%02d', intdiv($minutes, 60), $minutes % 60);
    }

    public function getStartClock(): string
    {
        return self::clock($this->startMinute);
    }

    public function getEndClock(): string
    {
        return self::clock($this->endMinute);
    }

    public function label(): string
    {
        return $this->getStartClock() . ' – ' . $this->getEndClock();
    }
}
