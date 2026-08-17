<?php

namespace App\Tests\UsageRights;

use App\UsageRights\UsageAllowance;
use PHPUnit\Framework\TestCase;

/**
 * "Ten hours of machine time a week" — the arithmetic underneath it.
 *
 * ⚠️ The whole risk of this feature is in two places: **which period a booking
 * falls into**, and **which allowances share a budget**. Both are pure functions
 * and both are silently wrong in ways nobody sees until a Sunday or until a
 * member holding two packages is refused with the wrong number.
 */
final class UsageAllowanceTest extends TestCase
{
    private function allowance(string $unit, int $amount, string $period, ?string $feature = null, ?string $kind = null): UsageAllowance
    {
        return new UsageAllowance(1, 1, $feature, $kind, $unit, $amount, $period);
    }

    private function at(string $when): \DateTimeImmutable
    {
        return new \DateTimeImmutable($when, new \DateTimeZone('Europe/Paris'));
    }

    /**
     * 🔴 **Monday, the same Monday as everywhere else.** `BookingPolicyService`
     * counts `maxPerWeek` from `monday this week` and the opening hours are read
     * the same way. Two different week boundaries inside one booking path is a
     * fault that only shows up on a Sunday, to one member, once.
     */
    public function testAWeekStartsOnMonday(): void
    {
        // 2026-08-23 is a Sunday; its week began on Monday the 17th.
        [$from, $until] = $this->allowance(UsageAllowance::UNIT_MINUTES, 600, 'week')->windowAround($this->at('2026-08-23 19:00'));

        self::assertSame('2026-08-17 00:00', $from?->format('Y-m-d H:i'));
        self::assertSame('2026-08-24 00:00', $until?->format('Y-m-d H:i'));
    }

    public function testADayIsTheLocalCalendarDay(): void
    {
        [$from, $until] = $this->allowance(UsageAllowance::UNIT_BOOKINGS, 2, 'day')->windowAround($this->at('2026-08-19 23:30'));

        self::assertSame('2026-08-19 00:00', $from?->format('Y-m-d H:i'));
        self::assertSame('2026-08-20 00:00', $until?->format('Y-m-d H:i'));
    }

    public function testAMonthRunsFromTheFirst(): void
    {
        [$from, $until] = $this->allowance(UsageAllowance::UNIT_MINUTES, 1200, 'month')->windowAround($this->at('2026-08-31 08:00'));

        self::assertSame('2026-08-01 00:00', $from?->format('Y-m-d H:i'));
        self::assertSame('2026-09-01 00:00', $until?->format('Y-m-d H:i'));
    }

    /**
     * ⚠️ `total` is the absence of a period, not a long one. Unbounded both ways
     * is what makes a prepaid block of hours spend once and never reset.
     */
    public function testTotalHasNoBounds(): void
    {
        [$from, $until] = $this->allowance(UsageAllowance::UNIT_MINUTES, 3000, 'total')->windowAround($this->at('2026-08-19 10:00'));

        self::assertNull($from);
        self::assertNull($until);
    }

    /**
     * ⚠️ Null is "not restricted to", exactly as on a grant. An allowance naming
     * machines must say nothing about booking a room, or a lab metering machine
     * hours would silently meter its meeting rooms too.
     */
    public function testAnAllowanceOnlySpeaksAboutWhatItNames(): void
    {
        $machineHours = $this->allowance(UsageAllowance::UNIT_MINUTES, 600, 'week', 'machines', 'machine');

        self::assertTrue($machineHours->applies('machines', 'machine'));
        self::assertFalse($machineHours->applies('places', 'place'));
        self::assertFalse($machineHours->applies('machines', 'place'));

        $everything = $this->allowance(UsageAllowance::UNIT_MINUTES, 600, 'week');
        self::assertTrue($everything->applies('machines', 'machine'));
        self::assertTrue($everything->applies('places', 'place'));
        self::assertTrue($everything->applies(null, null));
    }

    /**
     * 🔴 **Two 5 h/week packages are 10 h, and a 5 h/week beside a 20 h/month are
     * two separate budgets that must both hold.** Summing the first pair is the
     * roadmap's "packages cumulatifs" rule; summing the second pair would let a
     * monthly cap silently pay for a weekly one.
     */
    public function testOnlyTheSameUnitPeriodAndScopeShareABudget(): void
    {
        $weeklyA = $this->allowance(UsageAllowance::UNIT_MINUTES, 300, 'week', 'machines', 'machine');
        $weeklyB = new UsageAllowance(2, 7, 'machines', 'machine', UsageAllowance::UNIT_MINUTES, 300, 'week');
        $monthly = $this->allowance(UsageAllowance::UNIT_MINUTES, 1200, 'month', 'machines', 'machine');
        $sessions = $this->allowance(UsageAllowance::UNIT_BOOKINGS, 3, 'week', 'machines', 'machine');

        self::assertSame($weeklyA->budgetKey(), $weeklyB->budgetKey(), 'Two packages selling the same weekly hours share one budget.');
        self::assertNotSame($weeklyA->budgetKey(), $monthly->budgetKey());
        self::assertNotSame($weeklyA->budgetKey(), $sessions->budgetKey());
    }

    public function testUnknownUnitsAndPeriodsAreRefused(): void
    {
        self::assertTrue(UsageAllowance::isValidUnit(UsageAllowance::UNIT_MINUTES));
        self::assertFalse(UsageAllowance::isValidUnit('euros'));
        self::assertTrue(UsageAllowance::isValidPeriod('total'));
        self::assertFalse(UsageAllowance::isValidPeriod('year'));
    }
}
