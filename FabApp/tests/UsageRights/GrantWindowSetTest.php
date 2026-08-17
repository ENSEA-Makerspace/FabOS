<?php

namespace App\Tests\UsageRights;

use App\UsageRights\GrantWindow;
use App\UsageRights\GrantWindowSet;
use PHPUnit\Framework\TestCase;

/**
 * "3D print on Monday afternoons" — and what that must refuse.
 *
 * 🔴 **The mistake this file exists to prevent is an overlap test.** A grant open
 * Monday 14:00–18:00 that admits a Monday 17:00–22:00 booking because the two
 * *touch* would be worse than having no windows at all: the lab would believe it
 * had sold restricted access and would have sold unrestricted access with extra
 * steps. Coverage of the whole interval is the rule, and every case below is a
 * way of getting that wrong.
 *
 * A real timezone throughout, because the arithmetic is wall-clock and the bugs
 * live at midnight.
 */
final class GrantWindowSetTest extends TestCase
{
    private const ZONE = 'Europe/Paris';

    /** Monday 2026-08-17 is the reference week used below. */
    private function at(string $when): \DateTimeImmutable
    {
        return new \DateTimeImmutable($when, new \DateTimeZone(self::ZONE));
    }

    /** ⚠️ No windows is not "closed" — it is every grant written before S144b. */
    public function testAGrantWithNoWindowsIsAwakeAllWeek(): void
    {
        self::assertTrue(GrantWindowSet::covers([], $this->at('2026-08-19 03:00'), $this->at('2026-08-19 05:00')));
    }

    public function testAMondayAfternoonGrantCoversAMondayAfternoonBooking(): void
    {
        self::assertTrue(GrantWindowSet::covers(
            [GrantWindow::fromClock(1, '14:00', '18:00')],
            $this->at('2026-08-17 14:00'),
            $this->at('2026-08-17 16:00'),
        ));
    }

    /** 🔴 The whole feature: overlapping the window is not being inside it. */
    public function testABookingThatRunsPastTheWindowIsRefused(): void
    {
        self::assertFalse(GrantWindowSet::covers(
            [GrantWindow::fromClock(1, '14:00', '18:00')],
            $this->at('2026-08-17 17:00'),
            $this->at('2026-08-17 22:00'),
        ));
    }

    public function testTheSameHoursOnAnotherDayAreRefused(): void
    {
        self::assertFalse(GrantWindowSet::covers(
            [GrantWindow::fromClock(1, '14:00', '18:00')],
            $this->at('2026-08-18 14:00'),
            $this->at('2026-08-18 16:00'),
        ));
    }

    /**
     * ⚠️ Labs write opening hours in pieces — morning, afternoon — and expect two
     * touching slices to behave as one. Neither window contains this booking.
     */
    public function testTwoAdjacentWindowsJoinUp(): void
    {
        self::assertTrue(GrantWindowSet::covers(
            [GrantWindow::fromClock(2, '09:00', '12:00'), GrantWindow::fromClock(2, '12:00', '14:00')],
            $this->at('2026-08-18 11:00'),
            $this->at('2026-08-18 13:00'),
        ));
    }

    /** And a real gap between them is a gap, even though both sides are open. */
    public function testAGapBetweenTwoWindowsIsNotCovered(): void
    {
        self::assertFalse(GrantWindowSet::covers(
            [GrantWindow::fromClock(2, '09:00', '11:00'), GrantWindow::fromClock(2, '12:00', '14:00')],
            $this->at('2026-08-18 10:00'),
            $this->at('2026-08-18 13:00'),
        ));
    }

    /**
     * ⚠️ Midnight at the end of a day is minute 1440 and not minute 0. Read the
     * other way, an evening booking becomes a backwards interval that nothing can
     * cover, and a lab open until midnight can never sell an evening.
     */
    public function testABookingEndingAtMidnightIsCoveredByAWindowEndingAtMidnight(): void
    {
        self::assertTrue(GrantWindowSet::covers(
            [GrantWindow::fromClock(1, '20:00', '00:00')],
            $this->at('2026-08-17 22:00'),
            $this->at('2026-08-18 00:00'),
        ));
    }

    public function testABookingAcrossMidnightNeedsBothDaysOpen(): void
    {
        $mondayEvening = GrantWindow::fromClock(1, '22:00', '24:00');
        $tuesdayNight = GrantWindow::fromClock(2, '00:00', '02:00');

        self::assertTrue(GrantWindowSet::covers(
            [$mondayEvening, $tuesdayNight],
            $this->at('2026-08-17 22:00'),
            $this->at('2026-08-18 01:00'),
        ));

        // Tuesday closed: the same booking must now be refused, and refused
        // because of the half nobody thinks about.
        self::assertFalse(GrantWindowSet::covers(
            [$mondayEvening],
            $this->at('2026-08-17 22:00'),
            $this->at('2026-08-18 01:00'),
        ));
    }

    public function testTheBoundsThemselvesAreInside(): void
    {
        self::assertTrue(GrantWindowSet::covers(
            [GrantWindow::fromClock(1, '14:00', '18:00')],
            $this->at('2026-08-17 14:00'),
            $this->at('2026-08-17 18:00'),
        ));
    }

    /**
     * ⚠️ A window whose end is the browser's "00:00" means the end of the day.
     * Read as minute zero it would be invalid, and an operator would be told
     * their opening hours are wrong for typing what they meant.
     */
    public function testMidnightAsAnEndIsTheEndOfTheDay(): void
    {
        $window = GrantWindow::fromClock(5, '18:00', '00:00');

        self::assertSame(GrantWindow::MINUTES_IN_DAY, $window->endMinute);
        self::assertTrue($window->isValid());
    }

    public function testABackwardsWindowIsInvalid(): void
    {
        self::assertFalse(GrantWindow::fromClock(1, '18:00', '14:00')->isValid());
        self::assertFalse((new GrantWindow(0, 0, 60))->isValid(), 'Day 0 is not an ISO weekday.');
        self::assertFalse((new GrantWindow(8, 0, 60))->isValid());
    }
}
