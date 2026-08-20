<?php

declare(strict_types=1);

namespace App\Tests\Schedule;

use App\Entity\ScheduleException;
use PHPUnit\Framework\TestCase;

/**
 * A dated closure can last more than one day (S146g).
 *
 * 🔴 **The operator's words: "cant do multi day closings? like week off".** A week
 * off used to be seven submissions of the same form, and then seven deletions to
 * undo. It is ONE row with a start and an end.
 *
 * ⚠️ **The opposite choice from event series (S146d), on purpose.** Sessions are
 * *occurrences* — each moves, fills and is cancelled on its own, so they must be
 * separate rows. A closure is a *single statement*; splitting it would make
 * "cancel the closure" a bulk delete.
 */
final class ScheduleExceptionSpanTest extends TestCase
{
    private function closure(string $from, ?string $to = null): ScheduleException
    {
        $row = (new ScheduleException())->setExceptionDate(new \DateTimeImmutable($from));

        return $row->setEndDate($to !== null ? new \DateTimeImmutable($to) : null);
    }

    public function testNoEndMeansOneDay(): void
    {
        $row = $this->closure('2026-12-25');

        self::assertNull($row->getEndDate());
        self::assertFalse($row->spansSeveralDays());
        self::assertSame(1, $row->dayCount());
        // ⚠️ Load-bearing: every row written before S146g has a null end, and every
        // reader falls back to this. A reader that forgets stops seeing all of them.
        self::assertSame('2026-12-25', $row->getLastDate()->format('Y-m-d'));
    }

    public function testAWeekOffIsOneRow(): void
    {
        $row = $this->closure('2026-08-10', '2026-08-16');

        self::assertTrue($row->spansSeveralDays());
        self::assertSame(7, $row->dayCount(), 'A week off is seven days, inclusive of both ends.');
        self::assertSame('2026-08-16', $row->getLastDate()->format('Y-m-d'));
    }

    /** ⚠️ An end before the start is not a range; it collapses to a single day. */
    public function testAnEndBeforeTheStartIsRefused(): void
    {
        $row = $this->closure('2026-08-10', '2026-08-03');

        self::assertNull($row->getEndDate());
        self::assertSame(1, $row->dayCount());
    }

    public function testAnEndEqualToTheStartIsStillOneDay(): void
    {
        self::assertFalse($this->closure('2026-08-10', '2026-08-10')->spansSeveralDays());
    }

    /**
     * 🔴 **The three queries must test the SPAN, not the start date.** A fortnight's
     * closure that began last week is still in force; one that runs from before a
     * calendar's window to after it covers every day of that window. A test on
     * `exceptionDate` alone misses both, silently.
     */
    public function testEveryQueryAsksAboutTheWholeSpan(): void
    {
        $repository = file_get_contents(__DIR__ . '/../../src/Repository/ScheduleExceptionRepository.php');

        self::assertStringContainsString(
            ':date BETWEEN e.exceptionDate AND COALESCE(e.endDate, e.exceptionDate)',
            $repository,
        );
        // Still in force = its LAST day has not passed.
        self::assertStringContainsString("COALESCE(e.endDate, e.exceptionDate) >= :from", $repository);
        // An overlap, not a containment.
        self::assertStringContainsString("->andWhere('e.exceptionDate <= :to')", $repository);
        self::assertStringNotContainsString("->andWhere('e.exceptionDate >= :from')", $repository);
    }

    /**
     * ⚠️ The surfaces want a map keyed by DATE, so a span is expanded — and clipped
     * to the window, or a fortnight's closure would put entries outside the week a
     * calendar drew.
     */
    public function testTheResolverExpandsASpanAcrossTheWindow(): void
    {
        $resolver = file_get_contents(__DIR__ . '/../../src/Schedule/ScheduleResolver.php');

        self::assertStringContainsString('$last = $exception->getLastDate()->setTime(0, 0);', $resolver);
        self::assertStringContainsString('while ($day <= $last && $day <= $windowEnd) {', $resolver);
        self::assertStringContainsString('if ($day < $windowStart) {', $resolver);
    }
}
