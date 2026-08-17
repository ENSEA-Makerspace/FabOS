<?php

declare(strict_types=1);

namespace App\UsageRights;

/**
 * Does a grant's set of weekly windows cover a whole booking? (S144b)
 *
 * 🔴 **Cover, not overlap — and the difference is the entire feature.** A package
 * that opens Monday 14:00–18:00 must refuse a booking of Monday 17:00–22:00. An
 * overlap test would allow it, and the lab would discover the rule it thought it
 * had sold does not exist. So the requested interval is walked day by day and
 * every minute of it has to fall inside the union of that day's windows.
 *
 * ⚠️ **The union matters too.** Two windows 09:00–12:00 and 12:00–14:00 cover a
 * booking from 11:00 to 13:00 even though neither contains it. Labs write their
 * opening hours in pieces (morning, afternoon) and would reasonably expect the
 * two to join up.
 *
 * ⚠️ **No windows at all means no time restriction**, which is what every grant
 * written before S144b has, and what keeps the migration expand-only: an install
 * that has not added a single window behaves exactly as it did.
 *
 * Pure arithmetic on wall-clock minutes, deliberately: it is the one part of the
 * grant model that can be unit-tested without a database, and the part where an
 * off-by-one is invisible in a code review.
 */
final class GrantWindowSet
{
    /**
     * @param list<GrantWindow> $windows every window of ONE grant
     */
    public static function covers(array $windows, \DateTimeImmutable $from, \DateTimeImmutable $until): bool
    {
        if ($windows === []) {
            return true;
        }
        if ($until <= $from) {
            return false;
        }

        $byDay = [];
        foreach ($windows as $window) {
            $byDay[$window->dayOfWeek][] = $window;
        }

        $cursor = $from;
        // A booking may run past midnight, and each calendar day it touches is a
        // separate question: Monday's windows say nothing about Tuesday morning.
        while ($cursor < $until) {
            $nextDay = $cursor->setTime(0, 0)->modify('+1 day');
            $segmentEnd = $until < $nextDay ? $until : $nextDay;

            $startMinute = self::minuteOfDay($cursor);
            // ⚠️ Midnight at the END of a segment is minute 1440, not 0. Reading
            // `format('H:i')` there would give 00:00 and turn a full evening into
            // a backwards interval that nothing can cover.
            $endMinute = $segmentEnd === $nextDay
                ? GrantWindow::MINUTES_IN_DAY
                : self::minuteOfDay($segmentEnd);

            if (!self::dayCovers($byDay[(int) $cursor->format('N')] ?? [], $startMinute, $endMinute)) {
                return false;
            }

            $cursor = $nextDay;
        }

        return true;
    }

    /**
     * @param list<GrantWindow> $windows the windows of one weekday
     */
    private static function dayCovers(array $windows, int $startMinute, int $endMinute): bool
    {
        // An empty segment asks nothing and is covered vacuously; a weekday with
        // no window is simply closed.
        if ($endMinute <= $startMinute) {
            return true;
        }
        if ($windows === []) {
            return false;
        }

        usort($windows, static fn (GrantWindow $a, GrantWindow $b): int => $a->startMinute <=> $b->startMinute);

        $reached = $startMinute;
        foreach ($windows as $window) {
            if ($window->startMinute > $reached) {
                // A gap opens before this window: the union is broken and nothing
                // later can close it, because the list is sorted by start.
                break;
            }
            if ($window->endMinute > $reached) {
                $reached = $window->endMinute;
            }
            if ($reached >= $endMinute) {
                return true;
            }
        }

        return $reached >= $endMinute;
    }

    private static function minuteOfDay(\DateTimeImmutable $moment): int
    {
        return ((int) $moment->format('H')) * 60 + (int) $moment->format('i');
    }
}
