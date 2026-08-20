<?php

namespace App\Tests\Schedule;

use PHPUnit\Framework\TestCase;

/**
 * The opening hours of the RIGHT location.
 *
 * 🔴 **The fault.** `OpeningHoursProvider` resolved the week by calling
 * `VenueRepository::findDefault()` — the venue whose slug is literally
 * `default` — and read nothing else. `ReservationService` validated every
 * booking through it with no location at all. So on a multi-location install a
 * machine in the second room was checked against the first room's hours, and an
 * install with no `slug = 'default'` venue had every booking checked against
 * seven rows hard-coded in a PHP constant.
 *
 * ⚠️ **This is the third time.** A venue-scoped grant did nothing for two
 * sessions (S134b) and `USAGE_RIGHT_ASSIGNMENT.groupId` was written by nothing
 * for two more (S144a) — both times the schema had the dimension and the caller
 * never supplied it. The lesson this file pins is the general one: **when a
 * table carries `venueId`, ask whether the CALLER passes a value, not whether
 * the query joins the column.**
 *
 * Source-text, like `VenueScopedGrantTest`, and for the same reason: what broke
 * was wiring, not SQL. Exercising it for real needs two locations, two sets of
 * hours, a machine in each and a booking — and the thing that was wrong would
 * survive all of it, because every line was correct except that one argument was
 * never supplied.
 */
final class ScheduleResolverWiringTest extends TestCase
{
    private const RESOLVER = __DIR__ . '/../../src/Schedule/ScheduleResolver.php';
    private const BOOKING = __DIR__ . '/../../src/Reservation/ReservationService.php';
    private const SLOTS = __DIR__ . '/../../src/Reservation/NextFreeSlotService.php';
    private const KIOSK = __DIR__ . '/../../src/Controller/KioskController.php';

    public function testEveryAnswerIsAskedOfALocation(): void
    {
        $source = file_get_contents(self::RESOLVER);

        foreach ([
            'public function rowsFor(?int $venueId, ?string $scopeType = null, ?int $scopeId = null)',
            'public function refusalFor(?int $venueId, \DateTimeImmutable $from, \DateTimeImmutable $until, ?string $scopeType = null, ?int $scopeId = null)',
            'public function openMinutesFor(?int $venueId, \DateTimeInterface $date, ?string $scopeType = null, ?int $scopeId = null)',
            'public function openIntervalsFor(?int $venueId, \DateTimeInterface $date, ?string $scopeType = null, ?int $scopeId = null)',
            'public function calendarStartHour(?int $venueId)',
            'public function calendarEndHour(?int $venueId)',
            'public function forJson(?int $venueId)',
        ] as $signature) {
            self::assertStringContainsString(
                $signature,
                $source,
                'Every question the resolver answers must name the location it is answering for.',
            );
        }
    }

    /** 🔴 The exact regression: the booking gate asked without saying where. */
    public function testTheBookingGateSuppliesTheResourcesLocation(): void
    {
        $source = file_get_contents(self::BOOKING);

        self::assertStringContainsString(
            '$venueId = $this->reservables->venueIdFor($type, $id);',
            $source,
            'The booking path must resolve the location of the thing being booked.',
        );
        // ⚠️ Location AND resource: a machine may carry its own week below its
        // location's, and levels intersect, so passing the resource can only ever
        // narrow the answer.
        self::assertStringContainsString(
            '$this->schedule->refusalFor($venueId, $start, $end, $type->value, $id)',
            $source,
            'and hand it to the schedule, or the hours of somewhere else decide.',
        );
        // ⚠️ Comments stripped first, the same way `VenueScopedGrantTest`
        // checks there is one grant table. The dead method is NAMED in a comment
        // here on purpose — recording what was wrong is how the next session
        // avoids rebuilding it — so the test has to forbid the call without
        // forbidding the sentence about the call.
        self::assertStringNotContainsString(
            'validateReservationPeriod(',
            preg_replace('/^\s*(\*|\/\/).*$/m', '', $source) ?? '',
            'The venue-blind entry point is gone and must not come back as a CALL.',
        );
    }

    /**
     * A proposed slot that the booking gate would then refuse is worse than no
     * proposal, because it reads as an invitation.
     */
    public function testTheSlotProposerAgreesWithTheGate(): void
    {
        $source = file_get_contents(self::SLOTS);

        self::assertStringContainsString('$venueId = $this->reservables->venueIdFor($type, $id);', $source);
        // ⚠️ Intervals, not the envelope (S134d). Walking 09:00–18:00 on a day
        // that shuts for lunch proposes the closed hour, and the gate then
        // refuses the slot this method invited somebody to take.
        self::assertStringContainsString('$this->schedule->openIntervalsFor($venueId, $day, $type->value, $id)', $source);
        self::assertStringNotContainsString(
            'openMinutesFor($venueId, $day)',
            preg_replace('/^\s*(\*|\/\/).*$/m', '', $source) ?? '',
            'The envelope must not decide which slots are proposed.',
        );
    }

    /** A kiosk is bolted to a wall in one room. */
    public function testTheKioskShowsItsOwnRoom(): void
    {
        self::assertStringContainsString(
            '$schedule->forJson($machine->getVenue()?->getId())',
            file_get_contents(self::KIOSK),
        );
    }

    /**
     * ⚠️ Null must stay CONSTRAINED, and that is the opposite of the rule for
     * grants. A grant is permission and an unanswerable restriction must not
     * refuse; opening hours are a restriction, so an unanswerable one must not
     * *permit*. Making null mean "no limit" here would quietly open every lab
     * that books people rather than machines.
     */
    public function testAnUnknownLocationStaysBoundByTheDefaultVenue(): void
    {
        $source = file_get_contents(self::RESOLVER);

        self::assertStringContainsString('defaultVenueRows(', $source);
        self::assertStringContainsString(
            'findDefault()',
            $source,
            'The fallback ladder ends at the default venue, which is what every location silently used before.',
        );
    }

    /**
     * ⚠️ The model half is NOT done: `UNIQ_OPENING_HOUR_VENUE_DAY` still forbids
     * a second range on a day, so a lunch break remains inexpressible. This test
     * exists so the next session does not read "ScheduleResolver exists" as
     * "S134d is finished".
     */
    /**
     * 🔴 **The envelope and the ranges must never be read from two places.**
     * `openMinutesFor()` survives for LAYOUT — a calendar has to know which rows
     * of the grid to draw — and it is derived from the intervals rather than from
     * a second pass over the table, so the two cannot disagree.
     */
    public function testTheEnvelopeIsDerivedFromTheIntervals(): void
    {
        $source = file_get_contents(self::RESOLVER);

        self::assertStringContainsString('public function openIntervalsFor(?int $venueId', $source);
        self::assertStringContainsString('$intervals = $this->openIntervalsFor($venueId, $date, $scopeType, $scopeId);', $source);
        self::assertStringContainsString('public function closureReasonFor(?int $venueId', $source);
    }

    /**
     * 🔴 **A level may only NARROW its location's hours** (operator's decision,
     * 2026-08-19). Intersecting is the only composition in which no level can
     * fail open: nobody uses the laser cutter while the building is locked. If
     * this ever became "most specific replaces", a sub-schedule could open a
     * resource in a shut building and nothing in the model would object.
     */
    public function testLevelsIntersectAndCannotWiden(): void
    {
        $source = file_get_contents(self::RESOLVER);

        self::assertStringContainsString('private function intersect(array $outer, array $inner): array', $source);
        self::assertStringContainsString('return $this->intersect($venueIntervals, $this->intervalsFromRows($narrower, $date));', $source);
        self::assertStringContainsString(
            'if ($scopeType === null || $venueIntervals === [])',
            $source,
            'A shut location must short-circuit: no level below it can open anything.',
        );
    }

    /**
     * ⚠️ **One level answers; they do not stack.** If a machine has its own week,
     * its kind's week is not also applied — otherwise narrowing "all machines"
     * would silently narrow the one machine somebody had just given wider hours,
     * and no screen could explain the result.
     */
    public function testOneLevelAnswers(): void
    {
        self::assertStringContainsString(
            'private function narrowestRowsFor(?int $venueId, string $scopeType, ?int $scopeId): array',
            file_get_contents(self::RESOLVER),
        );
    }

    /**
     * ⚠️ The price of intersecting is that hours written wider than the location
     * do nothing — and hours that silently do nothing are the fault this codebase
     * hit three times in one week. The editor must resolve and show them.
     */
    public function testTheEditorShowsWhatTheHoursActuallyDo(): void
    {
        self::assertStringContainsString(
            'private function effectiveWeek(',
            file_get_contents(__DIR__ . '/../../src/Controller/AdminController.php'),
        );
        self::assertStringContainsString(
            'hours.effective_none',
            file_get_contents(__DIR__ . '/../../templates/site/admin-opening-hours.html.twig'),
            'A range with no effect has to say so on screen.',
        );
    }

    /**
     * 🔴 A booking must fit inside ONE range. With a lunch break, 11:00–15:00 is
     * inside the envelope and is still an hour of booking the shut lab — testing
     * the envelope would make the whole feature decorative on day one.
     */
    public function testABookingMustFitInsideOneRange(): void
    {
        self::assertStringContainsString(
            'if ($startMinute >= $interval[\'start\'] && $endMinute <= $interval[\'end\'])',
            file_get_contents(self::RESOLVER),
            'The refusal looks for a containing interval, never the outer envelope.',
        );
    }

    /**
     * 🔴 **"Unconfigured" means NO rows, and it must mean that in BOTH places.**
     * The fallback ladder tested `count($rows) === 7`, which stopped being true
     * the moment a day held two ranges. It was fixed in `rowsFor()` and missed in
     * `defaultVenueRows()`, where it was worse: every caller that asks without a
     * location — the homepage, the aggregated calendar, the public API, the
     * seeding of a new location, and the availability of every PERSON booking —
     * silently got the built-in 08:00–20:00 week instead of the lab's real hours.
     * Found by a log line during a self-test whose assertions all passed.
     */
    public function testAWeekIsNoLongerSevenRows(): void
    {
        $source = file_get_contents(self::RESOLVER);

        self::assertStringNotContainsString(
            'count($rows) === 7',
            preg_replace('/^\s*(\*|\/\/).*$/m', '', $source) ?? '',
            'A seven-row test throws away every week that uses several ranges in a day.',
        );
        self::assertStringNotContainsString('count($rows) !== 7', preg_replace('/^\s*(\*|\/\/).*$/m', '', $source) ?? '');
    }

    /**
     * ⚠️ Both calendars build their lookup with `hours[row.dayIndex] = …`, so one
     * entry per ROW meant a second range silently overwrote the first and the
     * afternoon vanished from the grid. One entry per DAY, carrying its ranges.
     */
    public function testTheCalendarsAreGivenRangesAndNotJustAnEnvelope(): void
    {
        self::assertStringContainsString("'ranges' => array_map(", file_get_contents(self::RESOLVER));

        foreach ([
            __DIR__ . '/../../templates/site/calendrier.html.twig',
            __DIR__ . '/../../templates/site/machine-calendrier.html.twig',
        ] as $calendar) {
            $source = file_get_contents($calendar);
            self::assertStringContainsString('const FABLAB_RANGES', $source, 'Each calendar needs the ranges, not only the envelope.');
            // ⚠️ S134e widened this call: the date decides too, because a dated
            // exception beats the weekday.
            self::assertStringContainsString('if (!isMinuteOpen(dayIndex, slotStart, date))', $source, 'and the slot state has to be decided by them.');
        }
    }

    /**
     * 🔴 **The closure REASON reached nothing but a booking refusal** until
     * S134e. The model had computed it since S134d, and a member looking at a
     * calendar on a public holiday saw "closed" and was left to guess whether the
     * lab was shut, broken, or whether they had misread the page. Same shape as
     * the three faults of that week — everything stored, nothing wired to a
     * screen — one layer up.
     */
    public function testTheClosureReasonReachesTheSurfaces(): void
    {
        $site = file_get_contents(__DIR__ . '/../../src/Controller/SiteController.php');
        self::assertStringContainsString('$schedule->closureReasonFor(', $site, 'The catalogues must ask for the reason.');
        self::assertStringContainsString("'venueClosureReason' => \$venueClosureReason,", $site);

        self::assertStringContainsString(
            "\$schedule->dayStatus(",
            file_get_contents(__DIR__ . '/../../src/Controller/KioskController.php'),
            'A kiosk is read by somebody standing at a locked door.',
        );

        foreach ([
            __DIR__ . '/../../templates/site/calendrier.html.twig',
            __DIR__ . '/../../templates/site/machine-calendrier.html.twig',
        ] as $calendar) {
            $source = file_get_contents($calendar);
            self::assertStringContainsString('const SCHEDULE_EXCEPTIONS', $source);
            // ⚠️ A dated exception REPLACES the weekday, client-side exactly as
            // it does on the server — keeping the two rules identical is what
            // makes the calendar and the booking gate agree about a holiday.
            self::assertStringContainsString('if (exception && exception.closed)', $source);
        }

        foreach ([
            __DIR__ . '/../../templates/site/machines.html.twig',
            __DIR__ . '/../../templates/site/places.html.twig',
        ] as $catalogue) {
            self::assertStringContainsString('venueClosureReason', file_get_contents($catalogue));
        }
    }

    /**
     * 🔴 **`$venueOpenNow` was computed BEFORE `$venueContext` existed** on
     * `/places`, so the location filter was ignored and the page answered for the
     * default venue. Prod runs without `strict_variables`, so an undefined
     * variable is silently null and nothing said a word — it took a warning in a
     * self-test to surface it. This pins the order.
     */
    public function testThePlacesCatalogueResolvesItsLocationFirst(): void
    {
        $source = file_get_contents(__DIR__ . '/../../src/Controller/SiteController.php');
        $assignment = strpos($source, '$venueContext = $venues->forRequest($request, $this->getUser() instanceof Utilisateur ? $this->getUser() : null);');
        $use = strpos($source, '$venueOpenNow = $schedule->isOpenAt($venueContext[\'selected\']?->getId(), $now);', (int) $assignment);

        self::assertNotFalse($assignment);
        self::assertNotFalse($use);
        self::assertGreaterThan($assignment, $use, 'The location must be resolved before anything asks it a question.');
    }
}
