<?php

declare(strict_types=1);

namespace App\Tests\Calendar;

use PHPUnit\Framework\TestCase;

/**
 * There is ONE calendar (S146a).
 *
 * 🔴 **The measurement this pins.** Before S146a, `calendrier.html.twig` (664
 * lines) and `machine-calendrier.html.twig` (288) each carried their own copy of
 * the same twelve JavaScript functions, and in the single session S134d/S134e the
 * same opening-hours rule had to be edited in both files four times over. A
 * calendar that disagrees with the booking gate about when the lab is open is the
 * exact fault the schedule model was rebuilt to prevent — so "there are two
 * calendars" is not an aesthetic complaint, it is the next silent divergence.
 *
 * ⚠️ These are source-text assertions and they cannot see whether the page
 * *renders*. That is the route sweep's job. What they can see is the shape coming
 * back — which is what actually happened here twice before.
 */
final class CalendarComponentTest extends TestCase
{
    private const COMPONENT = __DIR__ . '/../../assets/controllers/calendar_controller.js';
    private const PAYLOAD = __DIR__ . '/../../src/Calendar/CalendarPayload.php';

    /**
     * ⚠️ **`machine-calendrier.html.twig` is GONE since S146b** — the machine's week
     * is a tab on `/machines/{id}` and the old URL 301s there. The page that draws
     * the calendar is the machine's own detail page now.
     */
    private const PAGES = [
        __DIR__ . '/../../templates/site/calendrier.html.twig',
        __DIR__ . '/../../templates/site/machine-detail.html.twig',
    ];

    /** The names that existed twice, verbatim, in the two templates. */
    private const DUPLICATED = [
        'timeToMinutes',
        'formatDateKey',
        'getWeekDays',
        'isMinuteOpen',
        'getSlotState',
        'exceptionFor',
        'buildTimeOptions',
        'overlapsHour',
        'reservationCardHtml',
        'getOpeningHoursLabel',
        'findNextOpenSlotLabel',
        'getWeekNumber',
    ];

    public function testNeitherCalendarPageCarriesCalendarLogicOfItsOwn(): void
    {
        foreach (self::PAGES as $page) {
            $source = file_get_contents($page);

            foreach (self::DUPLICATED as $name) {
                self::assertStringNotContainsString(
                    $name,
                    $source,
                    sprintf('%s is back in %s — that is the duplication S146a removed.', $name, basename($page)),
                );
            }

            // ⚠️ Only the aggregated calendar is held to "no JavaScript at all".
            // `machine-detail.html.twig` has its own tab strip and favourite button,
            // which are its business and not the calendar's — the twelve names above
            // are what must never come back.
            if (str_contains($page, 'calendrier.html.twig')) {
                self::assertStringNotContainsString(
                    'function ',
                    $source,
                    sprintf('%s defines JavaScript again; it belongs in the calendar controller.', basename($page)),
                );
            }
        }
    }

    public function testBothCalendarPagesUseTheOneComponent(): void
    {
        foreach (self::PAGES as $page) {
            $source = file_get_contents($page);
            self::assertStringContainsString('data-controller="calendar"', $source, basename($page));
            self::assertStringContainsString('data-calendar-payload-value="{{ calendar|json_encode }}"', $source, basename($page));
            self::assertStringContainsString("include 'site/_calendar.html.twig'", $source, basename($page));
            self::assertStringContainsString("include 'site/_calendar_booking.html.twig'", $source, basename($page));
        }
    }

    /**
     * ⚠️ **Both views live in the shared component or they get written twice.** The
     * month view is the first thing added to a calendar since it became one; if it
     * had landed before the extraction it would already exist in two copies.
     */
    public function testTheComponentDrawsBothAWeekAndAMonth(): void
    {
        $source = file_get_contents(self::COMPONENT);

        self::assertStringContainsString('renderWeek()', $source);
        self::assertStringContainsString('renderMonth()', $source);
        self::assertStringContainsString("this.view === 'month'", $source);
    }

    /**
     * 🔴 **A booking's identity rules are the server's, and were written out twice
     * in Twig.** `user` only for an entitled viewer, `motif` only for the booker or
     * an entitled viewer, `url` only for your own booking — `/reservations/{id}`
     * answers 404 to anyone else — and never the raw `user_id` (S38).
     */
    public function testTheIdentityRulesAreDecidedOnceOnTheServer(): void
    {
        $payload = file_get_contents(self::PAYLOAD);

        self::assertStringContainsString('$showName = $this->identity->canSeeOthersIdentity();', $payload);
        self::assertStringContainsString("'user' => \$showName && \$owner !== null ? \$owner->getDisplayName() : null,", $payload);
        self::assertStringContainsString("'motif' => (\$showName || \$mine) && \$motif ? \$motif : null,", $payload);
        // ⚠️ Comments stripped first: this file EXPLAINS that `user_id` is never
        // emitted, so a naive search finds the prose and fails on the fix.
        $code = preg_replace('#^\s*(//|\*|/\*).*$#m', '', $payload) ?? '';
        self::assertStringNotContainsString('user_id', $code, "A booking's owner id belongs to nobody but its owner.");

        foreach (self::PAGES as $page) {
            self::assertStringNotContainsString(
                'showBookerIdentity',
                file_get_contents($page),
                sprintf('%s decides an identity rule again.', basename($page)),
            );
        }
    }

    /**
     * ⚠️ Every user-visible string the component writes comes from the catalogues,
     * through the payload. S134c had to clear twenty French literals out of these
     * two `<script>` blocks on a site that ships five languages.
     */
    public function testTheComponentWritesNoStringOfItsOwn(): void
    {
        $source = preg_replace('#^\s*(//|\*|/\*).*$#m', '', file_get_contents(self::COMPONENT)) ?? '';

        // Accented letters are the cheap tell: any French left in the file is a
        // string a translator cannot reach.
        self::assertSame(
            0,
            preg_match('/[éèêàùçôîœ]/u', $source),
            'The calendar controller holds a literal word; it belongs in messages.*.yaml and in CalendarPayload::labels().',
        );
    }

    /**
     * ⚠️ **The location filter is a server-rendered link, not a client toggle.** The
     * opening hours, the dated exceptions and the bookable resources all change with
     * the location and all come from the server; a client-side toggle would draw one
     * location's week over another location's machines.
     */
    public function testTheAggregatedCalendarAsksWhichLocation(): void
    {
        $controller = file_get_contents(__DIR__ . '/../../src/Controller/SiteController.php');
        // ⚠️ Sliced to the NEXT route attribute, not a byte count. A fixed window
        // silently stops covering the action the moment a comment is added to it,
        // and a test that stops looking is worse than no test.
        $start = (int) strpos($controller, "#[Route('/calendrier', name: 'app_calendar'");
        $next = strpos($controller, '#[Route(', $start + 10);
        $calendar = substr($controller, $start, $next === false ? null : $next - $start);

        self::assertStringContainsString('$venues->forRequest($request, $member)', $calendar);
        self::assertStringContainsString("\$scope = \$venue !== null ? ['venue' => \$venue] : [];", $calendar);
        self::assertStringContainsString('$machines->findBy($scope', $calendar, 'The resources are the location\'s, or the filter shows the wrong week over them.');
        self::assertStringContainsString('$calendarPayload->build(', $calendar);
        self::assertStringContainsString('$venue?->getId()', $calendar, 'and the schedule must be asked for THAT location.');
    }
}
