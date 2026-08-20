<?php

declare(strict_types=1);

namespace App\Tests\Calendar;

use App\Calendar\EventSeries;
use App\Entity\Event;
use PHPUnit\Framework\TestCase;

/**
 * "Every week, ×4" — four events, generated once (S146d).
 *
 * ⚠️ Pure unit tests: `EventSeries` touches no database, which is the point of it
 * being a class rather than a loop in the controller.
 */
final class EventSeriesTest extends TestCase
{
    private function event(string $start, ?string $end = null): Event
    {
        $event = (new Event())
            ->setTitre('Atelier découverte')
            ->setDateDebut(new \DateTimeImmutable($start))
            ->setLieu('FabLab - D173')
            ->setCapacite(12);

        return $end !== null ? $event->setDateFin(new \DateTimeImmutable($end)) : $event;
    }

    public function testNothingIsGeneratedWithoutARepeat(): void
    {
        $series = new EventSeries();

        self::assertSame([], $series->extraOccurrences($this->event('2026-09-03 14:00'), EventSeries::NONE, 4));
        self::assertSame([], $series->extraOccurrences($this->event('2026-09-03 14:00'), EventSeries::EVERY_WEEK, 1));
    }

    /** Four sessions means the operator's own plus THREE more. */
    public function testFourWeeklySessionsMeanThreeExtraEvents(): void
    {
        $extras = (new EventSeries())->extraOccurrences(
            $this->event('2026-09-03 14:00', '2026-09-03 17:00'),
            EventSeries::EVERY_WEEK,
            4,
        );

        self::assertCount(3, $extras);
        self::assertSame(
            ['2026-09-10 14:00', '2026-09-17 14:00', '2026-09-24 14:00'],
            array_map(static fn (Event $e): string => $e->getDateDebut()->format('Y-m-d H:i'), $extras),
        );
        self::assertSame(
            ['2026-09-10 17:00', '2026-09-17 17:00', '2026-09-24 17:00'],
            array_map(static fn (Event $e): string => $e->getDateFin()->format('Y-m-d H:i'), $extras),
            'The end shifts with the start, or session three runs for a week.',
        );
    }

    public function testEveryTwoWeeksSkipsAWeek(): void
    {
        $extras = (new EventSeries())->extraOccurrences(
            $this->event('2026-09-03 14:00'),
            EventSeries::EVERY_TWO_WEEKS,
            3,
        );

        self::assertSame(
            ['2026-09-17 14:00', '2026-10-01 14:00'],
            array_map(static fn (Event $e): string => $e->getDateDebut()->format('Y-m-d H:i'), $extras),
        );
    }

    /**
     * ⚠️ **Each occurrence is shifted from the FIRST date, never from the previous
     * one.** Stepping from each result accumulates drift, and across a DST change
     * "+1 week" from an already-shifted value is not the wall clock that was typed.
     */
    public function testOccurrencesAreShiftedFromTheOriginalNotFromEachOther(): void
    {
        // Europe/Paris leaves summer time on 2026-10-25.
        $extras = (new EventSeries())->extraOccurrences(
            $this->event('2026-10-15 14:00'),
            EventSeries::EVERY_WEEK,
            4,
        );

        self::assertSame(
            ['2026-10-22 14:00', '2026-10-29 14:00', '2026-11-05 14:00'],
            array_map(static fn (Event $e): string => $e->getDateDebut()->format('Y-m-d H:i'), $extras),
            'Every session stays at the hour the operator typed.',
        );
    }

    /** 🔴 A count arrives from a request; 9 999 events is not a preference. */
    public function testTheCountIsCapped(): void
    {
        $extras = (new EventSeries())->extraOccurrences($this->event('2026-09-03 14:00'), EventSeries::EVERY_WEEK, 9999);

        self::assertCount(EventSeries::MAX_OCCURRENCES - 1, $extras);
    }

    /** ⚠️ Everything that describes the event carries over — except the poster file. */
    public function testTheOccurrencesCarryTheEventTheyCopy(): void
    {
        $first = $this->event('2026-09-03 14:00')->setPosterFilename('affiche.jpg');
        $extras = (new EventSeries())->extraOccurrences($first, EventSeries::EVERY_WEEK, 2);

        self::assertSame('Atelier découverte', $extras[0]->getTitre());
        self::assertSame('FabLab - D173', $extras[0]->getLieu());
        self::assertSame(12, $extras[0]->getCapacite());
        // One file, one row: sharing the name would break the picture on the others
        // the day somebody deletes one event.
        self::assertNull($extras[0]->getPosterFilename());
    }

    /**
     * 🔴 **A field added to `EventAdminType` draws nothing until the template names
     * it.** Both event forms render row by row with no `form_rest()`, which is how
     * S146f's category and formation fields shipped invisible.
     */
    public function testEveryEventFormFieldIsActuallyRendered(): void
    {
        $type = file_get_contents(__DIR__ . '/../../src/Form/EventAdminType.php');
        preg_match_all("/->add\('([a-zA-Z]+)'/", $type, $matches);
        $fields = array_diff($matches[1], ['save']);

        $new = file_get_contents(__DIR__ . '/../../templates/site/admin-event-new.html.twig');
        $edit = file_get_contents(__DIR__ . '/../../templates/site/admin-event-edit.html.twig');

        foreach ($fields as $field) {
            self::assertStringContainsString('form.' . $field, $new, sprintf('%s is missing from the creation form.', $field));

            // The series controls exist only on creation, by design.
            if (in_array($field, ['repeatEvery', 'repeatCount'], true)) {
                self::assertStringNotContainsString('form.' . $field, $edit, 'Editing must not offer to generate more events.');
                continue;
            }

            self::assertStringContainsString('form.' . $field, $edit, sprintf('%s is missing from the edit form.', $field));
        }
    }
}
