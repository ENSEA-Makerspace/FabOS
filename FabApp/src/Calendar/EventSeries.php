<?php

declare(strict_types=1);

namespace App\Calendar;

use App\Entity\Event;

/**
 * "Every week, ×4" — four events, generated once, at creation (S146d).
 *
 * 🔴 **This is deliberately NOT a recurrence engine.** The roadmap's words: a
 * weekly course running for a month is *four events generated at creation*, not a
 * rule evaluated at read time. It is simpler, and it is more honest — each session
 * moves, fills up or gets called off **individually**, with its own registrations
 * and its own cancellation reason, and a rule cannot express any of that. A stored
 * rule would also have to be re-evaluated by every surface that lists events, which
 * is the duplication S146a spent a whole step removing.
 *
 * ⚠️ **Weeks only, and that is a decision.** `+1 month` on the 31st lands on the 3rd
 * of the month after next, so a monthly series would have to answer "what is the
 * monthly repeat of 31 January" before it could be offered. Weeks have no such
 * question. Adding months later means making that call explicitly, not widening a
 * constant.
 *
 * ⚠️ **The generated events are independent rows, not children.** Nothing links them
 * back to each other: there is no series id, and editing or deleting one does not
 * touch the others. That is the point — if they were a series, "move the third
 * session" would need an exception model, and that is the engine this avoids.
 */
final class EventSeries
{
    public const NONE = 'none';
    public const EVERY_WEEK = 'week';
    public const EVERY_TWO_WEEKS = 'two_weeks';

    /**
     * ⚠️ A cap, not a preference. The form offers up to 12; this refuses more, because
     * the count arrives from a request and 9 999 events is a denial of service typed
     * into a number field.
     */
    public const MAX_OCCURRENCES = 12;

    /**
     * The events to create BESIDES the one the operator filled in.
     *
     * @return Event[] empty when nothing repeats — the caller always persists its own
     */
    public function extraOccurrences(Event $first, string $every, int $count): array
    {
        $interval = match ($every) {
            self::EVERY_WEEK => '+1 week',
            self::EVERY_TWO_WEEKS => '+2 weeks',
            default => null,
        };

        $start = $first->getDateDebut();
        if ($interval === null || $start === null) {
            return [];
        }

        $count = min(max($count, 1), self::MAX_OCCURRENCES);
        $end = $first->getDateFin();
        $extras = [];

        for ($n = 1; $n < $count; $n++) {
            // ⚠️ Each occurrence is shifted from the FIRST date, never from the
            // previous one: stepping repeatedly would accumulate any drift, and
            // across a DST boundary "+1 week" from a shifted date is not the same
            // wall-clock time the operator typed.
            $weeks = $every === self::EVERY_TWO_WEEKS ? 2 * $n : $n;
            $shift = sprintf('+%d weeks', $weeks);

            $occurrence = (new Event())
                ->setTitre($first->getTitre())
                ->setDescription($first->getDescription())
                ->setDateDebut($start->modify($shift))
                ->setDateFin($end?->modify($shift))
                ->setLieu($first->getLieu())
                ->setVenue($first->getVenue())
                ->setCapacite($first->getCapacite())
                ->setGuestsAllowed($first->isGuestsAllowed())
                ->setLocationMode($first->getLocationMode())
                ->setAddress($first->getAddress())
                ->setCategory($first->getCategory())
                ->setFormation($first->getFormation());

            // ⚠️ The poster is NOT copied. The file belongs to one row: sharing the
            // filename would make deleting one event break the picture on the others,
            // and `AccountAnonymiser` already taught this codebase what shared files
            // on disk cost.
            $extras[] = $occurrence;
        }

        return $extras;
    }
}
