<?php

declare(strict_types=1);

namespace App\Calendar;

use App\Entity\Event;
use App\Entity\Reservation;
use App\Reservation\ReservableResolver;
use App\Schedule\ScheduleResolver;
use App\Service\BookingIdentityPolicy;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * Everything the one calendar component needs, built once (S146a).
 *
 * 🔴 **This is the server half of the same duplication.** `calendrier.html.twig`
 * and `machine-calendrier.html.twig` each carried their own hand-written
 * `RESERVATIONS = [ … ]` loop, each re-deciding — in Twig, inside a `<script>` —
 * whose name may be shown, whose booking becomes a link, and which reason string
 * to print. Two copies of a *privacy* rule is one copy too many: S38 put that
 * decision on the server precisely so a template could not get it wrong, and
 * then it was written out twice in templates.
 *
 * ⚠️ **The identity rules are the reason this is a service and not a Twig loop.**
 * `user` is emitted only when the viewer is entitled to it, `motif` only to the
 * booker or an entitled viewer, `url` only for your own booking — because
 * `/reservations/{id}` answers 404 to anyone else — and the raw `user_id` is
 * never emitted at all. Anything added here must answer "may this viewer see it"
 * before it is put on the wire.
 *
 * ⚠️ **Plain `format()`, not the lab timezone.** A booking's start is a wall
 * clock a human typed, stored and read in the same default timezone; converting
 * it would move a 10:00 slot to 12:00. This mirrors the templates' plain
 * `|date()` exactly — see `LabTimeExtension` for which is which.
 */
final class CalendarPayload
{
    public function __construct(
        private readonly ReservableResolver $reservables,
        private readonly BookingIdentityPolicy $identity,
        private readonly ScheduleResolver $schedule,
        private readonly TranslatorInterface $translator,
        private readonly UrlGeneratorInterface $urls,
    ) {}

    /**
     * @param Reservation[]                      $reservations
     * @param list<array<string, mixed>>         $resources  `kind:id` keyed rows, as the picker and the grid read them
     * @param array<string, array<string, mixed>> $access    booking verdicts, keyed the same `kind:id` way
     * @param Event[]                            $events
     *
     * @return array<string, mixed>
     */
    public function build(
        ?int $venueId,
        array $reservations,
        array $resources,
        array $access,
        array $events,
        bool $authenticated,
        bool $admin,
        bool $booking,
        bool $unavailable = false,
    ): array {
        $this->reservables->warm($reservations);

        return [
            'startHour' => $this->schedule->calendarStartHour($venueId),
            'endHour' => $this->schedule->calendarEndHour($venueId),
            'slotMinutes' => 60,
            'hours' => $this->schedule->forJson($venueId),
            // A horizon rather than everything: a lab three years old would ship a
            // payload of dead holidays to every visitor (S134e).
            'exceptions' => $this->schedule->exceptionsBetween(
                $venueId,
                new \DateTimeImmutable('today'),
                new \DateTimeImmutable('+120 days'),
            ) ?: new \stdClass(),
            'resources' => $resources,
            'access' => $access ?: new \stdClass(),
            'reservations' => $this->reservationRows($reservations),
            'events' => $this->eventRows($events),
            'authenticated' => $authenticated,
            'admin' => $admin,
            'booking' => $booking,
            'unavailable' => $unavailable,
            'createUrl' => $this->urls->generate('api_reservation_create'),
            'labels' => $this->labels(),
        ];
    }

    /**
     * @param Reservation[] $reservations
     *
     * @return list<array<string, mixed>>
     */
    private function reservationRows(array $reservations): array
    {
        $showName = $this->identity->canSeeOthersIdentity();
        $viewerId = $this->identity->viewerId();
        $rows = [];

        foreach ($reservations as $reservation) {
            $ref = $this->reservables->resolve($reservation);
            $type = $ref->type;
            $id = $ref->id;
            $owner = $reservation->getUtilisateur();
            $mine = $viewerId !== null && $owner !== null && $owner->getId() === $viewerId;
            $motif = $reservation->getMotif();

            $rows[] = [
                'id' => $reservation->getId(),
                'resource_key' => $type !== null && $id !== null ? $type->value . ':' . $id : null,
                'machine' => $ref->name,
                'date' => $reservation->getDateDebut()->format('Y-m-d'),
                'start' => $reservation->getDateDebut()->format('H:i'),
                'end' => $reservation->getDateFin()->format('H:i'),
                'mine' => $mine,
                'url' => $mine
                    ? $this->urls->generate('app_reservation_detail', ['id' => $reservation->getId()])
                    : null,
                'user' => $showName && $owner !== null ? $owner->getDisplayName() : null,
                'motif' => ($showName || $mine) && $motif ? $motif : null,
                'statut' => $reservation->getStatut(),
            ];
        }

        return $rows;
    }

    /**
     * @param Event[] $events
     *
     * @return list<array<string, mixed>>
     */
    private function eventRows(array $events): array
    {
        $rows = [];

        foreach ($events as $event) {
            // ⚠️ `Event::getDateDebut()` is nullable — a draft with no date has no
            // place on a calendar, and formatting null is a TypeError, not a blank.
            $start = $event->getDateDebut();
            if ($start === null) {
                continue;
            }

            $rows[] = [
                'id' => $event->getId(),
                'title' => $event->getTitre(),
                'date' => $start->format('Y-m-d'),
                'hour' => $start->format('H'),
                'startLabel' => $start->format('H:i'),
                'lieu' => $event->getLieu(),
                // The lab's own word for what kind of thing this is (S146f), and
                // null when it has none — the calendar simply shows no chip.
                'category' => $event->getCategory()?->getLabel(),
                'url' => $this->urls->generate('app_event_detail', ['id' => $event->getId()]),
            ];
        }

        return $rows;
    }

    /**
     * ⚠️ **Every user-visible string the component writes comes from here.** S134c
     * had to clear twenty French literals out of these two `<script>` blocks, on a
     * site that ships five languages; a new string belongs in `messages.*.yaml` and
     * in this map, never inline in the controller.
     *
     * @return array<string, string>
     */
    private function labels(): array
    {
        $keys = [
            'practicalLocked' => 'cal.js_practical_locked',
            'trainingRequired' => 'cal.js_training_required',
            'venueClosed' => 'cal.js_venue_closed',
            'outOfHours' => 'cal.js_out_of_hours',
            'available' => 'cal.legend_free',
            'clickToBook' => 'cal.js_click_to_book',
            'moreBookings' => 'cal.js_more_bookings',
            'bookingsCount' => 'cal.js_bookings_count',
            'openWeek' => 'cal.js_open_week',
            'cancelled' => 'cal.js_cancelled',
            'confirmed' => 'cal.js_confirmed',
            'mine' => 'cal.js_mine',
            'taken' => 'cal.js_taken',
            'slotContext' => 'cal.js_slot_context',
            'slotDay' => 'cal.js_slot_day',
            'endBeforeStart' => 'cal.js_end_before_start',
            'outsideHours' => 'cal.js_outside_hours',
            'noResource' => 'cal.js_no_resource',
            'requiredFields' => 'cal.js_required_fields',
            'endDateBeforeStart' => 'cal.js_end_date_before_start',
            'creating' => 'cal.js_creating',
            'refused' => 'cal.js_refused',
            'created' => 'cal.js_created',
            'closedShort' => 'cal.js_closed_short',
            'unavailableWarning' => 'machine_calendar.js_unavailable_warning',
            'nextWeek' => 'cal.next_week',
            'navPrevWeek' => 'cal.prev_week',
            'navNextWeek' => 'cal.next_week',
            'navPrevMonth' => 'cal.prev_month',
            'navNextMonth' => 'cal.next_month',
            'week' => 'cal.week_label',
            'hour' => 'profile.col_time',
            'manage' => 'resv.manage',
        ];

        $labels = [];
        foreach ($keys as $name => $key) {
            $labels[$name] = $this->translator->trans($key);
        }

        return $labels;
    }
}
