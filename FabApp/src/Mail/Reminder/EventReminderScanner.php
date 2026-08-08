<?php

namespace App\Mail\Reminder;

use App\Entity\EventRegistration;
use App\Mail\ReminderSettings;
use App\Repository\EventRegistrationRepository;
use App\Feature\SiteFeatureService;

/**
 * "The workshop you signed up for is tomorrow."
 *
 * The reminder the events module was missing until registrations existed —
 * before S20 there was simply nobody to remind, which is why this scanner is
 * arriving a session later than the other three.
 *
 * Same sweep-a-window shape as the booking scanner: everything starting
 * between now and the configured lead time, claimed once per registration, so
 * someone who signs up *inside* the window still gets their reminder on the
 * next tick rather than never.
 *
 * Guests are reminded too. They have no account, so the candidate is built
 * against their bare address — the same split the loan scanner already makes
 * for walk-in borrowers. One consequence worth knowing: a member's reminder
 * honours their notification preferences, a guest's cannot, because there are
 * no preferences attached to an address. A guest who wants out cancels their
 * registration, which is the only lever they have and the right one.
 */
final class EventReminderScanner implements ReminderScanner
{
    public function __construct(
        private readonly EventRegistrationRepository $registrations,
        private readonly ReminderSettings $settings,
        private readonly SiteFeatureService $modules,
    ) {
    }

    public function kind(): string
    {
        return ReminderSettings::EVENT;
    }

    public function scan(\DateTimeImmutable $now): array
    {
        // A lab that has switched the events module off should not be mailing
        // people about events, even if the reminder toggle was left on.
        if (!$this->modules->isEnabled('events')) {
            return [];
        }

        $horizon = $now->modify(sprintf('+%d hours', $this->settings->getEventLeadHours()));

        $candidates = [];
        foreach ($this->registrations->findForEventsStartingBetween($now, $horizon) as $registration) {
            $candidate = $this->candidate($registration);
            if ($candidate !== null) {
                $candidates[] = $candidate;
            }
        }

        return $candidates;
    }

    private function candidate(EventRegistration $registration): ?ReminderCandidate
    {
        $event = $registration->getEvent();
        $id = $registration->getId();
        $start = $event?->getDateDebut();

        if ($event === null || $id === null || $start === null) {
            return null;
        }

        // Keyed on the registration, not the event: one reminder per person,
        // and a later signup gets its own claim rather than being swallowed by
        // an event-wide key someone else already used up.
        $key = sprintf('event:%d', $id);

        $context = [
            'event' => $event->getTitre(),
            // ISO strings — re-rendered by the worker, possibly in another locale.
            'start' => $start->format(\DATE_ATOM),
            'end' => $event->getDateFin()?->format(\DATE_ATOM),
            'place' => $event->getLieu(),
            'attendee' => $registration->getDisplayName(),
        ];

        $user = $registration->getUtilisateur();

        return $user !== null
            ? ReminderCandidate::forUser($key, $user, 'reminder_event', $context)
            : ReminderCandidate::forAddress($key, $registration->getContactEmail(), $registration->getGuestName(), 'reminder_event', $context);
    }
}
