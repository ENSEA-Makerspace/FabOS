<?php

namespace App\Event;

use App\Entity\EventRegistration;
use App\Mail\Mailer;
use App\Mail\NotificationCategory;
use App\Mail\UnsubscribeLinker;

/**
 * Every event-registration mail, in one place — the same shape as
 * ReservationMailer, and equally unable to fail the thing that triggered it.
 *
 * All four are **transactional**: each one answers something the person did
 * themselves, and "a seat opened up for you" is precisely the mail somebody
 * joined a waitlist in order to receive. Suppressing any of them because of a
 * preference would break the feature rather than respect a choice.
 *
 * Guests get the same mail as members. That is why everything goes out through
 * the raw-address `queue()` rather than `queueToUser()`: the registration's
 * contactEmail is the identity here, and half the registrants may have no
 * account at all.
 */
final class EventMailer
{
    public function __construct(
        private readonly Mailer $mailer,
        private readonly UnsubscribeLinker $links,
    ) {
    }

    public function registered(EventRegistration $registration): void
    {
        $this->send($registration, 'event_registered');
    }

    public function waitlisted(EventRegistration $registration): void
    {
        $this->send($registration, 'event_waitlisted');
    }

    /** A seat opened and this person was next in the queue. */
    public function promoted(EventRegistration $registration): void
    {
        $this->send($registration, 'event_promoted');
    }

    /** The registrant gave up their own place. */
    public function cancelled(EventRegistration $registration): void
    {
        $this->send($registration, 'event_cancelled');
    }

    /**
     * The organiser called the whole event off. Distinct from cancelled(): that
     * one confirms something the recipient chose, this one delivers news they
     * did not, and carries the organiser's reason.
     */
    public function calledOff(EventRegistration $registration, ?string $reason): void
    {
        $this->send($registration, 'event_called_off', ['reason' => $reason]);
    }

    /** @param array<string, mixed> $extra */
    private function send(EventRegistration $registration, string $template, array $extra = []): void
    {
        try {
            $event = $registration->getEvent();
            if ($event === null) {
                return;
            }

            $user = $registration->getUtilisateur();

            $this->mailer->queue(
                $registration->getContactEmail(),
                $registration->getDisplayName(),
                $template,
                [
                    // ISO strings: the context is stored as JSON and re-rendered
                    // later by the worker, possibly in another locale.
                    'event' => $event->getTitre(),
                    'start' => $event->getDateDebut()?->format(\DATE_ATOM),
                    'end' => $event->getDateFin()?->format(\DATE_ATOM),
                    'place' => $event->getLieu(),
                    'attendee' => $registration->getDisplayName(),
                    // Guests have no account to cancel from, so the mail carries
                    // the signed link that is their only way out.
                    'cancel_url' => $registration->isGuest() ? $this->links->eventCancelUrl($registration) : null,
                ] + $extra,
                NotificationCategory::EVENT,
                // Members read in their own language; a guest gets the site default.
                $user?->getLangue(),
                $user?->getId(),
            );
        } catch (\Throwable) {
            // Telling someone is never worth failing their registration.
        }
    }
}
