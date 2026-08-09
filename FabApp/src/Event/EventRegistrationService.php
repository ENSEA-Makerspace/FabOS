<?php

namespace App\Event;

use App\Entity\Event;
use App\Entity\EventRegistration;
use App\Entity\Utilisateur;
use App\Repository\EventRegistrationRepository;
use Doctrine\DBAL\LockMode;
use Doctrine\ORM\EntityManagerInterface;
use App\UsageRights\UsageRightsService;

/**
 * The single chokepoint for taking and giving up a place at an event, in the
 * same spirit as ReservationService: callers parse input and render, this owns
 * the rules.
 *
 * The hard part is the last seat. Counting seats and then inserting is a
 * check-then-act race — two people can both read "one left" and both take it.
 * So the count happens inside a transaction that has first taken a lock on the
 * event row, which serialises everyone competing for the same event while
 * leaving registrations for *different* events completely unblocked.
 *
 * Re-registering after cancelling reuses the original row rather than inserting
 * a second one. That keeps the unique (event, contactEmail) index honest and
 * means one person is one line on the organiser's list no matter how many times
 * they changed their mind.
 */
final class EventRegistrationService
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly EventRegistrationRepository $registrations,
        private readonly EventMailer $mails,
        private readonly UsageRightsService $usageRights,
    ) {
    }

    /**
     * Take a place, as a member or a guest.
     *
     * @param Utilisateur|null $user  the account, when someone is signed in
     * @param string|null      $email required only for guests; members use their account address
     */
    public function register(Event $event, ?Utilisateur $user, ?string $email = null, ?string $guestName = null): RegistrationResult
    {
        $contact = EventRegistration::normaliseEmail($user?->getEmail() ?? (string) $email);

        if ($contact === '' || filter_var($contact, FILTER_VALIDATE_EMAIL) === false) {
            return RegistrationResult::refused('EMAIL_INVALID', 'Indiquez une adresse e-mail valide pour vous inscrire.', 400);
        }

        if (!$event->isRegistrationOpen()) {
            return RegistrationResult::refused('REGISTRATION_CLOSED', 'Les inscriptions à cet événement sont closes.');
        }

        // Checked here rather than only in the template: the form is a courtesy,
        // this is the rule. A members-only event must refuse a hand-posted guest
        // registration just as firmly as it hides the guest form.
        if ($user === null && !$event->isGuestsAllowed()) {
            return RegistrationResult::refused(
                'MEMBERS_ONLY',
                'Cet événement est réservé aux membres : connectez-vous pour vous inscrire.',
                403,
            );
        }

        // Guests do not have an account to which a package can be assigned; the
        // existing guest policy remains authoritative for them.
        // A guest-open event is public by definition; a signed-in member must
        // not be worse off than the same person signed out. Packages govern the
        // member-only path, while the event's guest policy governs public entry.
        if ($user !== null && !$event->isGuestsAllowed() && !$this->usageRights->verdict($user, 'events', $event->getDateDebut(), $event->getDateFin())->allowed) {
            return RegistrationResult::refused('USAGE_RIGHTS_DENIED', 'Votre package de droits d’usage ne couvre pas les événements.', 403);
        }

        try {
            $result = $this->em->wrapInTransaction(function () use ($event, $user, $contact, $guestName): RegistrationResult {
                // Serialises everyone racing for this event's last seat, and only
                // this event's — the lock is on the one row.
                $this->em->lock($event, LockMode::PESSIMISTIC_WRITE);

                $existing = $this->registrations->findOneForContact($event, $contact);
                if ($existing !== null && !$existing->isCancelled()) {
                    return RegistrationResult::refused(
                        'ALREADY_REGISTERED',
                        $existing->isWaitlisted()
                            ? 'Vous êtes déjà sur la liste d\'attente de cet événement.'
                            : 'Vous êtes déjà inscrit à cet événement.',
                    );
                }

                $registration = $existing ?? new EventRegistration();
                $registration
                    ->setEvent($event)
                    ->setUtilisateur($user)
                    ->setContactEmail($contact)
                    ->setGuestName($user === null ? $guestName : null)
                    ->setCancelledAt(null)
                    ->setPromotedAt(null)
                    // A returning registrant joins the queue from now, not from
                    // whenever they first signed up and then dropped out.
                    ->setCreatedAt(new \DateTimeImmutable());

                $full = $event->hasCapacityLimit()
                    && $this->registrations->countSeatsTaken($event) >= (int) $event->getCapacite();

                $registration->setStatus($full ? EventRegistration::STATUS_WAITLISTED : EventRegistration::STATUS_REGISTERED);

                $this->em->persist($registration);
                $this->em->flush();

                return $full
                    ? RegistrationResult::waitlisted($registration)
                    : RegistrationResult::registered($registration);
            });
        } catch (\Throwable $e) {
            return RegistrationResult::refused('REGISTRATION_FAILED', 'L\'inscription n\'a pas pu être enregistrée : ' . $e->getMessage(), 500);
        }

        // Outside the transaction, and never able to fail it: the place is taken
        // whether or not anyone can be told about it.
        if ($result->ok && $result->registration !== null) {
            $result->isWaitlisted()
                ? $this->mails->waitlisted($result->registration)
                : $this->mails->registered($result->registration);
        }

        return $result;
    }

    /**
     * Give up a place, and hand it to whoever has been waiting longest.
     *
     * The promotion happens in the same transaction as the cancellation, so a
     * seat is never briefly free for someone else to take, and never lost
     * because the process died between the two writes.
     */
    public function cancel(EventRegistration $registration): RegistrationResult
    {
        if ($registration->isCancelled()) {
            return RegistrationResult::refused('ALREADY_CANCELLED', 'Cette inscription est déjà annulée.');
        }

        $event = $registration->getEvent();
        $freedSeat = $registration->holdsSeat();

        try {
            $promoted = $this->em->wrapInTransaction(function () use ($registration, $event, $freedSeat): ?EventRegistration {
                if ($event !== null) {
                    $this->em->lock($event, LockMode::PESSIMISTIC_WRITE);
                }

                $registration
                    ->setStatus(EventRegistration::STATUS_CANCELLED)
                    ->setCancelledAt(new \DateTimeImmutable());

                $promoted = null;

                // Only a seat that was actually occupied frees one up; cancelling
                // from the waitlist promotes nobody.
                if ($freedSeat && $event !== null) {
                    $this->em->flush();
                    $promoted = $this->promoteNextIfSeatFree($event);
                }

                $this->em->flush();

                return $promoted;
            });
        } catch (\Throwable $e) {
            return RegistrationResult::refused('CANCEL_FAILED', 'L\'annulation n\'a pas pu être enregistrée : ' . $e->getMessage(), 500);
        }

        $this->mails->cancelled($registration);

        if ($promoted !== null) {
            $this->mails->promoted($promoted);
        }

        return RegistrationResult::cancelled($registration);
    }

    /**
     * Moves the longest-waiting person into a free seat, if there is one.
     *
     * Re-checks capacity rather than assuming: an organiser may have *lowered*
     * the capacity since, in which case a cancellation should not promote
     * anybody into a seat that no longer exists.
     */
    private function promoteNextIfSeatFree(Event $event): ?EventRegistration
    {
        if (!$event->hasCapacityLimit()) {
            return null;
        }

        if ($this->registrations->countSeatsTaken($event) >= (int) $event->getCapacite()) {
            return null;
        }

        $next = $this->registrations->findNextWaitlisted($event);
        if ($next === null) {
            return null;
        }

        $next
            ->setStatus(EventRegistration::STATUS_REGISTERED)
            ->setPromotedAt(new \DateTimeImmutable());

        return $next;
    }

    /**
     * Call the whole event off and tell everyone holding a place why.
     *
     * The registrations are deliberately left standing rather than cancelled:
     * the organiser needs the list of who was told, and a registrant reopening
     * an old link should see "this was called off, here's why" rather than an
     * empty page. `Event::isRegistrationOpen()` already refuses once cancelled,
     * so nothing new can be booked either way.
     *
     * Waitlisted people are told too — they were waiting for something that is
     * no longer going to happen, and silence would leave them expecting a seat.
     *
     * @return int how many people were notified
     */
    public function callOff(Event $event, ?string $reason): int
    {
        if ($event->isCancelled()) {
            return 0;
        }

        $event->callOff($reason);
        $this->em->flush();

        $notified = 0;
        foreach ($this->registrations->findForEvent($event) as $registration) {
            // People who had already dropped out don't need telling.
            if ($registration->isCancelled()) {
                continue;
            }

            $this->mails->calledOff($registration, $event->getCancellationReason());
            ++$notified;
        }

        return $notified;
    }

    /** Seats left, or null when the event is unlimited. */
    public function seatsRemaining(Event $event): ?int
    {
        if (!$event->hasCapacityLimit()) {
            return null;
        }

        return max(0, (int) $event->getCapacite() - $this->registrations->countSeatsTaken($event));
    }
}
