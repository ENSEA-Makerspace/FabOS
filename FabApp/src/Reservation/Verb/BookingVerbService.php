<?php

namespace App\Reservation\Verb;

use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Reservation\LabClock;
use App\Reservation\Policy\BookingPolicyService;
use Symfony\Bundle\SecurityBundle\Security;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * One place that decides what may be done to an existing booking — asked by the
 * endpoints before they mutate anything, and by `/mes-reservations` before it
 * draws anything.
 *
 * ⚠️ **The page and the endpoint must ask the same question.** The reason this
 * is a service and not two `if`s is that the previous arrangement had the
 * template deciding whether to draw the cancel form and the controller deciding
 * whether to honour it, with the rule written out twice. They already disagreed:
 * the template hid cancel on a booking that had started, the endpoint refused it
 * with a different sentence, and neither could say what a member should do
 * instead.
 *
 * ⚠️ **This asks `BookingPolicyService` for the lock window and does not own
 * one.** S68 fills that in; until it does, `changeDeadlineFor()` answers null and
 * every window check here passes. That ordering is deliberate — a window
 * restricting verbs that do not exist yet cannot be tested.
 */
final class BookingVerbService
{
    public function __construct(
        private readonly BookingPolicyService $policies,
        private readonly LabClock $clock,
        private readonly Security $security,
        private readonly TranslatorInterface $translator,
    ) {
    }

    /**
     * Every verb's verdict for one booking, keyed by verb value — one call per
     * card, so the template never re-derives a rule.
     *
     * @return array<string, VerbVerdict>
     */
    public function verdicts(
        Reservation $reservation,
        ?Utilisateur $actor,
        ?\DateTimeImmutable $now = null,
        VerbContext $context = VerbContext::Member,
    ): array {
        $now ??= $this->clock->now();
        $verdicts = [];

        foreach (BookingVerb::cases() as $verb) {
            $verdicts[$verb->value] = $this->verdict($verb, $reservation, $actor, $now, $context);
        }

        return $verdicts;
    }

    public function verdict(
        BookingVerb $verb,
        Reservation $reservation,
        ?Utilisateur $actor,
        ?\DateTimeImmutable $now = null,
        VerbContext $context = VerbContext::Member,
    ): VerbVerdict {
        $now ??= $this->clock->now();

        // ⚠️ Both ends re-labelled from stored wall-clock to a real instant
        // before anything is compared. Skipping this is a silent shift by the
        // lab's UTC offset, always in the permissive direction — see LabClock.
        $start = $this->clock->instantOf($reservation->getDateDebut());
        $end = $this->clock->instantOf($reservation->getDateFin());

        $isAdmin = $this->security->isGranted('ROLE_ADMIN');
        $owns = $actor !== null && $reservation->getUtilisateur()?->getId() === $actor->getId();

        if (!$owns && !$isAdmin) {
            // Quiet: a stranger's booking should not explain itself, and on the
            // member's own page this branch is unreachable anyway.
            return VerbVerdict::quietly($verb, 'RESERVATION_FORBIDDEN', $this->say('resv.why_not_yours'), 403);
        }

        return match ($verb) {
            BookingVerb::Cancel => $this->judgeCancel($reservation, $start, $end, $now, $isAdmin, $context),
            BookingVerb::EndNow => $this->judgeEndNow($reservation, $start, $end, $now),
            BookingVerb::Reschedule => $this->judgeReschedule($reservation, $start, $end, $now, $isAdmin),
            BookingVerb::Restore => $this->judgeRestore($reservation, $start, $now),
        };
    }

    private function judgeCancel(
        Reservation $reservation,
        \DateTimeImmutable $start,
        \DateTimeImmutable $end,
        \DateTimeImmutable $now,
        bool $isAdmin,
        VerbContext $context,
    ): VerbVerdict {
        if (!$reservation->isActive()) {
            return VerbVerdict::quietly(
                BookingVerb::Cancel,
                $reservation->isDeclined() ? 'RESERVATION_DECLINED' : 'RESERVATION_ALREADY_CANCELLED',
                $this->say($reservation->isDeclined() ? 'resv.why_declined' : 'resv.why_already_cancelled'),
            );
        }

        // ⚠️ **Staff may cancel a booking that has started or finished — but only
        // from a staff surface.** That is a data correction, not a member
        // changing their mind about the past, and the two are different acts by
        // the same person. Gating it on the role alone (as this did until an
        // operator asked why their own finished bookings still offered
        // "Annuler") leaks a records-management control onto a personal page.
        // The context is only reachable through a role-gated staff route, so
        // this confines the power rather than widening it. Auditing it is S63.
        if ($isAdmin && $context === VerbContext::Staff) {
            return VerbVerdict::allow(BookingVerb::Cancel);
        }

        if ($end <= $now) {
            return VerbVerdict::quietly(BookingVerb::Cancel, 'RESERVATION_NOT_FUTURE', $this->say('resv.why_finished'));
        }

        if ($start <= $now) {
            // The one refusal that was costing the lab something real: "I am
            // here and I am done early" had no path at all, so the member simply
            // walked off and the slot stayed blocked. Now it names the verb that
            // does work, on the same card.
            return VerbVerdict::explained(BookingVerb::Cancel, 'RESERVATION_RUNNING', $this->say('resv.why_running_cancel'));
        }

        return $this->withinChangeWindow(BookingVerb::Cancel, $reservation, $start, $now);
    }

    private function judgeReschedule(
        Reservation $reservation,
        \DateTimeImmutable $start,
        \DateTimeImmutable $end,
        \DateTimeImmutable $now,
        bool $isAdmin,
    ): VerbVerdict {
        if (!$reservation->isActive()) {
            return VerbVerdict::quietly(
                BookingVerb::Reschedule,
                $reservation->isDeclined() ? 'RESERVATION_DECLINED' : 'RESERVATION_ALREADY_CANCELLED',
                $this->say($reservation->isDeclined() ? 'resv.why_declined' : 'resv.why_already_cancelled'),
            );
        }

        if ($end <= $now) {
            return VerbVerdict::quietly(BookingVerb::Reschedule, 'RESERVATION_NOT_FUTURE', $this->say('resv.why_finished'));
        }

        // ⚠️ Not admin-exempt, and not an oversight. Moving a booking that has
        // started would move a slot somebody is standing in; the honest staff
        // action there is to end it and book afresh.
        if ($start <= $now) {
            return VerbVerdict::explained(BookingVerb::Reschedule, 'RESERVATION_RUNNING', $this->say('resv.why_running_move'));
        }

        if ($isAdmin) {
            return VerbVerdict::allow(BookingVerb::Reschedule);
        }

        return $this->withinChangeWindow(BookingVerb::Reschedule, $reservation, $start, $now);
    }

    private function judgeEndNow(
        Reservation $reservation,
        \DateTimeImmutable $start,
        \DateTimeImmutable $end,
        \DateTimeImmutable $now,
    ): VerbVerdict {
        if (!$reservation->isActive()) {
            return VerbVerdict::quietly(BookingVerb::EndNow, 'RESERVATION_NOT_RUNNING', $this->say('resv.why_already_cancelled'));
        }

        // A pending request is not a booking anyone is standing in — there is
        // nothing to finish early. Withdrawing it is a cancel.
        if ($reservation->isPending()) {
            return VerbVerdict::quietly(BookingVerb::EndNow, 'RESERVATION_PENDING', $this->say('resv.why_pending_end'));
        }

        if ($start > $now || $end <= $now) {
            return VerbVerdict::quietly(BookingVerb::EndNow, 'RESERVATION_NOT_RUNNING', $this->say('resv.why_not_running'));
        }

        // ⚠️ No change-window check here, by decision, and no admin branch
        // either — there is nothing to override. Finishing early is always
        // allowed to whoever holds the booking.
        return VerbVerdict::allow(BookingVerb::EndNow);
    }

    private function judgeRestore(
        Reservation $reservation,
        \DateTimeImmutable $start,
        \DateTimeImmutable $now,
    ): VerbVerdict {
        if (!$reservation->isCancelled()) {
            return VerbVerdict::quietly(BookingVerb::Restore, 'RESERVATION_NOT_CANCELLED', $this->say('resv.why_not_cancelled'));
        }

        if ($start <= $now) {
            return VerbVerdict::quietly(BookingVerb::Restore, 'RESERVATION_NOT_FUTURE', $this->say('resv.why_restore_late'));
        }

        // Whether the slot is *still free* is not decided here: it is a race, and
        // an answer computed while drawing a page would be stale by the time the
        // member clicked. The restore path re-runs the full booking rules and
        // refuses there if somebody took the slot in between.
        return VerbVerdict::allow(BookingVerb::Restore);
    }

    /**
     * S68's lock window, asked rather than owned.
     *
     * ⚠️ When it refuses it **names the deadline**. "Trop tard" alone reads as a
     * bug; "il fallait annuler avant le 02/08/2026 08:00" reads as a rule, and a
     * member can plan around a rule.
     */
    private function withinChangeWindow(
        BookingVerb $verb,
        Reservation $reservation,
        \DateTimeImmutable $start,
        \DateTimeImmutable $now,
    ): VerbVerdict {
        $type = $reservation->getReservableType();
        $owner = $reservation->getUtilisateur();
        if ($type === null || $owner === null) {
            return VerbVerdict::allow($verb);
        }

        $deadline = $this->policies->changeDeadlineFor($owner, $type, $start);
        if ($deadline === null || $now <= $deadline) {
            return VerbVerdict::allow($verb);
        }

        return VerbVerdict::explained($verb, 'CHANGE_WINDOW_CLOSED', $this->translator->trans(
            $verb === BookingVerb::Cancel ? 'resv.why_locked_cancel' : 'resv.why_locked_move',
            ['%deadline%' => $deadline->format('d/m/Y H:i')],
        ));
    }

    private function say(string $key): string
    {
        return $this->translator->trans($key);
    }
}
