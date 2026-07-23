<?php

namespace App\Reservation\Policy;

use App\Entity\Utilisateur;
use App\Repository\ReservationRepository;
use App\Reservation\BookingResult;
use App\Reservation\ReservableType;

/**
 * "How much may you book?" — the quota half of booking permission.
 *
 * Kept strictly apart from the certification gate, which answers a different
 * question: cert-gating is *may you touch this at all* (a safety fact about the
 * person), quotas are *how much and how far ahead* (a fairness policy of the
 * lab). Mixing them would mean a lab that loosens its quotas could accidentally
 * loosen its safety rules, which is exactly the mistake worth designing out.
 *
 * Every check is skipped when its limit is null, so an unconfigured lab pays
 * nothing — not even the count queries, which only run when a cap exists.
 */
final class BookingPolicyService
{
    public function __construct(
        private readonly BookingPolicyRepository $policies,
        private readonly ReservationRepository $reservations,
    ) {
    }

    public function policyFor(Utilisateur $user, ReservableType $type): BookingPolicy
    {
        $tier = BookingTier::forUser($user);

        return $this->policies->find($type, $tier) ?? BookingPolicy::unrestricted($type, $tier);
    }

    /**
     * The quota verdict for one proposed booking.
     *
     * Ordered coarsest-constraint-first, because a booking usually breaks more
     * than one rule and the person only sees the first answer. "You can't book
     * this soon at all" has to come before "round it to the nearest half hour":
     * moving the booking to a legal time often fixes the alignment on the way,
     * whereas fixing the alignment of a slot that was never bookable just earns
     * a second refusal. Cheap pure-arithmetic checks still precede the ones that
     * hit the database.
     *
     * @return BookingResult|null null when the booking is within quota
     */
    public function check(
        Utilisateur $user,
        ReservableType $type,
        int $id,
        \DateTimeImmutable $start,
        \DateTimeImmutable $end,
        \DateTimeImmutable $now,
        bool $passApplies = false,
    ): ?BookingResult {
        // A pass lifts the quotas and stops there. It does not reach the access
        // gate, the opening hours or the overlap check — those have already run
        // or are still to come, and none of them are things staff should be able
        // to wave away by handing someone a pass.
        if ($passApplies) {
            return null;
        }

        $policy = $this->policyFor($user, $type);
        if ($policy->isUnrestricted()) {
            return null;
        }

        $durationMinutes = (int) round(($end->getTimestamp() - $start->getTimestamp()) / 60);

        if ($policy->minNoticeMinutes !== null) {
            $earliest = $now->modify(sprintf('+%d minutes', $policy->minNoticeMinutes));
            if ($start < $earliest) {
                return $this->refuse('NOTICE_TOO_SHORT', sprintf(
                    'Il faut réserver au moins %s à l\'avance.',
                    $this->humanMinutes($policy->minNoticeMinutes),
                ));
            }
        }

        if ($policy->maxHorizonDays !== null) {
            $latest = $now->modify(sprintf('+%d days', $policy->maxHorizonDays));
            if ($start > $latest) {
                return $this->refuse('HORIZON_EXCEEDED', sprintf(
                    'Vous ne pouvez pas réserver plus de %d jour(s) à l\'avance.',
                    $policy->maxHorizonDays,
                ));
            }
        }

        if ($policy->minDurationMinutes !== null && $durationMinutes < $policy->minDurationMinutes) {
            return $this->refuse('DURATION_TOO_SHORT', sprintf(
                'Cette réservation est trop courte : le minimum est de %s.',
                $this->humanMinutes($policy->minDurationMinutes),
            ));
        }

        if ($policy->maxDurationMinutes !== null && $durationMinutes > $policy->maxDurationMinutes) {
            return $this->refuse('DURATION_TOO_LONG', sprintf(
                'Cette réservation est trop longue : le maximum est de %s.',
                $this->humanMinutes($policy->maxDurationMinutes),
            ));
        }

        if ($policy->slotIncrementMinutes !== null && $policy->slotIncrementMinutes > 0) {
            $step = $policy->slotIncrementMinutes;
            // Measured from midnight local time, which is what "half-past" means
            // to a human reading a calendar.
            $startOffset = ((int) $start->format('H')) * 60 + (int) $start->format('i');

            if ($startOffset % $step !== 0 || $durationMinutes % $step !== 0) {
                return $this->refuse('SLOT_NOT_ALIGNED', sprintf(
                    'Les réservations se font par tranches de %s : ajustez le début et la durée.',
                    $this->humanMinutes($step),
                ));
            }
        }

        // The resource-side rule: a gap either side of the slot, so the next
        // person isn't waiting on a machine that's still cooling down. Reuses
        // the overlap query with a widened window.
        if ($policy->bufferMinutes !== null && $policy->bufferMinutes > 0) {
            $padded = $this->reservations->hasOverlap(
                $type,
                $id,
                $start->modify(sprintf('-%d minutes', $policy->bufferMinutes)),
                $end->modify(sprintf('+%d minutes', $policy->bufferMinutes)),
            );

            if ($padded) {
                return $this->refuse('BUFFER_CONFLICT', sprintf(
                    'Il faut laisser %s entre deux réservations de cette ressource.',
                    $this->humanMinutes($policy->bufferMinutes),
                ));
            }
        }

        if ($policy->maxActiveReservations !== null
            && $this->reservations->countActiveUpcomingForUser($user, $now) >= $policy->maxActiveReservations) {
            return $this->refuse('TOO_MANY_ACTIVE', sprintf(
                'Vous avez déjà %d réservation(s) en cours : annulez-en une avant d\'en ajouter une autre.',
                $policy->maxActiveReservations,
            ));
        }

        if ($policy->maxPerDay !== null) {
            $dayStart = $start->setTime(0, 0);
            if ($this->reservations->countForUserStartingBetween($user, $dayStart, $dayStart->modify('+1 day')) >= $policy->maxPerDay) {
                return $this->refuse('DAILY_LIMIT_REACHED', sprintf(
                    'Vous avez atteint la limite de %d réservation(s) par jour.',
                    $policy->maxPerDay,
                ));
            }
        }

        if ($policy->maxPerWeek !== null) {
            // Monday-based, matching how the lab's opening hours are read.
            $weekStart = $start->modify('monday this week')->setTime(0, 0);
            if ($this->reservations->countForUserStartingBetween($user, $weekStart, $weekStart->modify('+7 days')) >= $policy->maxPerWeek) {
                return $this->refuse('WEEKLY_LIMIT_REACHED', sprintf(
                    'Vous avez atteint la limite de %d réservation(s) par semaine.',
                    $policy->maxPerWeek,
                ));
            }
        }

        return null;
    }

    private function refuse(string $code, string $message): BookingResult
    {
        // 409, not 403: the booking isn't forbidden to this person in principle,
        // it conflicts with how much they've already taken. Changing the request
        // (or cancelling something) makes it succeed.
        return BookingResult::refused($code, $message, 409);
    }

    private function humanMinutes(int $minutes): string
    {
        if ($minutes < 60) {
            return sprintf('%d minute(s)', $minutes);
        }

        $hours = intdiv($minutes, 60);
        $rest = $minutes % 60;

        return $rest === 0
            ? sprintf('%dh', $hours)
            : sprintf('%dh%02d', $hours, $rest);
    }
}
