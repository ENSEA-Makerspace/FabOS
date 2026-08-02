<?php

namespace App\Reservation\Verb;

/**
 * *Which surface is asking?* — because the same person gets a different answer
 * depending on where they are standing.
 *
 * An admin looking at `/mes-reservations` is a **member looking at their own
 * bookings**. They should see exactly what any member sees: no "Annuler" on a
 * booking that finished last March, because changing your mind about the past is
 * not a thing a member does. The same admin on `/admin/reservations` is doing
 * something else entirely — correcting the record — and that is a legitimate
 * staff act.
 *
 * ⚠️ Before this existed the service asked `isGranted('ROLE_ADMIN')` and nothing
 * else, so the staff power leaked onto the member pages: an operator saw a
 * Cancel button on their own past bookings and rightly asked why. Role says
 * *what you may do*; context says *what you are currently doing*. Both are
 * needed, and conflating them is what put a data-correction control on a
 * personal page.
 *
 * ⚠️ This is **not** a security boundary on its own. `Staff` is only reachable
 * through a route that is itself gated on the role, so a member cannot obtain it
 * by asking. It decides which verbs a surface *offers*; the firewall and the
 * ownership check still decide what the endpoint *honours*.
 */
enum VerbContext
{
    /** A member acting on their own booking. The default everywhere. */
    case Member;

    /**
     * Staff correcting the record from an admin surface.
     *
     * ⚠️ It lifts the *time* restriction on cancelling (past and running
     * bookings), nothing else. It is not a general skeleton key: ownership,
     * already-cancelled and the resource checks all still apply.
     *
     * ⚠️ **Unaudited today.** S63 (notes + change log) is what makes this
     * attributable, and S62 is where staff-on-behalf properly belongs. This
     * enum confines a power that was previously ambient; it does not widen it,
     * and it does not make it safe to widen further before S63 lands.
     */
    case Staff;
}
