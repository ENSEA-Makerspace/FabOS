<?php

namespace App\Reservation\Verb;

/**
 * The four things a member can do to a booking they already hold.
 *
 * They are separate verbs rather than flags on one "edit" action because the
 * lab treats them differently and the member reads them differently:
 *
 *  - **Cancel** gives the slot back and the booking stops counting as honoured.
 *  - **EndNow** keeps the booking attributed and honoured; only its end moves.
 *    ⚠️ It is deliberately *not* a cancellation, and deliberately *never*
 *    restricted by the lock window — somebody standing at the machine who has
 *    finished early is doing the lab a favour, and a rule that stops them is a
 *    rule that teaches people to walk away instead.
 *  - **Reschedule** is a move: the old slot is held until the new one is
 *    confirmed, so a failed move leaves the member exactly where they were.
 *  - **Restore** undoes a cancellation while the slot is still free. It is the
 *    cancel-with-undo S47 left behind, folded in here rather than built a second
 *    time as its own model of the same action.
 */
enum BookingVerb: string
{
    case Cancel = 'cancel';
    case EndNow = 'end_now';
    case Reschedule = 'reschedule';
    case Restore = 'restore';

    /** The CSRF token name a form posting this verb must carry. */
    public function csrfToken(int $reservationId): string
    {
        // ⚠️ `cancel_reservation_<id>` is the name already in the wild, on forms
        // this session did not write. Renaming it would 400 every cancel form on
        // a page a member had open across the deploy.
        return match ($this) {
            self::Cancel => 'cancel_reservation_' . $reservationId,
            default => $this->value . '_reservation_' . $reservationId,
        };
    }

    /** Message key for the button label. */
    public function labelKey(): string
    {
        return match ($this) {
            self::Cancel => 'common.cancel',
            self::EndNow => 'resv.act_end_now',
            self::Reschedule => 'resv.act_reschedule',
            self::Restore => 'resv.act_restore',
        };
    }
}
