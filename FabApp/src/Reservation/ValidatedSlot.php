<?php

namespace App\Reservation;

use App\Reservation\Policy\AccessPass;

/**
 * What survives `ReservationService::validate()` when a slot is legal: the
 * resolved resource name, whether the booking lands pending, and the pass that
 * lifted the quotas (if any).
 *
 * It exists so that creating a booking and *moving* one can run the identical
 * rule sequence and differ only in what they do afterwards. Before this, moving
 * a booking would have meant a second copy of the access gate, the opening
 * hours, the overlap check and the quotas — four rules that would have drifted
 * apart the first time one of them changed.
 */
final readonly class ValidatedSlot
{
    public function __construct(
        public string $name,
        public bool $pending,
        public ?AccessPass $pass = null,
    ) {
    }
}
