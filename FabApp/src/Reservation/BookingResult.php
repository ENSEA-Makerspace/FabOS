<?php

namespace App\Reservation;

use App\Entity\Reservation;

/**
 * Outcome of a booking attempt, in a shape both callers can render: the JSON API
 * needs a stable machine code plus an HTTP status, the place form needs a French
 * sentence for a flash message. Carrying all three here is what lets the two
 * flows share one rule set instead of each re-implementing it.
 */
final readonly class BookingResult
{
    private function __construct(
        public bool $ok,
        public ?Reservation $reservation = null,
        public ?string $code = null,
        public ?string $message = null,
        public int $status = 200,
        /** @var array<string, mixed> extra payload fields, e.g. missingBadges */
        public array $context = [],
    ) {
    }

    public static function created(Reservation $reservation): self
    {
        return new self(true, $reservation, status: 201);
    }

    /** @param array<string, mixed> $context */
    public static function refused(string $code, string $message, int $status, array $context = []): self
    {
        return new self(false, null, $code, $message, $status, $context);
    }
}
