<?php

namespace App\Event;

use App\Entity\EventRegistration;

/**
 * What came of a registration attempt.
 *
 * Modelled on BookingResult for the same reason: the caller needs to tell apart
 * "you're in", "you're queued" and "no" without parsing a message, and the
 * message itself has to be something a human can read straight out.
 */
final readonly class RegistrationResult
{
    private function __construct(
        public bool $ok,
        public ?EventRegistration $registration = null,
        public ?string $code = null,
        public ?string $message = null,
        public int $status = 200,
    ) {
    }

    public static function registered(EventRegistration $registration): self
    {
        return new self(true, $registration, 'REGISTERED', 'Votre inscription est confirmée.', 201);
    }

    public static function waitlisted(EventRegistration $registration): self
    {
        return new self(true, $registration, 'WAITLISTED', 'L\'événement est complet : vous êtes sur liste d\'attente et serez prévenu si une place se libère.', 201);
    }

    public static function cancelled(EventRegistration $registration): self
    {
        return new self(true, $registration, 'CANCELLED', 'Votre inscription a été annulée.', 200);
    }

    public static function refused(string $code, string $message, int $status = 409): self
    {
        return new self(false, null, $code, $message, $status);
    }

    /** True when the person ended up queued rather than seated. */
    public function isWaitlisted(): bool
    {
        return $this->code === 'WAITLISTED';
    }
}
