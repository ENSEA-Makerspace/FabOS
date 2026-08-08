<?php

namespace App\Reservation\Verb;

use App\Reservation\BookingResult;

/**
 * Whether one verb is available on one booking, and — the part that matters —
 * **why not, in words, when it isn't**.
 *
 * A plain boolean would have been enough for the endpoints and useless for the
 * page. `/mes-reservations` shipped with cancel rendered only when it applied,
 * so a member whose list was mostly past saw a page with no controls at all and
 * concluded there were none. An absent button and an impossible action look
 * identical. This carries the sentence that tells them apart.
 *
 * ⚠️ `$explain` is not the same as `!$allowed`. Every refusal has a message,
 * because the JSON API should answer a curl with a sentence; only some deserve
 * screen space. "This booking is finished" next to a card already labelled
 * *Passée* is noise, and noise on every past card is how the useful sentences
 * get skipped. Refusals worth reading are the ones a member would otherwise
 * mistake for a missing feature: the lock window, and "it has already started".
 */
final readonly class VerbVerdict
{
    private function __construct(
        public BookingVerb $verb,
        public bool $allowed,
        public ?string $code = null,
        public ?string $message = null,
        public int $status = 200,
        public bool $explain = false,
    ) {
    }

    public static function allow(BookingVerb $verb): self
    {
        return new self($verb, true);
    }

    /** Refused, and the member is not told — the card's state already says it. */
    public static function quietly(BookingVerb $verb, string $code, string $message, int $status = 409): self
    {
        return new self($verb, false, $code, $message, $status);
    }

    /** Refused, and the reason goes on screen where the control would have been. */
    public static function explained(BookingVerb $verb, string $code, string $message, int $status = 409): self
    {
        return new self($verb, false, $code, $message, $status, true);
    }

    /** The refusal in the shape every booking endpoint already returns. */
    public function toRefusal(): BookingResult
    {
        return BookingResult::refused(
            $this->code ?? 'VERB_UNAVAILABLE',
            $this->message ?? 'Action indisponible.',
            $this->status,
        );
    }
}
