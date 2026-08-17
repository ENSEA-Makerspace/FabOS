<?php

declare(strict_types=1);

namespace App\UsageRights;

/**
 * What is being asked of a grant — the question, not the answer (S144b).
 *
 * Grants gained dimensions one session at a time: an action and a section in
 * S133b, a location in S134b, and in S144b a **resource** and a **time of week**.
 * Every one of them arrived as another positional argument threaded through
 * `verdict()` into `paths()`, and the venue one spent two sessions being passed
 * as a literal `null` by every caller — a restriction that was stored, read, and
 * could never refuse anything. Eight positional nullables is how that happens
 * twice.
 *
 * ⚠️ **Null means "the caller is not asking", never "no restriction".** Those are
 * different sentences and the SQL says so: `g.venueId IS NULL` is an unrestricted
 * grant, `:venue IS NULL` is an overview that has no location in hand. A caller
 * with nothing to say about a dimension must not be refused on it — refusing a
 * question nobody asked is worse than answering it broadly.
 */
final readonly class UsageScope
{
    public function __construct(
        /** The location of the thing being acted on. */
        public ?int $venueId = null,
        /** `ReservableType->value` — machine, place, user. */
        public ?string $reservableType = null,
        /** One specific resource, when the caller knows which. */
        public ?int $reservableId = null,
        /** `MACHINE.categoryLabel` — how "the 3D printers" is addressed. */
        public ?string $categoryLabel = null,
        public ?\DateTimeImmutable $from = null,
        public ?\DateTimeImmutable $until = null,
    ) {
    }

    /** An overview: does this person hold this at all, anywhere, at any time. */
    public static function any(?\DateTimeImmutable $at = null): self
    {
        return new self(from: $at);
    }

    /**
     * ⚠️ **Only an explicit interval evaluates a grant's opening windows.**
     * A booking says "Monday 14:00 to 16:00" and gets the window enforced. An
     * overview says nothing about time — and answering it with "denied, it is
     * Tuesday" would make a member's rights page flap between allowed and
     * refused as the week turns, which is not what that page is answering. The
     * windows are shown there in words instead.
     */
    public function isTimed(): bool
    {
        return $this->from !== null && $this->until !== null && $this->until > $this->from;
    }

    /** The instant an assignment's validity is judged at. */
    public function at(): \DateTimeImmutable
    {
        return $this->from ?? new \DateTimeImmutable();
    }
}
