<?php

namespace App\Entity;

use App\Repository\EventRegistrationRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * One person's place at an event — a member or a guest.
 *
 * The guest case is why `utilisateur` is nullable and why `contactEmail` is
 * not: the address is the identity here. It is filled from the account when
 * there is one, so both kinds of registrant are reachable by the same mail
 * code and countable by the same queries, and the unique index on
 * (event, contactEmail) can stop the same person registering twice regardless
 * of whether they were signed in at the time.
 */
#[ORM\Entity(repositoryClass: EventRegistrationRepository::class)]
#[ORM\Table(name: 'EVENT_REGISTRATION')]
class EventRegistration
{
    /** Holds a seat. */
    public const STATUS_REGISTERED = 'registered';

    /** Queued behind a full event; promoted automatically when a seat frees. */
    public const STATUS_WAITLISTED = 'waitlisted';

    public const STATUS_CANCELLED = 'cancelled';

    /** The statuses that occupy a seat — the single list every count filters on. */
    public const ACTIVE_STATUSES = [self::STATUS_REGISTERED];

    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Event::class)]
    #[ORM\JoinColumn(name: 'eventId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Event $event = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?Utilisateur $utilisateur = null;

    #[ORM\Column(name: 'guestName', length: 180, nullable: true)]
    private ?string $guestName = null;

    #[ORM\Column(name: 'contactEmail', length: 180)]
    private string $contactEmail = '';

    #[ORM\Column(length: 20)]
    private string $status = self::STATUS_REGISTERED;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;

    #[ORM\Column(name: 'cancelledAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $cancelledAt = null;

    #[ORM\Column(name: 'promotedAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $promotedAt = null;

    /**
     * When this person actually walked in. Null = not (yet) arrived.
     *
     * Separate from `status` on purpose: attendance and registration are two
     * different facts, and merging them would lose the difference between a
     * no-show and a cancellation.
     */
    #[ORM\Column(name: 'checkedInAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $checkedInAt = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'checkedInById', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?Utilisateur $checkedInBy = null;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }

    public function getEvent(): ?Event { return $this->event; }
    public function setEvent(?Event $event): self { $this->event = $event; return $this; }

    public function getUtilisateur(): ?Utilisateur { return $this->utilisateur; }
    public function setUtilisateur(?Utilisateur $utilisateur): self { $this->utilisateur = $utilisateur; return $this; }

    public function getGuestName(): ?string { return $this->guestName; }
    public function setGuestName(?string $guestName): self { $this->guestName = $guestName; return $this; }

    public function getContactEmail(): string { return $this->contactEmail; }
    public function setContactEmail(string $contactEmail): self { $this->contactEmail = self::normaliseEmail($contactEmail); return $this; }

    public function getStatus(): string { return $this->status; }
    public function setStatus(string $status): self { $this->status = $status; return $this; }

    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }

    public function getCancelledAt(): ?\DateTimeImmutable { return $this->cancelledAt; }
    public function setCancelledAt(?\DateTimeImmutable $cancelledAt): self { $this->cancelledAt = $cancelledAt; return $this; }

    public function getPromotedAt(): ?\DateTimeImmutable { return $this->promotedAt; }
    public function setPromotedAt(?\DateTimeImmutable $promotedAt): self { $this->promotedAt = $promotedAt; return $this; }

    public function getCheckedInAt(): ?\DateTimeImmutable { return $this->checkedInAt; }
    public function getCheckedInBy(): ?Utilisateur { return $this->checkedInBy; }
    public function isCheckedIn(): bool { return $this->checkedInAt !== null; }

    /** Marks arrival, recording who was on the door. */
    public function checkIn(?Utilisateur $by, ?\DateTimeImmutable $at = null): self
    {
        $this->checkedInAt = $at ?? new \DateTimeImmutable();
        $this->checkedInBy = $by;

        return $this;
    }

    /** Undoes a mistaken tap — the one thing a door desk always needs. */
    public function undoCheckIn(): self
    {
        $this->checkedInAt = null;
        $this->checkedInBy = null;

        return $this;
    }

    /**
     * Whether checking this person in makes sense: they must hold a seat.
     * Someone waitlisted has nothing to be admitted to until they're promoted.
     */
    public function isCheckInEligible(): bool
    {
        return $this->holdsSeat();
    }

    public function isRegistered(): bool { return $this->status === self::STATUS_REGISTERED; }
    public function isWaitlisted(): bool { return $this->status === self::STATUS_WAITLISTED; }
    public function isCancelled(): bool { return $this->status === self::STATUS_CANCELLED; }

    /** Whether this row currently occupies one of the event's seats. */
    public function holdsSeat(): bool
    {
        return in_array($this->status, self::ACTIVE_STATUSES, true);
    }

    /** The name to show an organiser, whoever registered. */
    public function getDisplayName(): string
    {
        if ($this->utilisateur !== null) {
            return $this->utilisateur->getDisplayName() ?: $this->utilisateur->getEmail();
        }

        return trim((string) $this->guestName) ?: $this->contactEmail;
    }

    public function isGuest(): bool
    {
        return $this->utilisateur === null;
    }

    /**
     * Lowercased and trimmed, because the unique index is what enforces "one
     * signup per person" and it compares bytes — Alice@example.org and
     * alice@example.org have to collide for that to mean anything.
     */
    public static function normaliseEmail(string $email): string
    {
        return mb_strtolower(trim($email));
    }
}
