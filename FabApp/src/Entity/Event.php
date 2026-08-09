<?php

namespace App\Entity;

use App\Repository\EventRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * A dated event (workshop, open day, meetup…). Display-only: shown on the home
 * page, the calendar and a kiosk screen, but not booked like a reservation.
 * Table is named EVENEMENT because EVENT is a reserved word in MySQL.
 */
#[ORM\Entity(repositoryClass: EventRepository::class)]
#[ORM\Table(name: 'EVENEMENT')]
class Event
{
    /** At the lab: the address comes from the site settings, not from the event. */
    public const LOCATION_ONSITE = 'onsite';

    /** Somewhere else: the event carries its own address. */
    public const LOCATION_OFFSITE = 'offsite';

    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 180)]
    private string $titre = '';

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $description = null;

    #[ORM\Column(name: 'dateDebut', type: 'datetime_immutable')]
    private ?\DateTimeImmutable $dateDebut = null;

    #[ORM\Column(name: 'dateFin', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $dateFin = null;

    #[ORM\Column(length: 180, nullable: true)]
    private ?string $lieu = null;

    /** Null means unlimited, matching how quotas and access passes read. */
    #[ORM\Column(nullable: true)]
    private ?int $capacite = null;

    /**
     * Whether people without an account may register. Restricting an event
     * after the fact does not evict guests who already have a place.
     */
    #[ORM\Column(name: 'guestsAllowed', options: ['default' => true])]
    private bool $guestsAllowed = true;

    #[ORM\Column(name: 'posterFilename', length: 255, nullable: true)]
    private ?string $posterFilename = null;

    /**
     * Where it happens. On site borrows the lab's own address from the site
     * settings; off site carries its own. Directions are derived from whichever
     * applies rather than stored, so they can never go stale against the address.
     */
    #[ORM\Column(name: 'locationMode', length: 20, options: ['default' => self::LOCATION_ONSITE])]
    private string $locationMode = self::LOCATION_ONSITE;

    #[ORM\Column(length: 500, nullable: true)]
    private ?string $address = null;

    #[ORM\Column(name: 'cancelledAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $cancelledAt = null;

    #[ORM\Column(name: 'cancellationReason', length: 500, nullable: true)]
    private ?string $cancellationReason = null;

    /** External or online events intentionally have no physical venue. */
    #[ORM\ManyToOne(targetEntity: Venue::class)]
    #[ORM\JoinColumn(name: 'venueId', nullable: true, onDelete: 'SET NULL')]
    private ?Venue $venue = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getPosterFilename(): ?string { return $this->posterFilename; }
    public function setPosterFilename(?string $posterFilename): self { $this->posterFilename = $posterFilename; return $this; }
    public function hasPoster(): bool { return $this->posterFilename !== null && $this->posterFilename !== ''; }

    public function getLocationMode(): string { return $this->locationMode; }
    public function setLocationMode(string $mode): self { $this->locationMode = $mode === self::LOCATION_OFFSITE ? self::LOCATION_OFFSITE : self::LOCATION_ONSITE; return $this; }
    public function isOnsite(): bool { return $this->locationMode !== self::LOCATION_OFFSITE; }

    public function getAddress(): ?string { return $this->address; }
    public function setAddress(?string $address): self { $address = $address !== null ? trim($address) : ''; $this->address = $address !== '' ? $address : null; return $this; }
    public function getTitre(): string { return $this->titre; }
    public function setTitre(string $titre): self { $this->titre = $titre; return $this; }
    public function getDescription(): ?string { return $this->description; }
    public function setDescription(?string $description): self { $this->description = $description; return $this; }
    public function getDateDebut(): ?\DateTimeImmutable { return $this->dateDebut; }
    public function setDateDebut(?\DateTimeImmutable $dateDebut): self { $this->dateDebut = $dateDebut; return $this; }
    public function getDateFin(): ?\DateTimeImmutable { return $this->dateFin; }
    public function setDateFin(?\DateTimeImmutable $dateFin): self { $this->dateFin = $dateFin; return $this; }
    public function getLieu(): ?string { return $this->lieu; }
    public function setLieu(?string $lieu): self { $this->lieu = $lieu; return $this; }
    public function getCapacite(): ?int { return $this->capacite; }
    public function setCapacite(?int $capacite): self { $this->capacite = $capacite !== null ? max(0, $capacite) : null; return $this; }
    public function hasCapacityLimit(): bool { return $this->capacite !== null; }
    public function isGuestsAllowed(): bool { return $this->guestsAllowed; }
    public function setGuestsAllowed(bool $guestsAllowed): self { $this->guestsAllowed = $guestsAllowed; return $this; }

    public function getCancelledAt(): ?\DateTimeImmutable { return $this->cancelledAt; }
    public function getCancellationReason(): ?string { return $this->cancellationReason; }
    public function isCancelled(): bool { return $this->cancelledAt !== null; }

    /** Called off, with the explanation registrants were given. */
    public function callOff(?string $reason, ?\DateTimeImmutable $at = null): self
    {
        $this->cancelledAt = $at ?? new \DateTimeImmutable();
        $reason = $reason !== null ? trim($reason) : '';
        $this->cancellationReason = $reason !== '' ? mb_substr($reason, 0, 500) : null;

        return $this;
    }

    /** Whether registration is still open — not started, and not called off. */
    public function isRegistrationOpen(?\DateTimeImmutable $now = null): bool
    {
        if ($this->dateDebut === null || $this->isCancelled()) {
            return false;
        }

        return $this->dateDebut > ($now ?? new \DateTimeImmutable());
    }

    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function getVenue(): ?Venue { return $this->venue; }
    public function setVenue(?Venue $venue): self { $this->venue = $venue; return $this; }
}
