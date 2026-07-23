<?php

namespace App\Entity;

use App\Repository\ReservationRepository;
use App\Reservation\ReservableType;
use Doctrine\ORM\Mapping as ORM;

/**
 * A booking of one reservable resource for one user over one time range.
 *
 * The resource is addressed polymorphically by (reservableType, reservableId)
 * so any kind of resource is bookable without another nullable FK. Resolve one
 * back to a displayable resource with ReservableResolver; book through
 * ReservationService, never by constructing this directly.
 *
 * reservableLabel snapshots the resource name at booking time. Because there is
 * no FK there is no cascade: deleting a machine or space leaves its bookings
 * behind, and the snapshot is what keeps them readable. Callers that delete a
 * resource must cancel its upcoming bookings themselves — see
 * ReservationRepository::cancelUpcomingForReservable().
 */
#[ORM\Entity(repositoryClass: ReservationRepository::class)]
#[ORM\Table(name: 'RESERVATION')]
#[ORM\Index(name: 'IDX_RESERVATION_RESERVABLE', columns: ['reservableType', 'reservableId', 'dateDebut'])]
class Reservation
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Utilisateur $utilisateur = null;

    #[ORM\Column(name: 'reservableType', length: 20)]
    private string $reservableType = '';

    #[ORM\Column(name: 'reservableId')]
    private int $reservableId = 0;

    #[ORM\Column(name: 'reservableLabel', length: 190, nullable: true)]
    private ?string $reservableLabel = null;

    #[ORM\Column(name: 'dateDebut', type: 'datetime_immutable')]
    private \DateTimeImmutable $dateDebut;

    #[ORM\Column(name: 'dateFin', type: 'datetime_immutable')]
    private \DateTimeImmutable $dateFin;

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $motif = null;

    #[ORM\Column(length: 30, options: ['default' => 'confirmed'])]
    private string $statut = 'confirmed';

    #[ORM\Column(name: 'created', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $created;

    public function __construct() { $this->dateDebut = new \DateTimeImmutable(); $this->dateFin = new \DateTimeImmutable('+1 hour'); $this->created = new \DateTimeImmutable(); }
    public function getId(): ?int { return $this->id; }
    public function getUtilisateur(): ?Utilisateur { return $this->utilisateur; }
    public function setUtilisateur(?Utilisateur $utilisateur): self { $this->utilisateur = $utilisateur; return $this; }
    public function getUser(): ?Utilisateur { return $this->utilisateur; }
    public function setUser(?Utilisateur $user): self { return $this->setUtilisateur($user); }
    public function getReservableType(): ?ReservableType { return ReservableType::tryParse($this->reservableType); }
    public function getReservableId(): ?int { return $this->reservableId ?: null; }
    public function getReservableLabel(): ?string { return $this->reservableLabel; }

    /**
     * Point this reservation at a resource. $label is the resource name as it
     * reads now — snapshotted so the row survives the resource being deleted.
     */
    public function setReservable(ReservableType $type, int $id, ?string $label = null): self
    {
        $this->reservableType = $type->value;
        $this->reservableId = $id;
        $this->reservableLabel = $label === null ? null : mb_substr($label, 0, 190);

        return $this;
    }

    public function isFor(ReservableType $type, int $id): bool
    {
        return $this->reservableType === $type->value && $this->reservableId === $id;
    }
    public function getDateDebut(): \DateTimeImmutable { return $this->dateDebut; }
    public function setDateDebut(\DateTimeImmutable $dateDebut): self { $this->dateDebut = $dateDebut; return $this; }
    public function getStartAt(): \DateTimeImmutable { return $this->dateDebut; }
    public function setStartAt(\DateTimeImmutable $startAt): self { return $this->setDateDebut($startAt); }
    public function getDateFin(): \DateTimeImmutable { return $this->dateFin; }
    public function setDateFin(\DateTimeImmutable $dateFin): self { $this->dateFin = $dateFin; return $this; }
    public function getEndAt(): \DateTimeImmutable { return $this->dateFin; }
    public function setEndAt(\DateTimeImmutable $endAt): self { return $this->setDateFin($endAt); }
    public function getMotif(): ?string { return $this->motif; }
    public function setMotif(?string $motif): self { $this->motif = $motif; return $this; }
    public function getComment(): ?string { return $this->motif; }
    public function setComment(?string $comment): self { return $this->setMotif($comment); }
    public function getStatut(): string { return $this->statut; }
    public function setStatut(string $statut): self { $this->statut = $statut; return $this; }
    public function getStatus(): string { return $this->statut; }
    public function setStatus(string $status): self { return $this->setStatut($status); }
    public function isCancelled(): bool { return $this->statut === 'cancelled'; }
    public function cancel(): self { return $this->setStatut('cancelled'); }
    public function getCreated(): \DateTimeImmutable { return $this->created; }
    public function setCreated(\DateTimeImmutable $created): self { $this->created = $created; return $this; }
}
