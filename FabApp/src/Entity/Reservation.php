<?php

namespace App\Entity;

use App\Repository\ReservationRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: ReservationRepository::class)]
#[ORM\Table(name: 'RESERVATION')]
class Reservation
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Utilisateur $utilisateur = null;

    #[ORM\ManyToOne(targetEntity: Machine::class)]
    #[ORM\JoinColumn(name: 'machineId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Machine $machine = null;

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
    public function getMachine(): ?Machine { return $this->machine; }
    public function setMachine(?Machine $machine): self { $this->machine = $machine; return $this; }
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
