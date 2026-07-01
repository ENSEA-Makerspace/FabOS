<?php

namespace App\Entity;

use App\Repository\LogUtilisationRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: LogUtilisationRepository::class)]
#[ORM\Table(name: 'LOG_UTILISATION')]
class LogUtilisation
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Machine::class)]
    #[ORM\JoinColumn(name: 'machineId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Machine $machine = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Utilisateur $utilisateur = null;

    #[ORM\Column(name: 'dateDebut', type: 'datetime_immutable')]
    private \DateTimeImmutable $dateDebut;

    #[ORM\Column(name: 'dateFin', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $dateFin = null;

    #[ORM\Column(nullable: true)]
    private ?int $duree = null;

    #[ORM\Column(length: 50, nullable: true)]
    private ?string $source = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct() { $this->dateDebut = new \DateTimeImmutable(); $this->createdAt = new \DateTimeImmutable(); }
    public function getId(): ?int { return $this->id; }
    public function getMachine(): ?Machine { return $this->machine; }
    public function setMachine(?Machine $machine): self { $this->machine = $machine; return $this; }
    public function getUtilisateur(): ?Utilisateur { return $this->utilisateur; }
    public function setUtilisateur(?Utilisateur $utilisateur): self { $this->utilisateur = $utilisateur; return $this; }
    public function getUser(): ?Utilisateur { return $this->utilisateur; }
    public function setUser(?Utilisateur $user): self { return $this->setUtilisateur($user); }
    public function getDateDebut(): \DateTimeImmutable { return $this->dateDebut; }
    public function setDateDebut(\DateTimeImmutable $dateDebut): self { $this->dateDebut = $dateDebut; return $this; }
    public function getDateFin(): ?\DateTimeImmutable { return $this->dateFin; }
    public function setDateFin(?\DateTimeImmutable $dateFin): self { $this->dateFin = $dateFin; return $this; }
    public function getDuree(): ?int { return $this->duree; }
    public function setDuree(?int $duree): self { $this->duree = $duree; return $this; }
    public function getSource(): ?string { return $this->source; }
    public function setSource(?string $source): self { $this->source = $source; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }
}
