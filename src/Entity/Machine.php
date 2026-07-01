<?php

namespace App\Entity;

use App\Repository\MachineRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: MachineRepository::class)]
#[ORM\Table(name: 'MACHINE')]
class Machine
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 255)]
    private string $nom = '';

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $description = null;

    #[ORM\Column(length: 255, nullable: true)]
    private ?string $localisation = null;

    #[ORM\Column(length: 255, nullable: true)]
    private ?string $photo = null;

    #[ORM\Column(length: 50, options: ['default' => 'idle'])]
    private string $statut = 'idle';

    #[ORM\Column(length: 50, nullable: true)]
    private ?string $granularite = null;

    #[ORM\Column(name: 'limiteReservations', options: ['default' => 0])]
    private int $limiteReservations = 0;

    #[ORM\Column(name: 'machineToken', length: 255, unique: true)]
    private string $machineToken = '';

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    #[ORM\Column(name: 'updated', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $updated;

    #[ORM\Column(name: 'lastAuthorizationTime', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $lastAuthorizationTime = null;

    public function __construct() { $this->createdAt = new \DateTimeImmutable(); $this->updated = new \DateTimeImmutable(); }
    public function getId(): ?int { return $this->id; }
    public function getNom(): string { return $this->nom; }
    public function setNom(string $nom): self { $this->nom = $nom; return $this; }
    public function getName(): string { return $this->nom; }
    public function setName(string $name): self { return $this->setNom($name); }
    public function getDescription(): ?string { return $this->description; }
    public function setDescription(?string $description): self { $this->description = $description; return $this; }
    public function getLocalisation(): ?string { return $this->localisation; }
    public function setLocalisation(?string $localisation): self { $this->localisation = $localisation; return $this; }
    public function getLocation(): ?string { return $this->localisation; }
    public function setLocation(?string $location): self { return $this->setLocalisation($location); }
    public function getPhoto(): ?string { return $this->photo; }
    public function setPhoto(?string $photo): self { $this->photo = $photo; return $this; }
    public function getImage(): ?string { return $this->photo; }
    public function setImage(?string $image): self { return $this->setPhoto($image); }
    public function getStatut(): string { return $this->statut; }
    public function setStatut(string $statut): self { $this->statut = $statut; return $this; }
    public function getStatus(): string { return $this->statut; }
    public function setStatus(string $status): self { return $this->setStatut($status); }
    public function getGranularite(): ?string { return $this->granularite; }
    public function setGranularite(?string $granularite): self { $this->granularite = $granularite; return $this; }
    public function getLimiteReservations(): int { return $this->limiteReservations; }
    public function setLimiteReservations(int $limiteReservations): self { $this->limiteReservations = $limiteReservations; return $this; }
    public function getMachineToken(): string { return $this->machineToken; }
    public function setMachineToken(string $machineToken): self { $this->machineToken = $machineToken; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }
    public function getUpdated(): \DateTimeImmutable { return $this->updated; }
    public function setUpdated(\DateTimeImmutable $updated): self { $this->updated = $updated; return $this; }
    public function getLastAuthorizationTime(): ?\DateTimeImmutable { return $this->lastAuthorizationTime; }
    public function setLastAuthorizationTime(?\DateTimeImmutable $lastAuthorizationTime): self { $this->lastAuthorizationTime = $lastAuthorizationTime; return $this; }
}
