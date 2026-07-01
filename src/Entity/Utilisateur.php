<?php

namespace App\Entity;

use App\Repository\UtilisateurRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: UtilisateurRepository::class)]
#[ORM\Table(name: 'UTILISATEUR')]
class Utilisateur
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 255, unique: true)]
    private string $email = '';

    #[ORM\Column(length: 255, unique: true)]
    private string $username = '';

    #[ORM\Column(length: 255)]
    private string $password = '';

    #[ORM\Column(name: 'firstName', length: 255, nullable: true)]
    private ?string $firstName = null;

    #[ORM\Column(name: 'lastName', length: 255, nullable: true)]
    private ?string $lastName = null;

    #[ORM\Column(length: 50, options: ['default' => 'actif'])]
    private string $statut = 'actif';

    #[ORM\Column(name: 'identifiantRfid', length: 255, unique: true, nullable: true)]
    private ?string $identifiantRfid = null;

    #[ORM\Column(name: 'tempsPresenceTotal', options: ['default' => 0])]
    private int $tempsPresenceTotal = 0;

    #[ORM\Column(name: 'isVerified', options: ['default' => false])]
    private bool $isVerified = false;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct() { $this->createdAt = new \DateTimeImmutable(); }
    public function getId(): ?int { return $this->id; }
    public function getEmail(): string { return $this->email; }
    public function setEmail(string $email): self { $this->email = $email; return $this; }
    public function getUsername(): string { return $this->username; }
    public function setUsername(string $username): self { $this->username = $username; return $this; }
    public function getPassword(): string { return $this->password; }
    public function setPassword(string $password): self { $this->password = $password; return $this; }
    public function getFirstName(): ?string { return $this->firstName; }
    public function setFirstName(?string $firstName): self { $this->firstName = $firstName; return $this; }
    public function getLastName(): ?string { return $this->lastName; }
    public function setLastName(?string $lastName): self { $this->lastName = $lastName; return $this; }
    public function getDisplayName(): string { return trim(($this->firstName ?? '') . ' ' . ($this->lastName ?? '')) ?: $this->username; }
    public function getStatut(): string { return $this->statut; }
    public function setStatut(string $statut): self { $this->statut = $statut; return $this; }
    public function getStatus(): string { return $this->statut; }
    public function getIdentifiantRfid(): ?string { return $this->identifiantRfid; }
    public function setIdentifiantRfid(?string $identifiantRfid): self { $this->identifiantRfid = $identifiantRfid; return $this; }
    public function getTempsPresenceTotal(): int { return $this->tempsPresenceTotal; }
    public function setTempsPresenceTotal(int $tempsPresenceTotal): self { $this->tempsPresenceTotal = $tempsPresenceTotal; return $this; }
    public function getPoints(): int { return $this->tempsPresenceTotal; }
    public function setPoints(int $points): self { return $this->setTempsPresenceTotal($points); }
    public function isVerified(): bool { return $this->isVerified; }
    public function setIsVerified(bool $isVerified): self { $this->isVerified = $isVerified; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }
}
