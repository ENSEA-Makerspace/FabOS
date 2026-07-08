<?php

namespace App\Entity;

use App\Repository\AccessRfidLogRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: AccessRfidLogRepository::class)]
#[ORM\Table(name: 'ACCESS_RFID_LOG')]
class AccessRfidLog
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(name: 'badgeUid', length: 255)]
    private string $badgeUid = '';

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?Utilisateur $utilisateur = null;

    #[ORM\ManyToOne(targetEntity: Machine::class)]
    #[ORM\JoinColumn(name: 'machineId', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?Machine $machine = null;

    #[ORM\ManyToOne(targetEntity: RfidReader::class)]
    #[ORM\JoinColumn(name: 'readerId', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?RfidReader $reader = null;

    #[ORM\Column(name: 'readerToken', length: 120, nullable: true)]
    private ?string $readerToken = null;

    #[ORM\Column(options: ['default' => false])]
    private bool $authorized = false;

    #[ORM\Column(length: 100)]
    private string $status = '';

    #[ORM\Column(length: 255, nullable: true)]
    private ?string $reason = null;

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $message = null;

    #[ORM\Column(length: 50, nullable: true)]
    private ?string $color = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct() { $this->createdAt = new \DateTimeImmutable(); }
    public function getId(): ?int { return $this->id; }
    public function getBadgeUid(): string { return $this->badgeUid; }
    public function setBadgeUid(string $badgeUid): self { $this->badgeUid = $badgeUid; return $this; }
    public function getUtilisateur(): ?Utilisateur { return $this->utilisateur; }
    public function setUtilisateur(?Utilisateur $utilisateur): self { $this->utilisateur = $utilisateur; return $this; }
    public function getUser(): ?Utilisateur { return $this->utilisateur; }
    public function setUser(?Utilisateur $user): self { return $this->setUtilisateur($user); }
    public function getMachine(): ?Machine { return $this->machine; }
    public function setMachine(?Machine $machine): self { $this->machine = $machine; return $this; }
    public function getReader(): ?RfidReader { return $this->reader; }
    public function setReader(?RfidReader $reader): self { $this->reader = $reader; return $this; }
    public function getReaderToken(): ?string { return $this->readerToken; }
    public function setReaderToken(?string $readerToken): self { $this->readerToken = $readerToken; return $this; }
    public function isAuthorized(): bool { return $this->authorized; }
    public function setAuthorized(bool $authorized): self { $this->authorized = $authorized; return $this; }
    public function getStatus(): string { return $this->status; }
    public function setStatus(string $status): self { $this->status = $status; return $this; }
    public function getReason(): ?string { return $this->reason; }
    public function setReason(?string $reason): self { $this->reason = $reason; return $this; }
    public function getMessage(): ?string { return $this->message; }
    public function setMessage(?string $message): self { $this->message = $message; return $this; }
    public function getColor(): ?string { return $this->color; }
    public function setColor(?string $color): self { $this->color = $color; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }
}
