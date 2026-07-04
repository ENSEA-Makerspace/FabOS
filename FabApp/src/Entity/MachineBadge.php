<?php

namespace App\Entity;

use App\Repository\MachineBadgeRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: MachineBadgeRepository::class)]
#[ORM\Table(name: 'MACHINE_BADGE')]
#[ORM\UniqueConstraint(name: 'unique_machine_badge', columns: ['machineId', 'badgeId'])]
class MachineBadge
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Machine::class, inversedBy: 'machineBadges')]
    #[ORM\JoinColumn(name: 'machineId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Machine $machine = null;

    #[ORM\ManyToOne(targetEntity: Badge::class, inversedBy: 'machineBadges')]
    #[ORM\JoinColumn(name: 'badgeId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Badge $badge = null;

    #[ORM\Column(name: 'requiredForAccess', options: ['default' => true])]
    private bool $requiredForAccess = true;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getMachine(): ?Machine { return $this->machine; }
    public function setMachine(?Machine $machine): self { $this->machine = $machine; return $this; }
    public function getBadge(): ?Badge { return $this->badge; }
    public function setBadge(?Badge $badge): self { $this->badge = $badge; return $this; }
    public function isRequiredForAccess(): bool { return $this->requiredForAccess; }
    public function setRequiredForAccess(bool $requiredForAccess): self { $this->requiredForAccess = $requiredForAccess; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }
}
