<?php

namespace App\Entity;

use App\Repository\BadgeRepository;
use Doctrine\Common\Collections\ArrayCollection;
use Doctrine\Common\Collections\Collection;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: BadgeRepository::class)]
#[ORM\Table(name: 'BADGE')]
class Badge
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
    private ?string $icone = null;

    #[ORM\Column(name: 'whereRecognized', length: 255, nullable: true)]
    private ?string $whereRecognized = null;

    #[ORM\ManyToOne(targetEntity: self::class)]
    #[ORM\JoinColumn(name: 'prerequisiteBadgeId', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?self $prerequisiteBadge = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    /**  Collection<int, MachineBadge> */
    #[ORM\OneToMany(mappedBy: 'badge', targetEntity: MachineBadge::class)]
    private Collection $machineBadges;

    /** Collection<int, Badge> */
    #[ORM\OneToMany(mappedBy: 'prerequisiteBadge', targetEntity: self::class)]
    private Collection $unlockedBadges;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
        $this->machineBadges = new ArrayCollection();
        $this->unlockedBadges = new ArrayCollection();
    }
    public function getId(): ?int { return $this->id; }
    public function getNom(): string { return $this->nom; }
    public function setNom(string $nom): self { $this->nom = $nom; return $this; }
    public function getName(): string { return $this->nom; }
    public function setName(string $name): self { return $this->setNom($name); }
    public function getDescription(): ?string { return $this->description; }
    public function setDescription(?string $description): self { $this->description = $description; return $this; }
    public function getIcone(): ?string { return $this->icone; }
    public function setIcone(?string $icone): self { $this->icone = $icone; return $this; }
    public function getIcon(): ?string { return $this->icone; }
    public function setIcon(?string $icon): self { return $this->setIcone($icon); }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }
    /**  Collection<int, MachineBadge> */
    public function getMachineBadges(): Collection { return $this->machineBadges; }
    public function getWhereRecognized(): ?string { return $this->whereRecognized; }
    public function setWhereRecognized(?string $whereRecognized): self { $this->whereRecognized = $whereRecognized; return $this; }
    public function getPrerequisiteBadge(): ?self { return $this->prerequisiteBadge; }
    public function setPrerequisiteBadge(?self $prerequisiteBadge): self { $this->prerequisiteBadge = $prerequisiteBadge; return $this; }
    /** @return Collection<int, Badge> */
    public function getUnlockedBadges(): Collection { return $this->unlockedBadges; }
}
