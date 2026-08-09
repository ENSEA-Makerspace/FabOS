<?php

namespace App\Entity;

use App\Repository\MachineRepository;
use Doctrine\Common\Collections\ArrayCollection;
use Doctrine\Common\Collections\Collection;
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

    #[ORM\Column(name: 'categorySlug', length: 100, nullable: true)]
    private ?string $categorySlug = null;

    #[ORM\Column(name: 'categoryLabel', length: 100, nullable: true)]
    private ?string $categoryLabel = null;

    #[ORM\Column(name: 'levelSlug', length: 50, nullable: true)]
    private ?string $levelSlug = null;

    #[ORM\Column(name: 'levelLabel', length: 50, nullable: true)]
    private ?string $levelLabel = null;

    #[ORM\Column(name: 'iconSlug', length: 50, nullable: true)]
    private ?string $iconSlug = null;

    #[ORM\Column(type: 'json', nullable: true)]
    private ?array $materials = null;

    #[ORM\Column(type: 'json', nullable: true)]
    private ?array $features = null;

    #[ORM\Column(name: 'requirementTitle', length: 255, nullable: true)]
    private ?string $requirementTitle = null;

    #[ORM\Column(name: 'requirementDescription', type: 'text', nullable: true)]
    private ?string $requirementDescription = null;

    #[ORM\Column(nullable: true)]
    private ?int $popularity = null;

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


    /** @var Collection<int, MachineBadge> */
    #[ORM\OneToMany(mappedBy: 'machine', targetEntity: MachineBadge::class)]
    private Collection $machineBadges;

    public function __construct() { $this->createdAt = new \DateTimeImmutable(); $this->updated = new \DateTimeImmutable(); $this->machineBadges = new ArrayCollection(); }
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
    public function getGranularite(): ?string { return self::normalizeGranularite($this->granularite); }
    public function setGranularite(?string $granularite): self { $this->granularite = self::normalizeGranularite($granularite); return $this; }
    public function getGranulariteMinutes(): int { $normalized = self::normalizeGranularite($this->granularite); return $normalized !== null ? (int) $normalized : 60; }
    public function getGranulariteLabel(): string { $minutes = $this->getGranulariteMinutes(); return $minutes >= 60 && $minutes % 60 === 0 ? (string) ($minutes / 60) . ' h' : $minutes . ' min'; }
    public function getCategorySlug(): string { return $this->categorySlug ?: 'impression-3d'; }
    /** Raw stored value; use when distinguishing legacy fallback data during migrations. */
    public function getStoredCategorySlug(): ?string { return $this->categorySlug; }
    public function setCategorySlug(?string $categorySlug): self { $this->categorySlug = $categorySlug; return $this; }
    public function getCategoryLabel(): string { return $this->categoryLabel ?: 'Impression 3D'; }
    /** Raw stored value; use when distinguishing legacy fallback data during migrations. */
    public function getStoredCategoryLabel(): ?string { return $this->categoryLabel; }
    public function setCategoryLabel(?string $categoryLabel): self { $this->categoryLabel = $categoryLabel; return $this; }
    public function getLevelSlug(): string { return $this->levelSlug ?: 'niveau-1'; }
    public function setLevelSlug(?string $levelSlug): self { $this->levelSlug = $levelSlug; return $this; }
    public function getLevelLabel(): string { return $this->levelLabel ?: 'Niveau 1'; }
    public function setLevelLabel(?string $levelLabel): self { $this->levelLabel = $levelLabel; return $this; }
    public function getIconSlug(): string { return $this->iconSlug ?: 'impression-3d'; }
    public function setIconSlug(?string $iconSlug): self { $this->iconSlug = $iconSlug; return $this; }
    public function getMaterials(): array { return $this->materials ?: ['PLA', 'PETG', 'TPU', 'Support']; }
    public function setMaterials(?array $materials): self { $this->materials = $materials; return $this; }
    public function getFeatures(): array { return $this->features ?: ['Volume utile standard FabLab', 'Réservation par créneau', 'Utilisation accompagnée possible', 'Traçabilité RFID']; }
    public function setFeatures(?array $features): self { $this->features = $features; return $this; }
    public function getRequirementTitle(): string { return $this->requirementTitle ?: 'Formation imprimante 3D recommandée'; }
    public function setRequirementTitle(?string $requirementTitle): self { $this->requirementTitle = $requirementTitle; return $this; }
    public function getRequirementDescription(): string { return $this->requirementDescription ?: 'Validation formation ou accompagnement staff conseillé avant première utilisation.'; }
    public function setRequirementDescription(?string $requirementDescription): self { $this->requirementDescription = $requirementDescription; return $this; }
    public function getPopularity(): int { return $this->popularity ?? 4; }
    public function setPopularity(?int $popularity): self { $this->popularity = $popularity; return $this; }
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

    /** @return Collection<int, MachineBadge> */
    public function getMachineBadges(): Collection { return $this->machineBadges; }
    /** @return MachineBadge[] */
    public function getRequiredMachineBadges(): array { return $this->machineBadges->filter(static fn (MachineBadge $machineBadge): bool => $machineBadge->isRequiredForAccess())->toArray(); }

    private static function normalizeGranularite(?string $granularite): ?string
    {
        $granularite = trim((string) $granularite);
        if ($granularite === '') {
            return null;
        }

        if (preg_match('/^(\d+)\s*(?:m|min|minutes?)?$/i', $granularite, $matches) !== 1) {
            return null;
        }

        $minutes = (int) $matches[1];

        return $minutes > 0 ? (string) $minutes : null;
    }
}
