<?php

namespace App\Entity;

use App\Repository\MaterialRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * A catalogue entry for a significant lab material (PLA/ABS/TPU filament, sheet
 * stock, resin…). Defined once, later reused by every machine that accepts it
 * and every training that requires it (those M2M links land in a follow-up
 * slice). Trivial consumables (glue, paper) stay as free text elsewhere.
 */
#[ORM\Entity(repositoryClass: MaterialRepository::class)]
#[ORM\Table(name: 'MATERIAL')]
class Material
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 150)]
    private string $name = '';

    /** Grouping/filter bucket, e.g. "filament", "sheet", "resin". */
    #[ORM\Column(length: 80, nullable: true)]
    private ?string $category = null;

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $description = null;

    /** Optional external image URL; takes precedence over the icon when set. */
    #[ORM\Column(name: 'imageUrl', length: 500, nullable: true)]
    private ?string $imageUrl = null;

    /** Optional emoji / short glyph shown when there is no image. */
    #[ORM\Column(length: 16, nullable: true)]
    private ?string $icon = null;

    /** Sizes/specs as a free "key: value" list (one per line) for v1. */
    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $specs = null;

    #[ORM\Column(name: 'storageLocation', length: 180, nullable: true)]
    private ?string $storageLocation = null;

    #[ORM\Column(name: 'purchaseUrl', length: 500, nullable: true)]
    private ?string $purchaseUrl = null;

    #[ORM\Column(length: 60, nullable: true)]
    private ?string $color = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getName(): string { return $this->name; }
    public function setName(string $name): self { $this->name = $name; return $this; }
    public function getCategory(): ?string { return $this->category; }
    public function setCategory(?string $category): self { $this->category = $category; return $this; }
    public function getDescription(): ?string { return $this->description; }
    public function setDescription(?string $description): self { $this->description = $description; return $this; }
    public function getImageUrl(): ?string { return $this->imageUrl; }
    public function setImageUrl(?string $imageUrl): self { $this->imageUrl = $imageUrl; return $this; }
    public function getIcon(): ?string { return $this->icon; }
    public function setIcon(?string $icon): self { $this->icon = $icon; return $this; }
    public function getSpecs(): ?string { return $this->specs; }
    public function setSpecs(?string $specs): self { $this->specs = $specs; return $this; }
    public function getStorageLocation(): ?string { return $this->storageLocation; }
    public function setStorageLocation(?string $storageLocation): self { $this->storageLocation = $storageLocation; return $this; }
    public function getPurchaseUrl(): ?string { return $this->purchaseUrl; }
    public function setPurchaseUrl(?string $purchaseUrl): self { $this->purchaseUrl = $purchaseUrl; return $this; }
    public function getColor(): ?string { return $this->color; }
    public function setColor(?string $color): self { $this->color = $color; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }

    /**
     * Specs parsed into [label, value] pairs from the free "key: value" text,
     * for tidy rendering. Lines without a colon become a value with no label.
     *
     * @return array<int, array{label: string, value: string}>
     */
    public function getSpecPairs(): array
    {
        $pairs = [];
        foreach (preg_split('/\r\n|\r|\n/', (string) $this->specs) ?: [] as $line) {
            $line = trim($line);
            if ($line === '') {
                continue;
            }
            $parts = explode(':', $line, 2);
            if (count($parts) === 2) {
                $pairs[] = ['label' => trim($parts[0]), 'value' => trim($parts[1])];
            } else {
                $pairs[] = ['label' => '', 'value' => $line];
            }
        }

        return $pairs;
    }
}
