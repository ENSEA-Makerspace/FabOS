<?php

namespace App\Entity;

use App\Repository\LoanableItemRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * A catalogue entry for something the lab lends out (a tool, a kit, a piece of
 * equipment). Individual checkouts are tracked as Loan records referencing this
 * item. `quantity` is how many units the lab owns (some items stack).
 */
#[ORM\Entity(repositoryClass: LoanableItemRepository::class)]
#[ORM\Table(name: 'LOANABLE_ITEM')]
class LoanableItem
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 150)]
    private string $name = '';

    #[ORM\Column(length: 80, nullable: true)]
    private ?string $category = null;

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $description = null;

    #[ORM\Column(name: 'imageUrl', length: 500, nullable: true)]
    private ?string $imageUrl = null;

    #[ORM\Column(length: 16, nullable: true)]
    private ?string $icon = null;

    #[ORM\Column(options: ['default' => 1])]
    private int $quantity = 1;

    #[ORM\Column(name: 'storageLocation', length: 180, nullable: true)]
    private ?string $storageLocation = null;

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
    public function getQuantity(): int { return $this->quantity; }
    public function setQuantity(int $quantity): self { $this->quantity = max(0, $quantity); return $this; }
    public function getStorageLocation(): ?string { return $this->storageLocation; }
    public function setStorageLocation(?string $storageLocation): self { $this->storageLocation = $storageLocation; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
}
