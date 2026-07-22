<?php

namespace App\Entity;

use App\Repository\LabPageRepository;
use Doctrine\Common\Collections\ArrayCollection;
use Doctrine\Common\Collections\Collection;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: LabPageRepository::class)]
#[ORM\Table(name: 'LAB_PAGE')]
class LabPage
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 255)]
    private string $titre = '';

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $contenu = null;

    #[ORM\ManyToOne(targetEntity: self::class, inversedBy: 'children')]
    #[ORM\JoinColumn(name: 'parentPageId', referencedColumnName: 'id', nullable: true, onDelete: 'CASCADE')]
    private ?self $parentPage = null;

    #[ORM\Column]
    private int $position = 0;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    #[ORM\Column(name: 'updatedAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $updatedAt = null;

    /** @var Collection<int, LabPage> */
    #[ORM\OneToMany(mappedBy: 'parentPage', targetEntity: self::class)]
    #[ORM\OrderBy(['position' => 'ASC', 'titre' => 'ASC'])]
    private Collection $children;

    /** @var Collection<int, LabPageImage> */
    #[ORM\OneToMany(mappedBy: 'labPage', targetEntity: LabPageImage::class)]
    #[ORM\OrderBy(['createdAt' => 'ASC'])]
    private Collection $images;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
        $this->children = new ArrayCollection();
        $this->images = new ArrayCollection();
    }

    public function getId(): ?int { return $this->id; }
    public function getTitre(): string { return $this->titre; }
    public function setTitre(string $titre): self { $this->titre = $titre; return $this; }
    public function getContenu(): ?string { return $this->contenu; }
    public function setContenu(?string $contenu): self { $this->contenu = $contenu; return $this; }
    public function getParentPage(): ?self { return $this->parentPage; }
    public function setParentPage(?self $parentPage): self { $this->parentPage = $parentPage; return $this; }
    public function getPosition(): int { return $this->position; }
    public function setPosition(int $position): self { $this->position = $position; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function getUpdatedAt(): ?\DateTimeImmutable { return $this->updatedAt; }
    public function setUpdatedAt(?\DateTimeImmutable $updatedAt): self { $this->updatedAt = $updatedAt; return $this; }
    /** @return Collection<int, LabPage> */
    public function getChildren(): Collection { return $this->children; }
    /** @return Collection<int, LabPageImage> */
    public function getImages(): Collection { return $this->images; }

    public function isTopLevel(): bool
    {
        return $this->parentPage === null;
    }
}
