<?php

namespace App\Entity;

use App\Repository\CreationRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: CreationRepository::class)]
#[ORM\Table(name: 'CREATION')]
class Creation
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 150)]
    private string $title = '';

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $description = null;

    #[ORM\Column(name: 'imageFilename', length: 255, nullable: true)]
    private ?string $imageFilename = null;

    #[ORM\Column(name: 'fileFilename', length: 255, nullable: true)]
    private ?string $fileFilename = null;

    #[ORM\Column(name: 'externalUrl', length: 500, nullable: true)]
    private ?string $externalUrl = null;

    #[ORM\Column(name: 'printDurationMinutes', nullable: true)]
    private ?int $printDurationMinutes = null;

    /** Comma-separated list of tags (normalized on save). */
    #[ORM\Column(name: 'tags', length: 255, nullable: true)]
    private ?string $tags = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'authorId', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?Utilisateur $author = null;

    #[ORM\Column(name: 'authorName', length: 150, nullable: true)]
    private ?string $authorName = null;

    #[ORM\Column(name: 'isPublished', options: ['default' => true])]
    private bool $isPublished = true;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    #[ORM\Column(name: 'updatedAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $updatedAt = null;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getTitle(): string { return $this->title; }
    public function setTitle(string $title): self { $this->title = $title; return $this; }
    public function getDescription(): ?string { return $this->description; }
    public function setDescription(?string $description): self { $this->description = $description; return $this; }
    public function getImageFilename(): ?string { return $this->imageFilename; }
    public function setImageFilename(?string $imageFilename): self { $this->imageFilename = $imageFilename; return $this; }
    public function getFileFilename(): ?string { return $this->fileFilename; }
    public function setFileFilename(?string $fileFilename): self { $this->fileFilename = $fileFilename; return $this; }
    public function getExternalUrl(): ?string { return $this->externalUrl; }
    public function setExternalUrl(?string $externalUrl): self { $this->externalUrl = $externalUrl; return $this; }
    public function getPrintDurationMinutes(): ?int { return $this->printDurationMinutes; }
    public function setPrintDurationMinutes(?int $printDurationMinutes): self { $this->printDurationMinutes = $printDurationMinutes; return $this; }
    public function getFormattedPrintDuration(): ?string
    {
        if ($this->printDurationMinutes === null) {
            return null;
        }

        $hours = intdiv($this->printDurationMinutes, 60);
        $minutes = $this->printDurationMinutes % 60;

        if ($hours === 0) {
            return sprintf('%d min', $minutes);
        }

        if ($minutes === 0) {
            return sprintf('%d h', $hours);
        }

        return sprintf('%d h %02d', $hours, $minutes);
    }
    public function getTags(): ?string { return $this->tags; }
    public function setTags(?string $tags): self { $this->tags = $tags; return $this; }
    /** @return string[] */
    public function getTagList(): array
    {
        if ($this->tags === null || trim($this->tags) === '') {
            return [];
        }

        return array_values(array_filter(array_map('trim', explode(',', $this->tags))));
    }
    public function getAuthor(): ?Utilisateur { return $this->author; }
    public function setAuthor(?Utilisateur $author): self { $this->author = $author; return $this; }
    public function getAuthorName(): ?string { return $this->authorName; }
    public function setAuthorName(?string $authorName): self { $this->authorName = $authorName; return $this; }
    public function isPublished(): bool { return $this->isPublished; }
    public function setIsPublished(bool $isPublished): self { $this->isPublished = $isPublished; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }
    public function getUpdatedAt(): ?\DateTimeImmutable { return $this->updatedAt; }
    public function setUpdatedAt(?\DateTimeImmutable $updatedAt): self { $this->updatedAt = $updatedAt; return $this; }

    public function getDisplayAuthor(): string
    {
        if ($this->author !== null) {
            return $this->author->getDisplayName();
        }

        $authorName = trim((string) $this->authorName);

        return $authorName !== '' ? $authorName : 'Créateur anonyme';
    }
}
