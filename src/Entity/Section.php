<?php

namespace App\Entity;

use App\Repository\SectionRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: SectionRepository::class)]
#[ORM\Table(name: 'SECTION')]
class Section
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Formation::class)]
    #[ORM\JoinColumn(name: 'formationId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Formation $formation = null;

    #[ORM\Column(length: 255)]
    private string $titre = '';

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $contenu = null;

    #[ORM\Column(name: 'videoUrl', length: 255, nullable: true)]
    private ?string $videoUrl = null;

    #[ORM\Column]
    private int $ordre = 0;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct() { $this->createdAt = new \DateTimeImmutable(); }
    public function getId(): ?int { return $this->id; }
    public function getFormation(): ?Formation { return $this->formation; }
    public function setFormation(?Formation $formation): self { $this->formation = $formation; return $this; }
    public function getTitre(): string { return $this->titre; }
    public function setTitre(string $titre): self { $this->titre = $titre; return $this; }
    public function getContenu(): ?string { return $this->contenu; }
    public function setContenu(?string $contenu): self { $this->contenu = $contenu; return $this; }
    public function getVideoUrl(): ?string { return $this->videoUrl; }
    public function setVideoUrl(?string $videoUrl): self { $this->videoUrl = $videoUrl; return $this; }
    public function getOrdre(): int { return $this->ordre; }
    public function setOrdre(int $ordre): self { $this->ordre = $ordre; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
}
