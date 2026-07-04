<?php

namespace App\Entity;

use App\Repository\FormationRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: FormationRepository::class)]
#[ORM\Table(name: 'FORMATION')]
class Formation
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Badge::class)]
    #[ORM\JoinColumn(name: 'badgeId', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?Badge $badge = null;

    #[ORM\Column(length: 255)]
    private string $titre = '';

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $description = null;

    #[ORM\Column(length: 255, nullable: true)]
    private ?string $image = null;

    #[ORM\Column(length: 100, nullable: true)]
    private ?string $categorie = null;

    #[ORM\Column(nullable: true)]
    private ?int $niveau = null;

    #[ORM\Column(length: 100, nullable: true)]
    private ?string $duree = null;

    #[ORM\Column(length: 150, nullable: true)]
    private ?string $formateur = null;

    #[ORM\Column(name: 'placesTotales', nullable: true)]
    private ?int $placesTotales = null;

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $objectifs = null;

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $prerequis = null;

    #[ORM\Column(name: 'materielFourni', type: 'text', nullable: true)]
    private ?string $materielFourni = null;


    public function getId(): ?int { return $this->id; }
    public function getBadge(): ?Badge { return $this->badge; }
    public function setBadge(?Badge $badge): self { $this->badge = $badge; return $this; }
    public function getTitre(): string { return $this->titre; }
    public function setTitre(string $titre): self { $this->titre = $titre; return $this; }
    public function getTitle(): string { return $this->titre; }
    public function setTitle(string $title): self { return $this->setTitre($title); }
    public function getDescription(): ?string { return $this->description; }
    public function setDescription(?string $description): self { $this->description = $description; return $this; }
    public function getImage(): ?string { return $this->image; }
    public function setImage(?string $image): self { $this->image = $image; return $this; }
    public function getCategorie(): ?string { return $this->categorie; }
    public function setCategorie(?string $categorie): self { $this->categorie = $categorie; return $this; }
    public function getNiveau(): ?int { return $this->niveau; }
    public function setNiveau(?int $niveau): self { $this->niveau = $niveau; return $this; }
    public function getDuree(): ?string { return $this->duree; }
    public function setDuree(?string $duree): self { $this->duree = $duree; return $this; }
    public function getFormateur(): ?string { return $this->formateur; }
    public function setFormateur(?string $formateur): self { $this->formateur = $formateur; return $this; }
    public function getPlacesTotales(): ?int { return $this->placesTotales; }
    public function setPlacesTotales(?int $placesTotales): self { $this->placesTotales = $placesTotales; return $this; }
    public function getObjectifs(): ?string { return $this->objectifs; }
    public function setObjectifs(?string $objectifs): self { $this->objectifs = $objectifs; return $this; }
    public function getPrerequis(): ?string { return $this->prerequis; }
    public function setPrerequis(?string $prerequis): self { $this->prerequis = $prerequis; return $this; }
    public function getMaterielFourni(): ?string { return $this->materielFourni; }
    public function setMaterielFourni(?string $materielFourni): self { $this->materielFourni = $materielFourni; return $this; }

}
