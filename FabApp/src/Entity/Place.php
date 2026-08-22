<?php

namespace App\Entity;

use App\Repository\PlaceRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: PlaceRepository::class)]
#[ORM\Table(name: 'PLACE')]
class Place
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 150)]
    private string $nom = '';

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $description = null;

    #[ORM\Column(length: 150, nullable: true)]
    private ?string $localisation = null;

    #[ORM\Column(nullable: true)]
    private ?int $capacite = null;

    #[ORM\Column(length: 120, nullable: true)]
    private ?string $category = null;

    #[ORM\Column(length: 150, nullable: true)]
    private ?string $manager = null;

    #[ORM\Column(length: 150, nullable: true)]
    private ?string $department = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    #[ORM\ManyToOne(targetEntity: Venue::class)]
    #[ORM\JoinColumn(name: 'venueId', nullable: false, onDelete: 'RESTRICT')]
    private ?Venue $venue = null;

    /**
     * ⚠️ **S147, J-2 — archivé, pas supprimé.** L'action d'administration
     * appelait `->remove()` : la ligne disparaissait, et avec elle tout ce qui
     * la nommait. Un espace archivé garde ses réservations passées et sort des catalogues. 🔴 Et l'archiver ANNULE ses réservations à venir : c'est un espace réservable, et la règle de S134f est explicite — on ne laisse pas un membre avec une réservation confirmée sur un lieu que le labo ne propose plus.
     *
     * ⚠️ `findBy()` reste la question de l'ADMIN : la liste d'administration
     * montre les archivés, marqués comme tels, parce que c'est de là qu'on les
     * restaure. Ce sont les surfaces qui PROPOSENT qui doivent filtrer.
     */
    #[ORM\Column(name: 'archivedAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $archivedAt = null;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getNom(): string { return $this->nom; }
    public function setNom(string $nom): self { $this->nom = $nom; return $this; }
    public function getDescription(): ?string { return $this->description; }
    public function setDescription(?string $description): self { $this->description = $description; return $this; }
    public function getLocalisation(): ?string { return $this->localisation; }
    public function setLocalisation(?string $localisation): self { $this->localisation = $localisation; return $this; }
    public function getCapacite(): ?int { return $this->capacite; }
    public function setCapacite(?int $capacite): self { $this->capacite = $capacite; return $this; }
    public function getCategory(): ?string { return $this->category; }
    public function setCategory(?string $category): self { $this->category = $category; return $this; }
    public function getManager(): ?string { return $this->manager; }
    public function setManager(?string $manager): self { $this->manager = $manager; return $this; }
    public function getDepartment(): ?string { return $this->department; }
    public function setDepartment(?string $department): self { $this->department = $department; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function getVenue(): ?Venue { return $this->venue; }
    public function setVenue(Venue $venue): self { $this->venue = $venue; return $this; }

    public function getArchivedAt(): ?\DateTimeImmutable { return $this->archivedAt; }
    public function isArchived(): bool { return $this->archivedAt !== null; }
    public function archive(): self { $this->archivedAt ??= new \DateTimeImmutable(); return $this; }
    public function restore(): self { $this->archivedAt = null; return $this; }
}
