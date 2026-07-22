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

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

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
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
}
