<?php

namespace App\Entity;

use App\Repository\VenueRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: VenueRepository::class)]
#[ORM\Table(name: 'VENUE')]
class Venue
{
    #[ORM\Id, ORM\GeneratedValue, ORM\Column]
    private ?int $id = null;
    #[ORM\Column(length: 80, unique: true)]
    private string $slug = 'default';
    #[ORM\Column(length: 140)]
    private string $name = '';
    #[ORM\Column(length: 255, nullable: true)]
    private ?string $address = null;
    #[ORM\Column(length: 64)]
    private string $timezone = 'Europe/Paris';
    #[ORM\Column(options: ['default' => true])]
    private bool $active = true;
    public function getId(): ?int { return $this->id; }
    public function getSlug(): string { return $this->slug; }
    public function getName(): string { return $this->name; }
    public function getTimezone(): string { return $this->timezone; }
    public function isActive(): bool { return $this->active; }
}
