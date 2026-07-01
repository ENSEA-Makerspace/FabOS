<?php

namespace App\Entity;

use App\Repository\ProgressionRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: ProgressionRepository::class)]
#[ORM\Table(name: 'PROGRESSION')]
#[ORM\UniqueConstraint(name: 'unique_user_formation', columns: ['userId', 'formationId'])]
class Progression
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Utilisateur $utilisateur = null;

    #[ORM\ManyToOne(targetEntity: Formation::class)]
    #[ORM\JoinColumn(name: 'formationId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Formation $formation = null;

    #[ORM\Column(options: ['default' => 0])]
    private int $score = 0;

    #[ORM\Column(options: ['default' => false])]
    private bool $completed = false;

    #[ORM\Column(name: 'dateDebut', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $dateDebut;

    #[ORM\Column(name: 'dateEnd', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $dateEnd = null;

    public function __construct() { $this->dateDebut = new \DateTimeImmutable(); }
    public function getId(): ?int { return $this->id; }
    public function getUtilisateur(): ?Utilisateur { return $this->utilisateur; }
    public function setUtilisateur(?Utilisateur $utilisateur): self { $this->utilisateur = $utilisateur; return $this; }
    public function getUser(): ?Utilisateur { return $this->utilisateur; }
    public function setUser(?Utilisateur $user): self { return $this->setUtilisateur($user); }
    public function getFormation(): ?Formation { return $this->formation; }
    public function setFormation(?Formation $formation): self { $this->formation = $formation; return $this; }
    public function getScore(): int { return $this->score; }
    public function setScore(int $score): self { $this->score = $score; return $this; }
    public function isCompleted(): bool { return $this->completed; }
    public function setCompleted(bool $completed): self { $this->completed = $completed; return $this; }
    public function getDateDebut(): \DateTimeImmutable { return $this->dateDebut; }
    public function setDateDebut(\DateTimeImmutable $dateDebut): self { $this->dateDebut = $dateDebut; return $this; }
    public function getDateEnd(): ?\DateTimeImmutable { return $this->dateEnd; }
    public function setDateEnd(?\DateTimeImmutable $dateEnd): self { $this->dateEnd = $dateEnd; return $this; }
}
