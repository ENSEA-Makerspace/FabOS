<?php

namespace App\Entity;

use App\Repository\ChoixRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: ChoixRepository::class)]
#[ORM\Table(name: 'CHOIX')]
class Choix
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Question::class)]
    #[ORM\JoinColumn(name: 'questionId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Question $question = null;

    #[ORM\Column(type: 'text')]
    private string $texte = '';

    #[ORM\Column(name: 'estCorrect')]
    private bool $estCorrect = false;

    #[ORM\Column]
    private int $ordre = 0;

    public function getId(): ?int { return $this->id; }
    public function getQuestion(): ?Question { return $this->question; }
    public function setQuestion(?Question $question): self { $this->question = $question; return $this; }
    public function getTexte(): string { return $this->texte; }
    public function setTexte(string $texte): self { $this->texte = $texte; return $this; }
    public function isEstCorrect(): bool { return $this->estCorrect; }
    public function setEstCorrect(bool $estCorrect): self { $this->estCorrect = $estCorrect; return $this; }
    public function getOrdre(): int { return $this->ordre; }
    public function setOrdre(int $ordre): self { $this->ordre = $ordre; return $this; }
}
