<?php

namespace App\Entity;

use App\Repository\QuestionRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: QuestionRepository::class)]
#[ORM\Table(name: 'QUESTION')]
class Question
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Quiz::class)]
    #[ORM\JoinColumn(name: 'quizId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Quiz $quiz = null;

    #[ORM\Column(type: 'text')]
    private string $texte = '';

    #[ORM\Column(length: 50)]
    private string $type = '';

    #[ORM\Column]
    private int $ordre = 0;

    public function getId(): ?int { return $this->id; }
    public function getQuiz(): ?Quiz { return $this->quiz; }
    public function setQuiz(?Quiz $quiz): self { $this->quiz = $quiz; return $this; }
    public function getTexte(): string { return $this->texte; }
    public function setTexte(string $texte): self { $this->texte = $texte; return $this; }
    public function getType(): string { return $this->type; }
    public function setType(string $type): self { $this->type = $type; return $this; }
    public function getOrdre(): int { return $this->ordre; }
    public function setOrdre(int $ordre): self { $this->ordre = $ordre; return $this; }
}
