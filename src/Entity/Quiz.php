<?php

namespace App\Entity;

use App\Repository\QuizRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: QuizRepository::class)]
#[ORM\Table(name: 'QUIZ')]
class Quiz
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Formation::class)]
    #[ORM\JoinColumn(name: 'formationId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Formation $formation = null;

    #[ORM\ManyToOne(targetEntity: Section::class)]
    #[ORM\JoinColumn(name: 'sectionId', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?Section $section = null;

    #[ORM\Column(name: 'noteMinimale', options: ['default' => 0])]
    private int $noteMinimale = 0;

    public function getId(): ?int { return $this->id; }
    public function getFormation(): ?Formation { return $this->formation; }
    public function setFormation(?Formation $formation): self { $this->formation = $formation; return $this; }
    public function getSection(): ?Section { return $this->section; }
    public function setSection(?Section $section): self { $this->section = $section; return $this; }
    public function getNoteMinimale(): int { return $this->noteMinimale; }
    public function setNoteMinimale(int $noteMinimale): self { $this->noteMinimale = $noteMinimale; return $this; }
}
