<?php

namespace App\Entity;

use App\Repository\CreationVoteRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: CreationVoteRepository::class)]
#[ORM\Table(name: 'CREATION_VOTE')]
#[ORM\UniqueConstraint(name: 'uniq_creation_vote_user', columns: ['creationId', 'userId'])]
class CreationVote
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Creation::class)]
    #[ORM\JoinColumn(name: 'creationId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Creation $creation = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Utilisateur $user = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getCreation(): ?Creation { return $this->creation; }
    public function setCreation(?Creation $creation): self { $this->creation = $creation; return $this; }
    public function getUser(): ?Utilisateur { return $this->user; }
    public function setUser(?Utilisateur $user): self { $this->user = $user; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }
}
