<?php

namespace App\Entity;

use App\Repository\MachineUtilisateurRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: MachineUtilisateurRepository::class)]
#[ORM\Table(name: 'MACHINE_UTILISATEUR')]
class MachineUtilisateur
{
    #[ORM\Id]
    #[ORM\ManyToOne(targetEntity: Machine::class)]
    #[ORM\JoinColumn(name: 'machineId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Machine $machine = null;

    #[ORM\Id]
    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Utilisateur $utilisateur = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct() { $this->createdAt = new \DateTimeImmutable(); }
    public function getMachine(): ?Machine { return $this->machine; }
    public function setMachine(?Machine $machine): self { $this->machine = $machine; return $this; }
    public function getUtilisateur(): ?Utilisateur { return $this->utilisateur; }
    public function setUtilisateur(?Utilisateur $utilisateur): self { $this->utilisateur = $utilisateur; return $this; }
    public function getUser(): ?Utilisateur { return $this->utilisateur; }
    public function setUser(?Utilisateur $user): self { return $this->setUtilisateur($user); }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }
}
