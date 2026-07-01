<?php

namespace App\Entity;

use App\Repository\UtilisateurRoleRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: UtilisateurRoleRepository::class)]
#[ORM\Table(name: 'UTILISATEUR_ROLE')]
class UtilisateurRole
{
    #[ORM\Id]
    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'utilisateurId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Utilisateur $utilisateur = null;

    #[ORM\Id]
    #[ORM\ManyToOne(targetEntity: Role::class)]
    #[ORM\JoinColumn(name: 'roleId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Role $role = null;

    public function getUtilisateur(): ?Utilisateur { return $this->utilisateur; }
    public function setUtilisateur(?Utilisateur $utilisateur): self { $this->utilisateur = $utilisateur; return $this; }
    public function getRole(): ?Role { return $this->role; }
    public function setRole(?Role $role): self { $this->role = $role; return $this; }
}
