<?php

namespace App\Entity;

use App\Repository\RoleRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: RoleRepository::class)]
#[ORM\Table(name: 'ROLE')]
class Role
{
    /** Stable built-ins; labels remain locally presentable without changing keys. */
    public const BUILTIN_GROUPS = [
        'admin' => ['Administrateur global', 'Récupération système hors packages.'],
        'manager' => ['Manager', 'Responsable métier.'],
        'staff' => ['Staff', 'Équipe opérationnelle.'],
        'super_user' => ['Super user', 'Utilisateur avancé.'],
        'user' => ['User', 'Audience de tout compte actif.'],
        'guest' => ['Guest', 'Audience anonyme virtuelle.'],
        'trainers' => ['Formateurs', 'Formateurs du FabOS.'],
    ];
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 50, unique: true)]
    private string $nom = '';

    #[ORM\Column(name: 'groupKey', length: 50, unique: true, nullable: true)]
    private ?string $groupKey = null;

    #[ORM\Column(length: 100, nullable: true)]
    private ?string $label = null;

    #[ORM\Column(length: 255, nullable: true)]
    private ?string $description = null;

    #[ORM\Column(name: 'isProtected', options: ['default' => false])]
    private bool $isProtected = false;

    public function getId(): ?int { return $this->id; }
    public function getNom(): string { return $this->nom; }
    public function setNom(string $nom): self { $this->nom = $nom; return $this; }
    public function getName(): string { return $this->nom; }
    public function setName(string $name): self { return $this->setNom($name); }
    public function getGroupKey(): ?string { return $this->groupKey; }
    public function setGroupKey(?string $groupKey): self { $this->groupKey = $groupKey; return $this; }
    public function getLabel(): string { return $this->label ?: $this->nom; }
    public function setLabel(?string $label): self { $this->label = $label; return $this; }
    public function getDescription(): ?string { return $this->description; }
    public function setDescription(?string $description): self { $this->description = $description; return $this; }
    public function isProtected(): bool { return $this->isProtected; }
    public function setIsProtected(bool $isProtected): self { $this->isProtected = $isProtected; return $this; }
}
