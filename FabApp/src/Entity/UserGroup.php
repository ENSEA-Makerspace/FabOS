<?php

namespace App\Entity;

use Doctrine\ORM\Mapping as ORM;

/**
 * Un GROUPE, vu par Doctrine (S159b).
 *
 * 🔴 **Pourquoi cette entité existe alors que `UserGroupRepository` fait déjà
 * tout en DBAL.** `Utilisateur::getRoles()` est sur le chemin de la SÉCURITÉ :
 * Symfony l'appelle à chaque requête, et il doit pouvoir répondre depuis le
 * compte seul, sans service injecté. Tant que les groupes n'existaient qu'en
 * DBAL, un rôle ne pouvait pas venir d'un groupe.
 *
 * ⚠️ **Elle est en LECTURE.** L'écriture reste dans `UserGroupRepository`, qui
 * porte les gardes — clé immuable, intégré non supprimable, dernier
 * administrateur. Deux surfaces d'écriture pour une table, c'est la faute que
 * toute cette phase range ; l'entité n'en ouvre pas une seconde.
 *
 * ⚠️ **Aucune migration** : le mapping décrit la table telle que
 * `Version20260816130000` l'a créée. Rien n'est ajouté, rien n'est renommé.
 */
#[ORM\Entity]
#[ORM\Table(name: 'USER_GROUP')]
class UserGroup
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;

    #[ORM\Column(name: 'groupKey', length: 60)]
    private string $groupKey = '';

    #[ORM\Column(length: 120)]
    private string $label = '';

    #[ORM\Column(length: 255, nullable: true)]
    private ?string $description = null;

    #[ORM\Column(type: 'boolean')]
    private bool $builtin = false;

    #[ORM\Column(type: 'boolean')]
    private bool $virtual = false;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getGroupKey(): string { return $this->groupKey; }
    public function getLabel(): string { return $this->label; }
    public function getDescription(): ?string { return $this->description; }
    public function isBuiltin(): bool { return $this->builtin; }
    public function isVirtual(): bool { return $this->virtual; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
}
