<?php

namespace App\Entity;

use Doctrine\ORM\Mapping as ORM;

/**
 * L'appartenance d'une personne à un groupe, vue par Doctrine (S159b).
 *
 * ⚠️ **Même forme que `UtilisateurRole`** : deux `ManyToOne` marqués `#[ORM\Id]`,
 * parce que la table a une clé primaire composite `(groupId, userId)`. C'est la
 * table telle qu'elle existe — aucune migration.
 *
 * ⚠️ **Lecture seule côté ORM.** `addedAt` est `NOT NULL` sans valeur par défaut :
 * une écriture par l'ORM échouerait si personne ne la renseigne. C'est voulu —
 * l'écriture passe par `UserGroupRepository`, qui porte les gardes.
 */
#[ORM\Entity]
#[ORM\Table(name: 'USER_GROUP_MEMBER')]
class UserGroupMember
{
    #[ORM\Id]
    #[ORM\ManyToOne(targetEntity: Utilisateur::class, inversedBy: 'groupMemberships')]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Utilisateur $utilisateur = null;

    #[ORM\Id]
    #[ORM\ManyToOne(targetEntity: UserGroup::class)]
    #[ORM\JoinColumn(name: 'groupId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?UserGroup $group = null;

    #[ORM\Column(name: 'addedAt', type: 'datetime_immutable')]
    private \DateTimeImmutable $addedAt;

    public function __construct()
    {
        $this->addedAt = new \DateTimeImmutable();
    }

    public function getUtilisateur(): ?Utilisateur { return $this->utilisateur; }
    public function getGroup(): ?UserGroup { return $this->group; }
    public function getAddedAt(): \DateTimeImmutable { return $this->addedAt; }
}
