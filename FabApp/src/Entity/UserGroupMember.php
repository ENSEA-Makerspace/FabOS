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
    // ⚠️ Pas de `nullable: false` sur ces deux colonnes de jointure : sur une
    // to-one qui fait partie de l'IDENTIFIANT, l'ORM le force déjà à `false` et
    // l'écrire est déprécié (erreur en 4.0). Vu dans la sortie de `cache:clear`.
    #[ORM\Id]
    #[ORM\ManyToOne(targetEntity: Utilisateur::class, inversedBy: 'groupMemberships')]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', onDelete: 'CASCADE')]
    private ?Utilisateur $utilisateur = null;

    #[ORM\Id]
    #[ORM\ManyToOne(targetEntity: UserGroup::class)]
    #[ORM\JoinColumn(name: 'groupId', referencedColumnName: 'id', onDelete: 'CASCADE')]
    private ?UserGroup $group = null;

    #[ORM\Column(name: 'addedAt', type: 'datetime_immutable')]
    private \DateTimeImmutable $addedAt;

    /**
     * 🔴 **Mappées parce que `getRoles()` doit pouvoir EXPIRER un rôle.**
     * `AudienceResolver` filtrait déjà la fenêtre en SQL ; l'entité, elle, ne
     * chargeait pas ces colonnes, donc `Utilisateur::getRoles()` parcourait les
     * appartenances SANS filtre. Une appartenance `staff` expirée continuait
     * d'accorder `ROLE_STAFF` — et avec lui `isStaff()`, le palier de
     * réservation et toute route gardée par ce rôle. Deux lecteurs d'une même
     * table dont un seul filtre : le motif de toute la phase, cette fois sur le
     * chemin de la SÉCURITÉ. Prouvé par la sonde avant d'être corrigé.
     *
     * ⚠️ **Un mapping ORM ne peut pas être conditionnel**, contrairement à
     * `UserGroupSchema` qui sonde. Ce fichier exige donc que
     * `Version20260902160000` soit jouée — elle l'est. C'est la règle de
     * séquencement habituelle, rappelée ici parce que `getRoles()` est sur le
     * chemin de la connexion : déployer ce code avant sa migration ferait tomber
     * chaque requête, pas un écran.
     */
    #[ORM\Column(name: 'validFrom', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $validFrom = null;

    #[ORM\Column(name: 'validUntil', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $validUntil = null;

    public function __construct()
    {
        $this->addedAt = new \DateTimeImmutable();
    }

    public function getUtilisateur(): ?Utilisateur { return $this->utilisateur; }
    public function getGroup(): ?UserGroup { return $this->group; }
    public function getAddedAt(): \DateTimeImmutable { return $this->addedAt; }
    public function getValidFrom(): ?\DateTimeImmutable { return $this->validFrom; }
    public function getValidUntil(): ?\DateTimeImmutable { return $this->validUntil; }

    /**
     * L'appartenance vaut-elle à cet instant ?
     *
     * 🔴 **`NULL` veut dire SANS LIMITE, des deux côtés** — la même règle que
     * `UserGroupSchema::activeClause()`, et le même piège : traiter
     * `validFrom IS NULL` comme « pas encore commencée » retirerait `staff`,
     * `admin` et `trainers` à tout le monde d'un coup.
     *
     * ⚠️ **Bornes identiques au SQL** : début INCLUS (`<=`), fin EXCLUE (`>`).
     * Deux conventions pour une même donnée feraient diverger l'écran et le
     * rôle sur la dernière seconde, ce qui est exactement le genre d'écart que
     * personne ne reproduit.
     *
     * 🔴 **L'instant qu'on passe ici décide de la justesse.** Les bornes stockées
     * sont l'heure MURALE du labo ; un `now` réel les décale de l'offset, dans le
     * sens PERMISSIF — voir la note de `Utilisateur::getRoles()`, qui est le seul
     * appelant à ne pas pouvoir fournir mieux.
     */
    public function isActiveAt(\DateTimeImmutable $at): bool
    {
        if ($this->validFrom !== null && $this->validFrom > $at) {
            return false;
        }

        return $this->validUntil === null || $this->validUntil > $at;
    }
}
