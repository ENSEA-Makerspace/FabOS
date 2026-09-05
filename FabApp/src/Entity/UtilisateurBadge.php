<?php

namespace App\Entity;

use App\Repository\UtilisateurBadgeRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: UtilisateurBadgeRepository::class)]
#[ORM\Table(name: 'UTILISATEUR_BADGE')]
class UtilisateurBadge
{
    // ⚠️ **Pas de `nullable:` sur une colonne de jointure qui fait partie de
    // l'identifiant** : Doctrine l'ignore et la force à `false` de toute façon,
    // et le dire est déprécié — erreur en Doctrine 4. La dépréciation sortait à
    // chaque `cache:clear`. Même correction qu'en S159 sur `UserGroupMember`.
    // ⚠️ Aucun effet en base : la contrainte NOT NULL était déjà celle-là.
    #[ORM\Id]
    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'utilisateurId', referencedColumnName: 'id', onDelete: 'CASCADE')]
    private ?Utilisateur $utilisateur = null;

    #[ORM\Id]
    #[ORM\ManyToOne(targetEntity: Badge::class)]
    #[ORM\JoinColumn(name: 'badgeId', referencedColumnName: 'id', onDelete: 'RESTRICT')]
    private ?Badge $badge = null;

    #[ORM\Column(name: 'dateObtention', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $dateObtention;

    public function __construct() { $this->dateObtention = new \DateTimeImmutable(); }
    public function getUtilisateur(): ?Utilisateur { return $this->utilisateur; }
    public function setUtilisateur(?Utilisateur $utilisateur): self { $this->utilisateur = $utilisateur; return $this; }
    public function getBadge(): ?Badge { return $this->badge; }
    public function setBadge(?Badge $badge): self { $this->badge = $badge; return $this; }
    public function getDateObtention(): \DateTimeImmutable { return $this->dateObtention; }
    public function setDateObtention(\DateTimeImmutable $dateObtention): self { $this->dateObtention = $dateObtention; return $this; }
}
