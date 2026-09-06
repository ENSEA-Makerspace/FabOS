<?php

namespace App\Repository;

use App\Entity\AccessPoint;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<AccessPoint>
 */
final class AccessPointRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, AccessPoint::class);
    }

    /**
     * ⚠️ **`findLive()` est la question de qui PROPOSE ; `findForAdmin()` celle de
     * qui gère.** Même partage que `MachineRepository` et `MaterialRepository` :
     * un point archivé sort des listes qui offrent un choix, et reste visible
     * là où on le restaure. Écrire ce filtre au cas par cas dans les
     * contrôleurs est la façon dont il finit par manquer à un endroit.
     *
     * @return AccessPoint[]
     */
    public function findLive(): array
    {
        return $this->createQueryBuilder('ap')
            ->andWhere('ap.archivedAt IS NULL')
            ->orderBy('ap.nom', 'ASC')
            ->getQuery()
            ->getResult();
    }

    /**
     * Les points d'accès qui ouvrent CET espace (S177).
     *
     * 🔴 **C'est la question « comment j'entre ? »**, et jusqu'à S175 le produit
     * ne pouvait pas y répondre : une porte n'existait pas comme objet. La fiche
     * d'un espace disait où il est et quand il est libre, jamais par où on y
     * entre ni avec quoi.
     *
     * ⚠️ Archivés exclus — c'est une surface qui PROPOSE. Une porte retirée du
     * service ne doit pas être annoncée comme la façon d'entrer.
     *
     * @return AccessPoint[]
     */
    public function findForPlace(int $placeId): array
    {
        return $this->createQueryBuilder('ap')
            ->andWhere('ap.archivedAt IS NULL')
            ->andWhere('IDENTITY(ap.place) = :placeId')
            ->setParameter('placeId', $placeId)
            ->orderBy('ap.nom', 'ASC')
            ->getQuery()
            ->getResult();
    }

    /**
     * Les identifiants des points qui n'ont AUCUN boîtier — donc que personne ne
     * peut ouvrir (S178).
     *
     * 🔴 **C'est une affordance morte au niveau physique.** Une porte déclarée,
     * nommée, rattachée à un espace, annoncée sur la fiche publique comme « voici
     * par où on entre » — et aucun lecteur au mur. Le membre arrive devant, badge,
     * et rien ne se passe. Rien dans le produit ne le disait : la ligne
     * s'affichait « Actif », en vert.
     *
     * ⚠️ Les lecteurs ARCHIVÉS ne comptent pas : un boîtier retiré du service
     * n'ouvre rien, et l'inclure ferait passer une porte muette pour équipée.
     *
     * @return list<int>
     */
    public function idsWithoutReader(): array
    {
        $rows = $this->getEntityManager()->createQuery(
            'SELECT ap.id FROM App\\Entity\\AccessPoint ap
             WHERE ap.archivedAt IS NULL
               AND NOT EXISTS (
                   SELECT r.id FROM App\\Entity\\RfidReader r
                   WHERE IDENTITY(r.accessPoint) = ap.id AND r.archivedAt IS NULL
               )'
        )->getScalarResult();

        return array_map(static fn (array $row): int => (int) $row['id'], $rows);
    }

    /** @return AccessPoint[] */
    public function findForAdmin(): array
    {
        return $this->createQueryBuilder('ap')
            ->orderBy('ap.archivedAt', 'ASC')
            ->addOrderBy('ap.nom', 'ASC')
            ->getQuery()
            ->getResult();
    }
}
