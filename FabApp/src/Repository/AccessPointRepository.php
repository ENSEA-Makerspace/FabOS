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
