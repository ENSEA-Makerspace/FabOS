<?php

namespace App\Repository;

use App\Entity\Place;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Place>
 */
class PlaceRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Place::class);
    }

    /**
     * The spaces a member may be offered — archived ones excluded.
     *
     * ⚠️ **Écrit à l'identique de `MachineRepository::findLive()`**, parce que les
     * deux répondent à la même question et que deux formulations divergent. Le
     * calendrier propose des créneaux sur ces lignes : un espace archivé n'y a rien
     * à faire, alors que `findBy()` reste la question de l'admin — c'est de sa liste
     * qu'on le restaure.
     *
     * @param array<string, mixed> $criteria
     * @param array<string, string> $orderBy
     * @return Place[]
     */
    public function findLive(array $criteria = [], array $orderBy = ['nom' => 'ASC'], ?int $limit = null): array
    {
        $qb = $this->createQueryBuilder('p')->andWhere('p.archivedAt IS NULL');

        foreach ($criteria as $field => $value) {
            $qb->andWhere(sprintf('p.%s = :%s', $field, $field))->setParameter($field, $value);
        }

        foreach ($orderBy as $field => $direction) {
            $qb->addOrderBy('p.' . $field, strtoupper($direction) === 'DESC' ? 'DESC' : 'ASC');
        }

        if ($limit !== null) {
            $qb->setMaxResults($limit);
        }

        return $qb->getQuery()->getResult();
    }
}
