<?php

namespace App\Repository;

use App\Entity\Formation;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Formation>
 */
class FormationRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Formation::class);
    }

    public function findOneByNormalizedTitle(string $title): ?Formation
    {
        $normalizedTitle = mb_strtolower(trim($title));
        if ($normalizedTitle === '') {
            return null;
        }

        return $this->createQueryBuilder('formation')
            ->andWhere('LOWER(formation.titre) = :title')
            ->setParameter('title', $normalizedTitle)
            ->setMaxResults(1)
            ->getQuery()
            ->getOneOrNullResult();
    }
}
