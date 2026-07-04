<?php

namespace App\Repository;

use App\Entity\Badge;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Badge>
 */
class BadgeRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Badge::class);
    }

    public function findOneByNormalizedName(string $name): ?Badge
    {
        $normalizedName = mb_strtolower(trim($name));
        if ($normalizedName === '') {
            return null;
        }

        return $this->createQueryBuilder('badge')
            ->andWhere('LOWER(badge.nom) = :name')
            ->setParameter('name', $normalizedName)
            ->setMaxResults(1)
            ->getQuery()
            ->getOneOrNullResult();
    }
}
