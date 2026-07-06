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

    /** @return Badge[] */
    public function findForAdminFilters(array $filters): array
    {
        $qb = $this->createQueryBuilder('badge')
            ->orderBy('badge.createdAt', 'DESC');

        $q = trim((string) ($filters['q'] ?? ''));
        if ($q !== '') {
            $qb
                ->andWhere('LOWER(badge.nom) LIKE :q OR LOWER(badge.description) LIKE :q OR LOWER(badge.icone) LIKE :q')
                ->setParameter('q', '%' . self::escapeLike(mb_strtolower($q)) . '%');
        }

        return $qb->getQuery()->getResult();
    }

    private static function escapeLike(string $value): string
    {
        return addcslashes($value, '%_');
    }
}
