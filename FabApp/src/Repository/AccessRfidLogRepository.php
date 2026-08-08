<?php

namespace App\Repository;

use App\Entity\AccessRfidLog;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<AccessRfidLog>
 */
class AccessRfidLogRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, AccessRfidLog::class);
    }

    /**
     * Most recent access events that are tied to a known user, for the "last entries"
     * wall display. Eager-joins the user to avoid per-row queries.
     *
     * @return AccessRfidLog[]
     */
    public function findRecentWithUser(int $limit = 12): array
    {
        return $this->createQueryBuilder('l')
            ->addSelect('u')
            ->join('l.utilisateur', 'u')
            ->orderBy('l.createdAt', 'DESC')
            ->setMaxResults($limit)
            ->getQuery()
            ->getResult();
    }
}
