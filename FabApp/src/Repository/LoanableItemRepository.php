<?php

namespace App\Repository;

use App\Entity\LoanableItem;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<LoanableItem>
 */
class LoanableItemRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, LoanableItem::class);
    }

    /**
     * All loanable items for the public catalogue. Fail-safe: returns [] if the
     * table doesn't exist yet (deployed ahead of its migration).
     *
     * @return LoanableItem[]
     */
    public function findAllSafe(): array
    {
        try {
            return $this->createQueryBuilder('item')
                ->orderBy('item.category', 'ASC')
                ->addOrderBy('item.name', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }
}
