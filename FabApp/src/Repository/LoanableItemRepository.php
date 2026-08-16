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
    /**
     * ⚠️ `$includeArchived` defaults to FALSE (S133). Every caller of this method
     * is a catalogue — the public loans page and the admin list — and a retired
     * object belongs in neither by default. The admin list passes `true` and
     * separates them with a tile, which is the one place the distinction is the
     * subject rather than noise.
     */
    public function findAllSafe(bool $includeArchived = false): array
    {
        try {
            $qb = $this->createQueryBuilder('item')
                ->orderBy('item.category', 'ASC')
                ->addOrderBy('item.name', 'ASC');
            if (!$includeArchived) {
                $qb->andWhere('item.archivedAt IS NULL');
            }

            return $qb->getQuery()->getResult();
        } catch (\Throwable) {
            return [];
        }
    }
}
