<?php

namespace App\Repository;

use App\Entity\Loan;
use App\Entity\LoanableItem;
use App\Entity\Utilisateur;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Loan>
 */
class LoanRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Loan::class);
    }

    /**
     * All loans, most recent first, with item + borrower joined. Fail-safe.
     *
     * @return Loan[]
     */
    public function findAllSafe(): array
    {
        try {
            return $this->createQueryBuilder('loan')
                ->leftJoin('loan.item', 'item')->addSelect('item')
                ->leftJoin('loan.borrower', 'borrower')->addSelect('borrower')
                ->orderBy('loan.dateTaken', 'DESC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * Loans belonging to a given registered borrower, most recent first.
     * Fail-safe so the profile page never breaks pre-migration.
     *
     * @return Loan[]
     */
    public function findForBorrower(Utilisateur $user): array
    {
        try {
            return $this->createQueryBuilder('loan')
                ->leftJoin('loan.item', 'item')->addSelect('item')
                ->andWhere('loan.borrower = :user')
                ->setParameter('user', $user)
                ->orderBy('loan.dateTaken', 'DESC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /** Number of active (not-returned) loans for an item — for availability. */
    public function countActiveForItem(LoanableItem $item): int
    {
        try {
            return (int) $this->createQueryBuilder('loan')
                ->select('COUNT(loan.id)')
                ->andWhere('loan.item = :item')
                ->andWhere('loan.status = :out')
                ->setParameter('item', $item)
                ->setParameter('out', Loan::STATUS_OUT)
                ->getQuery()
                ->getSingleScalarResult();
        } catch (\Throwable) {
            return 0;
        }
    }

    /**
     * Active-loan counts keyed by item id, in one query (avoids N+1 on the
     * catalogue / admin item list). Fail-safe → empty map.
     *
     * @return array<int, int>
     */
    public function activeCountsByItem(): array
    {
        try {
            $rows = $this->createQueryBuilder('loan')
                ->select('IDENTITY(loan.item) AS itemId, COUNT(loan.id) AS c')
                ->andWhere('loan.status = :out')
                ->setParameter('out', Loan::STATUS_OUT)
                ->groupBy('loan.item')
                ->getQuery()
                ->getScalarResult();
        } catch (\Throwable) {
            return [];
        }

        $map = [];
        foreach ($rows as $row) {
            $map[(int) $row['itemId']] = (int) $row['c'];
        }

        return $map;
    }
}
