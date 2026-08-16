<?php

namespace App\Repository;

use App\Entity\MachineCategory;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<MachineCategory>
 */
class MachineCategoryRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, MachineCategory::class);
    }

    /**
     * ⚠️ **Fail-safe on read, like the other tables that arrive by migration.**
     * `MACHINE_CATEGORY` is new in S133 and the operator runs migrations by hand,
     * so between a code deploy and that run the table does not exist. An admin
     * screen that 500s in that window would be indistinguishable from a broken
     * deploy; an empty catalogue plus the labels already in use is the honest
     * answer, and it is what the categories screen falls back to.
     *
     * @return MachineCategory[]
     */
    public function allOrdered(bool $includeArchived = true): array
    {
        try {
            $qb = $this->createQueryBuilder('c')->orderBy('c.label', 'ASC');
            if (!$includeArchived) {
                $qb->andWhere('c.archivedAt IS NULL');
            }

            return $qb->getQuery()->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    public function findOneByLabel(string $label): ?MachineCategory
    {
        try {
            return $this->findOneBy(['label' => trim($label)]);
        } catch (\Throwable) {
            return null;
        }
    }

    public function tableExists(): bool
    {
        try {
            $this->createQueryBuilder('c')->select('COUNT(c.id)')->getQuery()->getSingleScalarResult();

            return true;
        } catch (\Throwable) {
            return false;
        }
    }
}
