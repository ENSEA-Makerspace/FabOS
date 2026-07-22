<?php

namespace App\Repository;

use App\Entity\LabPage;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<LabPage>
 */
class LabPageRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, LabPage::class);
    }

    /** @return LabPage[] */
    public function findTopLevel(): array
    {
        return $this->createQueryBuilder('page')
            ->andWhere('page.parentPage IS NULL')
            ->orderBy('page.position', 'ASC')
            ->addOrderBy('page.titre', 'ASC')
            ->getQuery()
            ->getResult();
    }

    /**
     * Top-level pages with their children eagerly hydrated in a single query,
     * for rendering the Lab hierarchy in the nav bar without an N+1.
     *
     * @return LabPage[]
     */
    public function findTopLevelWithChildren(): array
    {
        return $this->createQueryBuilder('page')
            ->leftJoin('page.children', 'child')
            ->addSelect('child')
            ->andWhere('page.parentPage IS NULL')
            ->orderBy('page.position', 'ASC')
            ->addOrderBy('page.titre', 'ASC')
            ->addOrderBy('child.position', 'ASC')
            ->addOrderBy('child.titre', 'ASC')
            ->getQuery()
            ->getResult();
    }
}
