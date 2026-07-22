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
}
