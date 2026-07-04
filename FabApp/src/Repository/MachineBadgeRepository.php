<?php

namespace App\Repository;

use App\Entity\Machine;
use App\Entity\MachineBadge;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<MachineBadge>
 */
class MachineBadgeRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, MachineBadge::class);
    }

    /**
     * @return MachineBadge[]
     */
    public function findRequiredForMachine(Machine $machine): array
    {
        return $this->findBy(['machine' => $machine, 'requiredForAccess' => true], ['id' => 'ASC']);
    }
}
