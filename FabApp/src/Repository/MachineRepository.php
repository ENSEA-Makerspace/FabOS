<?php

namespace App\Repository;

use App\Entity\Machine;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Machine>
 */
class MachineRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Machine::class);
    }

    public function findOneByMachineToken(string $machineToken): ?Machine
    {
        $machineToken = trim($machineToken);
        if ($machineToken === '') {
            return null;
        }

        return $this->findOneBy(['machineToken' => $machineToken]);
    }
}
