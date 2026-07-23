<?php

namespace App\Repository;

use App\Entity\Machine;
use App\Entity\MaintenanceTask;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<MaintenanceTask>
 */
class MaintenanceTaskRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, MaintenanceTask::class);
    }

    /**
     * All tasks (admin list), machine joined, newest/soonest first. Fail-safe.
     *
     * @return MaintenanceTask[]
     */
    public function findAllSafe(): array
    {
        return $this->querySafe(false);
    }

    /**
     * Open (pending) tasks only — for the public backlog page. Fail-safe.
     *
     * @return MaintenanceTask[]
     */
    public function findOpenSafe(): array
    {
        return $this->querySafe(true);
    }

    /**
     * Still-pending tasks whose due date has passed — what staff get chased
     * about. Undated tasks can't be overdue, so they're excluded. Fail-safe.
     *
     * @return MaintenanceTask[]
     */
    public function findOverdue(\DateTimeImmutable $asOf): array
    {
        try {
            return $this->createQueryBuilder('task')
                ->leftJoin('task.machine', 'machine')->addSelect('machine')
                ->andWhere('task.status = :pending')
                ->andWhere('task.dueDate IS NOT NULL')
                ->andWhere('task.dueDate < :asOf')
                ->setParameter('pending', MaintenanceTask::STATUS_PENDING)
                ->setParameter('asOf', $asOf)
                ->orderBy('task.dueDate', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /** @return MaintenanceTask[] */
    private function querySafe(bool $openOnly): array
    {
        try {
            $qb = $this->createQueryBuilder('task')
                ->leftJoin('task.machine', 'machine')->addSelect('machine')
                ->leftJoin('task.doneBy', 'doneBy')->addSelect('doneBy');

            if ($openOnly) {
                $qb->andWhere('task.status = :pending')->setParameter('pending', MaintenanceTask::STATUS_PENDING);
            }

            // Soonest due first (nulls last), then most recent.
            $qb->addOrderBy('CASE WHEN task.dueDate IS NULL THEN 1 ELSE 0 END', 'ASC')
                ->addOrderBy('task.dueDate', 'ASC')
                ->addOrderBy('task.createdAt', 'DESC');

            return $qb->getQuery()->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * Open tasks for one machine (for the machine detail page). Fail-safe.
     *
     * @return MaintenanceTask[]
     */
    public function findOpenForMachine(Machine $machine): array
    {
        try {
            return $this->createQueryBuilder('task')
                ->andWhere('task.machine = :machine')
                ->andWhere('task.status = :pending')
                ->setParameter('machine', $machine)
                ->setParameter('pending', MaintenanceTask::STATUS_PENDING)
                ->addOrderBy('CASE WHEN task.dueDate IS NULL THEN 1 ELSE 0 END', 'ASC')
                ->addOrderBy('task.dueDate', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * Worst pending health per machine: machineId → 'overdue' | 'due_soon'.
     * Machines with no urgent pending task are absent (treated as healthy).
     * One query; computed in PHP. Fail-safe → empty map.
     *
     * @return array<int, string>
     */
    public function healthByMachine(): array
    {
        try {
            $rows = $this->createQueryBuilder('task')
                ->select('IDENTITY(task.machine) AS machineId, task.dueDate AS dueDate')
                ->andWhere('task.status = :pending')
                ->andWhere('task.dueDate IS NOT NULL')
                ->setParameter('pending', MaintenanceTask::STATUS_PENDING)
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }

        $today = new \DateTimeImmutable('today');
        $soon = $today->modify('+7 days');
        $health = [];
        foreach ($rows as $row) {
            $machineId = (int) $row['machineId'];
            $due = $row['dueDate'];
            if (!$due instanceof \DateTimeInterface) {
                continue;
            }
            $status = $due < $today ? 'overdue' : ($due <= $soon ? 'due_soon' : null);
            if ($status === null) {
                continue;
            }
            // 'overdue' wins over 'due_soon'.
            if (($health[$machineId] ?? null) !== 'overdue') {
                $health[$machineId] = $status;
            }
        }

        return $health;
    }
}
