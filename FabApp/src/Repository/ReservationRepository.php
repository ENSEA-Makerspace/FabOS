<?php

namespace App\Repository;

use App\Entity\Machine;
use App\Entity\Reservation;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Reservation>
 */
class ReservationRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Reservation::class);
    }

    /** @return Reservation[] */
    public function findAllActive(array $orderBy = ['dateDebut' => 'ASC']): array
    {
        $qb = $this->createQueryBuilder('reservation')
            ->andWhere('reservation.statut != :cancelled')
            ->setParameter('cancelled', 'cancelled');

        foreach ($orderBy as $field => $direction) {
            $qb->addOrderBy('reservation.' . $field, $direction);
        }

        return $qb->getQuery()->getResult();
    }

    /** @return Reservation[] */
    public function findActiveByMachine(Machine $machine, array $orderBy = ['dateDebut' => 'ASC']): array
    {
        $qb = $this->createQueryBuilder('reservation')
            ->andWhere('reservation.machine = :machine')
            ->andWhere('reservation.statut != :cancelled')
            ->setParameter('machine', $machine)
            ->setParameter('cancelled', 'cancelled');

        foreach ($orderBy as $field => $direction) {
            $qb->addOrderBy('reservation.' . $field, $direction);
        }

        return $qb->getQuery()->getResult();
    }

    public function hasOverlap(Machine $machine, \DateTimeImmutable $dateDebut, \DateTimeImmutable $dateFin): bool
    {
        return (bool) $this->createQueryBuilder('reservation')
            ->select('COUNT(reservation.id)')
            ->andWhere('reservation.machine = :machine')
            ->andWhere('reservation.statut != :cancelled')
            ->andWhere('reservation.dateDebut < :dateFin')
            ->andWhere('reservation.dateFin > :dateDebut')
            ->setParameter('machine', $machine)
            ->setParameter('cancelled', 'cancelled')
            ->setParameter('dateDebut', $dateDebut)
            ->setParameter('dateFin', $dateFin)
            ->getQuery()
            ->getSingleScalarResult();
    }
}

