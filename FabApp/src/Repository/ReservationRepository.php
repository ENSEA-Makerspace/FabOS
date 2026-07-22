<?php

namespace App\Repository;

use App\Entity\Machine;
use App\Entity\Place;
use App\Entity\Reservation;
use App\Entity\Utilisateur;
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


    /** @return Reservation[] */
    public function findForUser(Utilisateur $user, array $orderBy = ['dateDebut' => 'DESC']): array
    {
        $qb = $this->createQueryBuilder('reservation')
            ->leftJoin('reservation.machine', 'machine')
            ->leftJoin('reservation.place', 'place')
            ->addSelect('machine', 'place')
            ->andWhere('reservation.utilisateur = :user')
            ->setParameter('user', $user);

        foreach ($orderBy as $field => $direction) {
            $qb->addOrderBy('reservation.' . $field, $direction);
        }

        return $qb->getQuery()->getResult();
    }

    /** @return Reservation[] */
    public function findActiveByPlace(Place $place, array $orderBy = ['dateDebut' => 'ASC']): array
    {
        $qb = $this->createQueryBuilder('reservation')
            ->andWhere('reservation.place = :place')
            ->andWhere('reservation.statut != :cancelled')
            ->setParameter('place', $place)
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

    public function hasOverlapForPlace(Place $place, \DateTimeImmutable $dateDebut, \DateTimeImmutable $dateFin): bool
    {
        return (bool) $this->createQueryBuilder('reservation')
            ->select('COUNT(reservation.id)')
            ->andWhere('reservation.place = :place')
            ->andWhere('reservation.statut != :cancelled')
            ->andWhere('reservation.dateDebut < :dateFin')
            ->andWhere('reservation.dateFin > :dateDebut')
            ->setParameter('place', $place)
            ->setParameter('cancelled', 'cancelled')
            ->setParameter('dateDebut', $dateDebut)
            ->setParameter('dateFin', $dateFin)
            ->getQuery()
            ->getSingleScalarResult();
    }
    /** @return Reservation[] */
    public function findForAdminFilters(array $filters): array
    {
        $qb = $this->createQueryBuilder('reservation')
            ->leftJoin('reservation.utilisateur', 'utilisateur')
            ->leftJoin('reservation.machine', 'machine')
            ->leftJoin('reservation.place', 'place')
            ->addSelect('utilisateur', 'machine', 'place')
            ->orderBy('reservation.dateDebut', 'DESC');

        $q = trim((string) ($filters['q'] ?? ''));
        if ($q !== '') {
            $qb
                ->andWhere('LOWER(machine.nom) LIKE :q OR LOWER(place.nom) LIKE :q OR LOWER(utilisateur.firstName) LIKE :q OR LOWER(utilisateur.lastName) LIKE :q OR LOWER(utilisateur.email) LIKE :q OR LOWER(utilisateur.username) LIKE :q OR LOWER(reservation.motif) LIKE :q')
                ->setParameter('q', '%' . self::escapeLike(mb_strtolower($q)) . '%');
        }

        $statut = trim((string) ($filters['statut'] ?? ''));
        if ($statut !== '' && $statut !== 'all') {
            if ($statut === 'completed') {
                $qb
                    ->andWhere('reservation.statut != :cancelledStatus')
                    ->andWhere('reservation.dateFin < :now')
                    ->setParameter('cancelledStatus', 'cancelled')
                    ->setParameter('now', new \DateTimeImmutable());
            } else {
                $qb
                    ->andWhere('reservation.statut = :statut')
                    ->setParameter('statut', $statut);
            }
        }

        $dateFrom = trim((string) ($filters['dateFrom'] ?? ''));
        if ($dateFrom !== '') {
            $from = \DateTimeImmutable::createFromFormat('!Y-m-d', $dateFrom);
            if ($from instanceof \DateTimeImmutable) {
                $qb
                    ->andWhere('reservation.dateDebut >= :dateFrom')
                    ->setParameter('dateFrom', $from);
            }
        }

        $dateTo = trim((string) ($filters['dateTo'] ?? ''));
        if ($dateTo !== '') {
            $to = \DateTimeImmutable::createFromFormat('!Y-m-d', $dateTo);
            if ($to instanceof \DateTimeImmutable) {
                $qb
                    ->andWhere('reservation.dateDebut <= :dateTo')
                    ->setParameter('dateTo', $to->modify('+1 day -1 second'));
            }
        }

        return $qb->getQuery()->getResult();
    }

    private static function escapeLike(string $value): string
    {
        return addcslashes($value, '%_');
    }
}
