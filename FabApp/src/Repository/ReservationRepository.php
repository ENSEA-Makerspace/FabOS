<?php

namespace App\Repository;

use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Reservation\ReservableType;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * Queries here address the booked resource polymorphically, by
 * (reservableType, reservableId) — there is no join to a machine or place any
 * more. Resource names for display come from ReservableResolver; the
 * reservableLabel snapshot is what admin search matches on.
 *
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
            ->andWhere('reservation.statut NOT IN (:inactive)')
            ->setParameter('inactive', Reservation::INACTIVE_STATUSES);

        return $this->ordered($qb, $orderBy)->getQuery()->getResult();
    }

    /** @return Reservation[] */
    public function findActiveForReservable(ReservableType $type, int $id, array $orderBy = ['dateDebut' => 'ASC']): array
    {
        $qb = $this->reservableQuery($type, $id)
            ->andWhere('reservation.statut NOT IN (:inactive)')
            ->setParameter('inactive', Reservation::INACTIVE_STATUSES);

        return $this->ordered($qb, $orderBy)->getQuery()->getResult();
    }

    /** Every booking of a resource, cancelled ones included (history views). @return Reservation[] */
    public function findForReservable(ReservableType $type, int $id, array $orderBy = ['dateDebut' => 'DESC']): array
    {
        return $this->ordered($this->reservableQuery($type, $id), $orderBy)->getQuery()->getResult();
    }

    public function countForReservable(ReservableType $type, int $id): int
    {
        return (int) $this->reservableQuery($type, $id)
            ->select('COUNT(reservation.id)')
            ->getQuery()
            ->getSingleScalarResult();
    }

    /** @return Reservation[] */
    public function findForUser(Utilisateur $user, array $orderBy = ['dateDebut' => 'DESC']): array
    {
        $qb = $this->createQueryBuilder('reservation')
            ->andWhere('reservation.utilisateur = :user')
            ->setParameter('user', $user);

        return $this->ordered($qb, $orderBy)->getQuery()->getResult();
    }

    /** Active bookings of a resource that haven't finished yet. @return Reservation[] */
    public function findUpcomingForReservable(ReservableType $type, int $id): array
    {
        $qb = $this->reservableQuery($type, $id)
            ->andWhere('reservation.statut NOT IN (:inactive)')
            ->andWhere('reservation.dateFin >= :now')
            ->setParameter('inactive', Reservation::INACTIVE_STATUSES)
            ->setParameter('now', new \DateTimeImmutable());

        return $this->ordered($qb, ['dateDebut' => 'ASC'])->getQuery()->getResult();
    }

    /**
     * Requests still waiting on a decision from the person they were sent to.
     * Past ones are dropped rather than shown as a stale to-do: a request for
     * last Tuesday isn't answerable any more.
     *
     * @return Reservation[]
     */
    public function findPendingForReservable(ReservableType $type, int $id): array
    {
        $qb = $this->reservableQuery($type, $id)
            ->andWhere('reservation.statut = :pending')
            ->andWhere('reservation.dateFin >= :now')
            ->setParameter('pending', Reservation::STATUS_PENDING)
            ->setParameter('now', new \DateTimeImmutable());

        return $this->ordered($qb, ['dateDebut' => 'ASC'])->getQuery()->getResult();
    }

    public function countPendingForReservable(ReservableType $type, int $id): int
    {
        return count($this->findPendingForReservable($type, $id));
    }

    public function hasOverlap(ReservableType $type, int $id, \DateTimeImmutable $dateDebut, \DateTimeImmutable $dateFin, ?int $ignoreId = null): bool
    {
        $qb = $this->reservableQuery($type, $id)
            ->select('COUNT(reservation.id)')
            ->andWhere('reservation.statut NOT IN (:inactive)')
            ->andWhere('reservation.dateDebut < :dateFin')
            ->andWhere('reservation.dateFin > :dateDebut')
            ->setParameter('inactive', Reservation::INACTIVE_STATUSES)
            ->setParameter('dateDebut', $dateDebut)
            ->setParameter('dateFin', $dateFin);

        if ($ignoreId !== null) {
            $qb->andWhere('reservation.id != :ignoreId')->setParameter('ignoreId', $ignoreId);
        }

        return (bool) $qb->getQuery()->getSingleScalarResult();
    }

    /**
     * Cancel every future booking of a resource that is being deleted. Replaces
     * the ON DELETE CASCADE the machineId/placeId FKs used to provide — past
     * bookings are kept (with their reservableLabel) so history stays readable.
     */
    /**
     * The bookings cancelUpcomingForReservable() is about to cancel. Call it first
     * when the people holding them have to be told — the bulk UPDATE below reports
     * only a count, and once it has run there is no way to find them again.
     *
     * @return Reservation[]
     */
    public function findUpcomingActiveForReservable(ReservableType $type, int $id): array
    {
        return $this->createQueryBuilder('reservation')
            ->andWhere('reservation.reservableType = :reservableType')
            ->andWhere('reservation.reservableId = :reservableId')
            ->andWhere('reservation.statut NOT IN (:inactive)')
            ->andWhere('reservation.dateFin >= :now')
            ->setParameter('reservableType', $type->value)
            ->setParameter('reservableId', $id)
            ->setParameter('inactive', Reservation::INACTIVE_STATUSES)
            ->setParameter('now', new \DateTimeImmutable())
            ->getQuery()
            ->getResult();
    }

    /**
     * How many still-active bookings this person holds that haven't finished —
     * the "you already have N on the go" quota. Counts pending requests too: a
     * slot someone is waiting on an answer for is still a slot they're holding.
     */
    public function countActiveUpcomingForUser(Utilisateur $user, ?\DateTimeImmutable $now = null): int
    {
        try {
            return (int) $this->createQueryBuilder('reservation')
                ->select('COUNT(reservation.id)')
                ->andWhere('reservation.utilisateur = :user')
                ->andWhere('reservation.statut NOT IN (:inactive)')
                ->andWhere('reservation.dateFin >= :now')
                ->setParameter('user', $user)
                ->setParameter('inactive', Reservation::INACTIVE_STATUSES)
                ->setParameter('now', $now ?? new \DateTimeImmutable())
                ->getQuery()
                ->getSingleScalarResult();
        } catch (\Throwable) {
            return 0;
        }
    }

    /**
     * Active bookings this person has starting inside a window — backs the
     * per-day and per-week caps. Half-open [$from, $to) so a booking at
     * midnight belongs to exactly one day.
     */
    public function countForUserStartingBetween(Utilisateur $user, \DateTimeImmutable $from, \DateTimeImmutable $to): int
    {
        try {
            return (int) $this->createQueryBuilder('reservation')
                ->select('COUNT(reservation.id)')
                ->andWhere('reservation.utilisateur = :user')
                ->andWhere('reservation.statut NOT IN (:inactive)')
                ->andWhere('reservation.dateDebut >= :from')
                ->andWhere('reservation.dateDebut < :to')
                ->setParameter('user', $user)
                ->setParameter('inactive', Reservation::INACTIVE_STATUSES)
                ->setParameter('from', $from)
                ->setParameter('to', $to)
                ->getQuery()
                ->getSingleScalarResult();
        } catch (\Throwable) {
            return 0;
        }
    }

    /**
     * Bookings that start inside a window — what the reminder scanner sweeps.
     *
     * Pending requests are deliberately included: a slot someone is still
     * waiting on an answer for is exactly the one they want reminding about.
     * Fail-safe, because a reminder run must not die on a half-migrated schema.
     *
     * @return Reservation[]
     */
    public function findStartingBetween(\DateTimeImmutable $from, \DateTimeImmutable $to): array
    {
        try {
            return $this->createQueryBuilder('reservation')
                ->andWhere('reservation.statut NOT IN (:inactive)')
                ->andWhere('reservation.dateDebut >= :from')
                ->andWhere('reservation.dateDebut < :to')
                ->setParameter('inactive', Reservation::INACTIVE_STATUSES)
                ->setParameter('from', $from)
                ->setParameter('to', $to)
                ->orderBy('reservation.dateDebut', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    public function cancelUpcomingForReservable(ReservableType $type, int $id): int
    {
        return (int) $this->getEntityManager()
            ->createQuery(
                'UPDATE ' . Reservation::class . ' reservation'
                . ' SET reservation.statut = :cancelled'
                . ' WHERE reservation.reservableType = :reservableType'
                . ' AND reservation.reservableId = :reservableId'
                . ' AND reservation.statut NOT IN (:inactive)'
                . ' AND reservation.dateFin >= :now'
            )
            ->setParameter('cancelled', Reservation::STATUS_CANCELLED)
            ->setParameter('inactive', Reservation::INACTIVE_STATUSES)
            ->setParameter('reservableType', $type->value)
            ->setParameter('reservableId', $id)
            ->setParameter('now', new \DateTimeImmutable())
            ->execute();
    }

    /** @return Reservation[] */
    public function findForAdminFilters(array $filters): array
    {
        $qb = $this->createQueryBuilder('reservation')
            ->leftJoin('reservation.utilisateur', 'utilisateur')
            ->addSelect('utilisateur')
            ->orderBy('reservation.dateDebut', 'DESC');

        $q = trim((string) ($filters['q'] ?? ''));
        if ($q !== '') {
            // reservableLabel is the booking-time resource name, so searching by
            // resource needs no join and still finds deleted resources.
            $qb
                ->andWhere('LOWER(reservation.reservableLabel) LIKE :q OR LOWER(utilisateur.firstName) LIKE :q OR LOWER(utilisateur.lastName) LIKE :q OR LOWER(utilisateur.email) LIKE :q OR LOWER(utilisateur.username) LIKE :q OR LOWER(reservation.motif) LIKE :q')
                ->setParameter('q', '%' . self::escapeLike(mb_strtolower($q)) . '%');
        }

        $type = ReservableType::tryParse(trim((string) ($filters['reservableType'] ?? '')));
        if ($type !== null) {
            $qb
                ->andWhere('reservation.reservableType = :reservableType')
                ->setParameter('reservableType', $type->value);
        }

        $statut = trim((string) ($filters['statut'] ?? ''));
        if ($statut !== '' && $statut !== 'all') {
            if ($statut === 'completed') {
                $qb
                    ->andWhere('reservation.statut NOT IN (:inactive)')
                    ->andWhere('reservation.dateFin < :now')
                    ->setParameter('inactive', Reservation::INACTIVE_STATUSES)
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

    private function reservableQuery(ReservableType $type, int $id): \Doctrine\ORM\QueryBuilder
    {
        return $this->createQueryBuilder('reservation')
            ->andWhere('reservation.reservableType = :reservableType')
            ->andWhere('reservation.reservableId = :reservableId')
            ->setParameter('reservableType', $type->value)
            ->setParameter('reservableId', $id);
    }

    private function ordered(\Doctrine\ORM\QueryBuilder $qb, array $orderBy): \Doctrine\ORM\QueryBuilder
    {
        foreach ($orderBy as $field => $direction) {
            $qb->addOrderBy('reservation.' . $field, $direction);
        }

        return $qb;
    }

    private static function escapeLike(string $value): string
    {
        return addcslashes($value, '%_');
    }
}
