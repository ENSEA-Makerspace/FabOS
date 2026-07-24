<?php

namespace App\Repository;

use App\Entity\Event;
use App\Entity\EventRegistration;
use App\Entity\Utilisateur;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<EventRegistration>
 *
 * Fail-safe on reads, like the other module repositories: an event page must
 * still render before the registration migration has been applied.
 */
class EventRegistrationRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, EventRegistration::class);
    }

    /** Seats taken. Deliberately a live count — see the migration for why. */
    public function countSeatsTaken(Event $event): int
    {
        try {
            return (int) $this->createQueryBuilder('reg')
                ->select('COUNT(reg.id)')
                ->andWhere('reg.event = :event')
                ->andWhere('reg.status IN (:active)')
                ->setParameter('event', $event)
                ->setParameter('active', EventRegistration::ACTIVE_STATUSES)
                ->getQuery()
                ->getSingleScalarResult();
        } catch (\Throwable) {
            return 0;
        }
    }

    public function countWaitlisted(Event $event): int
    {
        try {
            return (int) $this->createQueryBuilder('reg')
                ->select('COUNT(reg.id)')
                ->andWhere('reg.event = :event')
                ->andWhere('reg.status = :waitlisted')
                ->setParameter('event', $event)
                ->setParameter('waitlisted', EventRegistration::STATUS_WAITLISTED)
                ->getQuery()
                ->getSingleScalarResult();
        } catch (\Throwable) {
            return 0;
        }
    }

    /**
     * The whole list for an organiser: seated first, then the queue in the order
     * it formed, then the people who dropped out.
     *
     * @return EventRegistration[]
     */
    public function findForEvent(Event $event): array
    {
        try {
            return $this->createQueryBuilder('reg')
                ->leftJoin('reg.utilisateur', 'user')->addSelect('user')
                ->andWhere('reg.event = :event')
                ->setParameter('event', $event)
                ->addOrderBy('CASE reg.status WHEN \'registered\' THEN 0 WHEN \'waitlisted\' THEN 1 ELSE 2 END', 'ASC')
                ->addOrderBy('reg.createdAt', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * The next person owed a seat: longest-waiting first, so the queue is
     * honoured rather than whoever happens to be sorted first by id.
     */
    public function findNextWaitlisted(Event $event): ?EventRegistration
    {
        try {
            return $this->createQueryBuilder('reg')
                ->andWhere('reg.event = :event')
                ->andWhere('reg.status = :waitlisted')
                ->setParameter('event', $event)
                ->setParameter('waitlisted', EventRegistration::STATUS_WAITLISTED)
                ->orderBy('reg.createdAt', 'ASC')
                ->setMaxResults(1)
                ->getQuery()
                ->getOneOrNullResult();
        } catch (\Throwable) {
            return null;
        }
    }

    /**
     * Everyone who should be reminded about an event starting in a window.
     *
     * Deliberately narrow on three counts: only seat-holders (a waitlisted
     * person has nothing to turn up to), only events that haven't been called
     * off (reminding people about a cancelled event is worse than silence), and
     * only events with a start date at all.
     *
     * @return EventRegistration[]
     */
    public function findForEventsStartingBetween(\DateTimeImmutable $from, \DateTimeImmutable $to): array
    {
        try {
            return $this->createQueryBuilder('reg')
                ->join('reg.event', 'event')->addSelect('event')
                ->leftJoin('reg.utilisateur', 'user')->addSelect('user')
                ->andWhere('reg.status IN (:active)')
                ->andWhere('event.cancelledAt IS NULL')
                ->andWhere('event.dateDebut >= :from')
                ->andWhere('event.dateDebut < :to')
                ->setParameter('active', EventRegistration::ACTIVE_STATUSES)
                ->setParameter('from', $from)
                ->setParameter('to', $to)
                ->orderBy('event.dateDebut', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /** The existing row for this address, whatever state it is in. */
    public function findOneForContact(Event $event, string $email): ?EventRegistration
    {
        try {
            return $this->createQueryBuilder('reg')
                ->andWhere('reg.event = :event')
                ->andWhere('reg.contactEmail = :email')
                ->setParameter('event', $event)
                ->setParameter('email', EventRegistration::normaliseEmail($email))
                ->setMaxResults(1)
                ->getQuery()
                ->getOneOrNullResult();
        } catch (\Throwable) {
            return null;
        }
    }

    /**
     * A member's own registrations, for their profile. Guest rows are not
     * reachable here by design — they belong to an address, not an account.
     *
     * @return EventRegistration[]
     */
    public function findForUser(Utilisateur $user): array
    {
        try {
            return $this->createQueryBuilder('reg')
                ->leftJoin('reg.event', 'event')->addSelect('event')
                ->andWhere('reg.utilisateur = :user')
                ->andWhere('reg.status != :cancelled')
                ->setParameter('user', $user)
                ->setParameter('cancelled', EventRegistration::STATUS_CANCELLED)
                ->orderBy('event.dateDebut', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * Seats taken across several events in one query — the events list would
     * otherwise run a count per card.
     *
     * @param Event[] $events
     *
     * @return array<int, int> eventId => seats taken
     */
    public function seatsTakenByEvent(array $events): array
    {
        if ($events === []) {
            return [];
        }

        try {
            $rows = $this->createQueryBuilder('reg')
                ->select('IDENTITY(reg.event) AS eventId, COUNT(reg.id) AS taken')
                ->andWhere('reg.event IN (:events)')
                ->andWhere('reg.status IN (:active)')
                ->setParameter('events', $events)
                ->setParameter('active', EventRegistration::ACTIVE_STATUSES)
                ->groupBy('reg.event')
                ->getQuery()
                ->getScalarResult();
        } catch (\Throwable) {
            return [];
        }

        $map = [];
        foreach ($rows as $row) {
            $map[(int) $row['eventId']] = (int) $row['taken'];
        }

        return $map;
    }
}
