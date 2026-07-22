<?php

namespace App\Repository;

use App\Entity\Event;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Event>
 */
class EventRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Event::class);
    }

    /**
     * Upcoming events (still running or starting in the future), soonest first.
     * An event counts as "upcoming" until its end (or its start, if it has no
     * end) has passed.
     *
     * @return Event[]
     */
    public function findUpcoming(?int $limit = null): array
    {
        $now = new \DateTimeImmutable('now');
        $qb = $this->createQueryBuilder('e')
            ->andWhere('COALESCE(e.dateFin, e.dateDebut) >= :now')
            ->setParameter('now', $now)
            ->orderBy('e.dateDebut', 'ASC');

        if ($limit !== null) {
            $qb->setMaxResults($limit);
        }

        return $qb->getQuery()->getResult();
    }
}
