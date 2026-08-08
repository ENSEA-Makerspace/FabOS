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

    /** The catalogue's two halves, and the definition of "past" is the same one. */
    public const WHEN_UPCOMING = 'upcoming';
    public const WHEN_PAST = 'past';

    /**
     * The public catalogue: one filter (when) and one search box.
     *
     * ⚠️ Upcoming is soonest-first and past is most-recent-first. Both are
     * "nearest to now" — a list of past events that started with the oldest one
     * would open on whatever happened first in the lab's history.
     *
     * @return Event[]
     */
    public function findForCatalogue(?string $when, string $search = ''): array
    {
        $qb = $this->createQueryBuilder('e');
        $this->applyWhen($qb, $when);

        $search = trim($search);
        if ($search !== '') {
            $qb->andWhere('e.titre LIKE :q OR e.description LIKE :q OR e.lieu LIKE :q')
                ->setParameter('q', '%' . $search . '%');
        }

        return $qb->orderBy('e.dateDebut', $when === self::WHEN_PAST ? 'DESC' : 'ASC')
            ->getQuery()
            ->getResult();
    }

    /** Tile counts, over the UNFILTERED set — see the note in _catalogue.html.twig. */
    public function countWhen(?string $when): int
    {
        $qb = $this->createQueryBuilder('e')->select('COUNT(e.id)');
        $this->applyWhen($qb, $when);

        return (int) $qb->getQuery()->getSingleScalarResult();
    }

    private function applyWhen(\Doctrine\ORM\QueryBuilder $qb, ?string $when): void
    {
        if ($when !== self::WHEN_UPCOMING && $when !== self::WHEN_PAST) {
            return;
        }

        // Same rule as findUpcoming(): an event is still upcoming until its end
        // (or its start, when it has no end) has passed. Stated once, here, so
        // the tile count and the list it labels cannot disagree.
        $qb->andWhere(sprintf('COALESCE(e.dateFin, e.dateDebut) %s :now', $when === self::WHEN_PAST ? '<' : '>='))
            ->setParameter('now', new \DateTimeImmutable('now'));
    }
}
