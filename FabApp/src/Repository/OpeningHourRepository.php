<?php

namespace App\Repository;

use App\Entity\OpeningHour;
use App\Entity\Venue;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<OpeningHour>
 */
class OpeningHourRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, OpeningHour::class);
    }

    /** @return OpeningHour[] */
    public function findOrdered(Venue $venue): array
    {
        return $this->findBy(['venue' => $venue], ['sortOrder' => 'ASC', 'dayOfWeek' => 'ASC']);
    }

    /**
     * One level of one location's schedule (S134d).
     *
     * ⚠️ **`findOrdered()` returns EVERY level**, which is right for a delete
     * sweep and wrong for a screen: without this the hours page would show a
     * machine's Saturday beside the location's, with nothing to say which was
     * which. Scope is matched exactly — null is a value here, not a wildcard.
     *
     * @return list<OpeningHour>
     */
    public function findOrderedForScope(Venue $venue, ?string $scopeType, ?int $scopeId): array
    {
        $qb = $this->createQueryBuilder('h')
            ->andWhere('h.venue = :venue')
            ->setParameter('venue', $venue)
            ->orderBy('h.sortOrder', 'ASC')
            ->addOrderBy('h.dayOfWeek', 'ASC');

        $qb->andWhere($scopeType === null ? 'h.scopeType IS NULL' : 'h.scopeType = :type');
        if ($scopeType !== null) {
            $qb->setParameter('type', $scopeType);
        }
        $qb->andWhere($scopeId === null ? 'h.scopeId IS NULL' : 'h.scopeId = :sid');
        if ($scopeId !== null) {
            $qb->setParameter('sid', $scopeId);
        }

        try {
            return array_values($qb->getQuery()->getResult());
        } catch (\Throwable) {
            // ⚠️ An install without the S134d scope columns answers with the
            // location's week, which is every row it has. Failing to "no hours"
            // would read as a closed lab.
            return $scopeType === null ? $this->findOrdered($venue) : [];
        }
    }

    /**
     * Which resources at this location carry their own schedule — for the editor's
     * picker, so an operator can find what they wrote without remembering it.
     *
     * @return list<array{scopeType: string, scopeId: ?int}>
     */
    public function scopesWithRows(Venue $venue): array
    {
        try {
            $rows = $this->createQueryBuilder('h')
                ->select('DISTINCT h.scopeType AS scopeType, h.scopeId AS scopeId')
                ->andWhere('h.venue = :venue')
                ->andWhere('h.scopeType IS NOT NULL')
                ->setParameter('venue', $venue)
                ->getQuery()->getArrayResult();
        } catch (\Throwable) {
            return [];
        }

        return array_map(static fn (array $row): array => [
            'scopeType' => (string) $row['scopeType'],
            'scopeId' => $row['scopeId'] !== null ? (int) $row['scopeId'] : null,
        ], $rows);
    }
}
