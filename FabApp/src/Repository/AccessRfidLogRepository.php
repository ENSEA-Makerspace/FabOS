<?php

namespace App\Repository;

use App\Entity\AccessRfidLog;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<AccessRfidLog>
 */
class AccessRfidLogRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, AccessRfidLog::class);
    }

    /**
     * The admin journal, filtered — period, reader, machine and result (S132).
     *
     * ⚠️ **Server-side, and the reason is the same one the mail log had.** The
     * screen fetched the hundred most recent rows and filtered them in the
     * browser, so on a door that is badged fifty times a day the hundredth row is
     * yesterday afternoon: "was Camille refused on Tuesday?" had no answer, and
     * the tile row on top of those hundred rows made it look as though it did.
     *
     * `$days === 0` means no period bound. `$result` is `'yes'`, `'no'` or null.
     *
     * @return AccessRfidLog[]
     */
    public function search(int $days = 30, ?int $readerId = null, ?int $machineId = null, ?string $result = null, int $limit = 200): array
    {
        $qb = $this->filtered($days, $readerId, $machineId, $result)
            ->select('l')
            // Eager-joined: four columns of a hundred rows each dereference the
            // user, the machine and the reader, which is 300 queries otherwise.
            ->leftJoin('l.utilisateur', 'u')->addSelect('u')
            ->leftJoin('l.machine', 'm')->addSelect('m')
            ->leftJoin('l.reader', 'r')->addSelect('r')
            ->orderBy('l.createdAt', 'DESC')
            ->setMaxResults(max(1, min($limit, 500)));

        return $qb->getQuery()->getResult();
    }

    /** How many rows the same filter matches, which is not how many are shown. */
    public function countMatching(int $days = 30, ?int $readerId = null, ?int $machineId = null, ?string $result = null): int
    {
        return (int) $this->filtered($days, $readerId, $machineId, $result)
            ->select('COUNT(l.id)')
            ->getQuery()
            ->getSingleScalarResult();
    }

    /** Result counts for the tile row, under the period and place filters only. */
    /** @return array{all: int, yes: int, no: int} */
    public function resultCounts(int $days = 30, ?int $readerId = null, ?int $machineId = null): array
    {
        return [
            'all' => $this->countMatching($days, $readerId, $machineId, null),
            'yes' => $this->countMatching($days, $readerId, $machineId, 'yes'),
            'no' => $this->countMatching($days, $readerId, $machineId, 'no'),
        ];
    }

    private function filtered(int $days, ?int $readerId, ?int $machineId, ?string $result): \Doctrine\ORM\QueryBuilder
    {
        $qb = $this->createQueryBuilder('l');

        if ($days > 0) {
            // ⚠️ `createdAt` is a machine timestamp written by the device path, so
            // the bound is computed in the server's zone. Reading the lab's
            // configured timezone here would shift the window by its offset.
            $qb->andWhere('l.createdAt >= :since')
                ->setParameter('since', new \DateTimeImmutable(sprintf('-%d days', $days)));
        }
        if ($readerId !== null) {
            $qb->andWhere('l.reader = :reader')->setParameter('reader', $readerId);
        }
        if ($machineId !== null) {
            $qb->andWhere('l.machine = :machine')->setParameter('machine', $machineId);
        }
        if ($result === 'yes' || $result === 'no') {
            $qb->andWhere('l.authorized = :authorized')->setParameter('authorized', $result === 'yes');
        }

        return $qb;
    }

    /**
     * Most recent access events that are tied to a known user, for the "last entries"
     * wall display. Eager-joins the user to avoid per-row queries.
     *
     * @return AccessRfidLog[]
     */
    public function findRecentWithUser(int $limit = 12): array
    {
        return $this->createQueryBuilder('l')
            ->addSelect('u')
            ->join('l.utilisateur', 'u')
            ->orderBy('l.createdAt', 'DESC')
            ->setMaxResults($limit)
            ->getQuery()
            ->getResult();
    }
}
