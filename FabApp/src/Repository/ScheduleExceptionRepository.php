<?php

declare(strict_types=1);

namespace App\Repository;

use App\Entity\ScheduleException;
use App\Entity\Venue;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * Dated exceptions to a location's week (S134d).
 *
 * ⚠️ **Every read fails safe to "no exception", never to "closed".** An
 * exception can only ever narrow or move a location's opening, so losing one to
 * a missing table costs a closure that should have applied — annoying — whereas
 * inventing one would shut a lab that is open. That direction is the whole
 * reason these methods swallow rather than throw.
 */
class ScheduleExceptionRepository extends ServiceEntityRepository
{
    private ?bool $tableExists = null;

    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, ScheduleException::class);
    }

    /**
     * The rows that decide one date, ordered so the earliest opening comes first.
     *
     * @return list<ScheduleException>
     */
    public function forDate(Venue $venue, \DateTimeInterface $date): array
    {
        if (!$this->tableExists()) {
            return [];
        }

        try {
            return array_values($this->createQueryBuilder('e')
                ->andWhere('e.venue = :venue')
                ->andWhere('e.exceptionDate = :date')
                ->setParameter('venue', $venue)
                ->setParameter('date', new \DateTimeImmutable($date->format('Y-m-d')))
                ->orderBy('e.openTime', 'ASC')
                ->getQuery()->getResult());
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * Upcoming exceptions for the editing screen.
     *
     * ⚠️ Bounded from today rather than showing everything: an install three
     * years old would otherwise open this screen on a list of dead holidays and
     * bury the two that matter.
     *
     * @return list<ScheduleException>
     */
    public function upcomingFor(Venue $venue, ?\DateTimeImmutable $from = null, int $limit = 60): array
    {
        if (!$this->tableExists()) {
            return [];
        }

        try {
            return array_values($this->createQueryBuilder('e')
                ->andWhere('e.venue = :venue')
                ->andWhere('e.exceptionDate >= :from')
                ->setParameter('venue', $venue)
                ->setParameter('from', ($from ?? new \DateTimeImmutable('today'))->setTime(0, 0))
                ->orderBy('e.exceptionDate', 'ASC')
                ->addOrderBy('e.openTime', 'ASC')
                ->setMaxResults($limit)
                ->getQuery()->getResult());
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * Every exception in a window, for the surfaces that render more than one day.
     *
     * ⚠️ **Bounded, because a calendar asks for a window and not for history.**
     * `forDate()` answers one day and is what the resolver uses; a calendar
     * drawing a week would otherwise make seven queries, and a kiosk left running
     * would make them for ever.
     *
     * @return list<ScheduleException>
     */
    public function betweenFor(Venue $venue, \DateTimeImmutable $from, \DateTimeImmutable $to): array
    {
        if (!$this->tableExists()) {
            return [];
        }

        try {
            return array_values($this->createQueryBuilder('e')
                ->andWhere('e.venue = :venue')
                ->andWhere('e.exceptionDate >= :from')
                ->andWhere('e.exceptionDate <= :to')
                ->setParameter('venue', $venue)
                ->setParameter('from', $from->setTime(0, 0))
                ->setParameter('to', $to->setTime(0, 0))
                ->orderBy('e.exceptionDate', 'ASC')
                ->addOrderBy('e.openTime', 'ASC')
                ->getQuery()->getResult());
        } catch (\Throwable) {
            return [];
        }
    }

    public function tableExists(): bool
    {
        if ($this->tableExists === null) {
            try {
                $this->createQueryBuilder('e')->select('COUNT(e.id)')->getQuery()->getSingleScalarResult();
                $this->tableExists = true;
            } catch (\Throwable) {
                $this->tableExists = false;
            }
        }

        return $this->tableExists;
    }
}
