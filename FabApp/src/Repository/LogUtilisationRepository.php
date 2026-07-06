<?php

namespace App\Repository;

use App\Entity\LogUtilisation;
use App\Entity\Machine;
use App\Entity\Utilisateur;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<LogUtilisation>
 */
class LogUtilisationRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, LogUtilisation::class);
    }

    public function findOpenForMachine(Machine $machine): ?LogUtilisation
    {
        return $this->createQueryBuilder('log')
            ->andWhere('log.machine = :machine')
            ->andWhere('log.dateFin IS NULL')
            ->setParameter('machine', $machine)
            ->orderBy('log.dateDebut', 'DESC')
            ->setMaxResults(1)
            ->getQuery()
            ->getOneOrNullResult();
    }

    public function findOpenForUser(Utilisateur $user): ?LogUtilisation
    {
        return $this->createQueryBuilder('log')
            ->andWhere('log.utilisateur = :user')
            ->andWhere('log.dateFin IS NULL')
            ->setParameter('user', $user)
            ->orderBy('log.dateDebut', 'DESC')
            ->setMaxResults(1)
            ->getQuery()
            ->getOneOrNullResult();
    }

    public function findCurrentForMachineAndUser(Machine $machine, Utilisateur $user): ?LogUtilisation
    {
        return $this->createQueryBuilder('log')
            ->andWhere('log.machine = :machine')
            ->andWhere('log.utilisateur = :user')
            ->andWhere('log.dateFin IS NULL')
            ->setParameter('machine', $machine)
            ->setParameter('user', $user)
            ->orderBy('log.dateDebut', 'DESC')
            ->setMaxResults(1)
            ->getQuery()
            ->getOneOrNullResult();
    }

    /**
     * @param array{q?: string, state?: string, source?: string, dateFrom?: string, dateTo?: string} $filters
     * @return LogUtilisation[]
     */
    public function findAdminUsageLogs(array $filters): array
    {
        $qb = $this->createQueryBuilder('log')
            ->leftJoin('log.machine', 'machine')
            ->addSelect('machine')
            ->leftJoin('log.utilisateur', 'utilisateur')
            ->addSelect('utilisateur');

        $query = trim((string) ($filters['q'] ?? ''));
        if ($query !== '') {
            $qb
                ->andWhere('LOWER(machine.nom) LIKE :q OR LOWER(machine.machineToken) LIKE :q OR LOWER(utilisateur.firstName) LIKE :q OR LOWER(utilisateur.lastName) LIKE :q OR LOWER(utilisateur.username) LIKE :q OR LOWER(utilisateur.email) LIKE :q OR LOWER(log.source) LIKE :q')
                ->setParameter('q', '%' . mb_strtolower($query) . '%');
        }

        $state = (string) ($filters['state'] ?? 'all');
        if ($state === 'open') {
            $qb->andWhere('log.dateFin IS NULL');
        } elseif ($state === 'closed') {
            $qb->andWhere('log.dateFin IS NOT NULL');
        }

        $source = trim((string) ($filters['source'] ?? 'all'));
        if ($source !== '' && $source !== 'all') {
            $qb
                ->andWhere('LOWER(log.source) = :source')
                ->setParameter('source', mb_strtolower($source));
        }

        $dateFrom = $this->parseAdminDate((string) ($filters['dateFrom'] ?? ''), false);
        if ($dateFrom !== null) {
            $qb
                ->andWhere('log.dateDebut >= :dateFrom')
                ->setParameter('dateFrom', $dateFrom);
        }

        $dateTo = $this->parseAdminDate((string) ($filters['dateTo'] ?? ''), true);
        if ($dateTo !== null) {
            $qb
                ->andWhere('log.dateDebut <= :dateTo')
                ->setParameter('dateTo', $dateTo);
        }

        return $qb
            ->orderBy('log.dateDebut', 'DESC')
            ->getQuery()
            ->getResult();
    }

    /** @return string[] */
    public function findAdminUsageSources(): array
    {
        $rows = $this->createQueryBuilder('log')
            ->select('DISTINCT log.source AS source')
            ->andWhere('log.source IS NOT NULL')
            ->andWhere('log.source != :empty')
            ->setParameter('empty', '')
            ->orderBy('log.source', 'ASC')
            ->getQuery()
            ->getArrayResult();

        return array_values(array_filter(array_map(
            static fn (array $row): string => (string) $row['source'],
            $rows
        )));
    }

    private function parseAdminDate(string $value, bool $endOfDay): ?\DateTimeImmutable
    {
        $value = trim($value);
        if ($value === '') {
            return null;
        }

        $date = \DateTimeImmutable::createFromFormat('!Y-m-d', $value);
        if (!$date instanceof \DateTimeImmutable) {
            return null;
        }

        return $endOfDay ? $date->setTime(23, 59, 59) : $date->setTime(0, 0);
    }
}
