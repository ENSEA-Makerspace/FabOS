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
     * Calcule le temps de présence réel depuis LOG_UTILISATION.
     *
     * Une ligne ouverte (dateFin NULL) ne donne pas une durée fiable, donc elle vaut 0.
     * Si duree est renseignée et positive, elle est utilisée en priorité. Sinon la durée
     * est calculée en minutes entre dateDebut et dateFin. Les logs RFID ne sont pas utilisés
     * ici : ils indiquent un passage, pas une présence mesurable.
     *
     * @param Utilisateur[] $users
     * @return array<int, int> minutes par id utilisateur
     */
    public function computePresenceMinutesByUser(array $users, ?\DateTimeImmutable $startAt = null, ?\DateTimeImmutable $endAt = null): array
    {
        $userIds = array_values(array_filter(array_map(
            static fn (Utilisateur $user): ?int => $user->getId(),
            $users
        )));

        if ($userIds === []) {
            return [];
        }

        $qb = $this->createQueryBuilder('log')
            ->leftJoin('log.utilisateur', 'utilisateur')
            ->addSelect('utilisateur')
            ->andWhere('log.utilisateur IN (:userIds)')
            ->setParameter('userIds', $userIds);

        $this->addDatePeriodFilter($qb, $startAt, $endAt);

        /** @var LogUtilisation[] $logs */
        $logs = $qb->getQuery()->getResult();
        $minutesByUser = array_fill_keys($userIds, 0);

        foreach ($logs as $log) {
            $userId = $log->getUtilisateur()?->getId();
            if ($userId === null) {
                continue;
            }

            $duration = $log->getDuree();
            if ($duration !== null && $duration > 0) {
                $minutesByUser[$userId] += $duration;
                continue;
            }

            $dateFin = $log->getDateFin();
            if ($dateFin === null || $dateFin <= $log->getDateDebut()) {
                continue;
            }

            $minutesByUser[$userId] += max(0, (int) floor(($dateFin->getTimestamp() - $log->getDateDebut()->getTimestamp()) / 60));
        }

        return $minutesByUser;
    }

    /** @param Utilisateur[] $users @return array<int, int> */
    public function count3dPrintsByUser(array $users, ?\DateTimeImmutable $startAt = null, ?\DateTimeImmutable $endAt = null): array
    {
        $userIds = array_values(array_filter(array_map(
            static fn (Utilisateur $user): ?int => $user->getId(),
            $users
        )));

        if ($userIds === []) {
            return [];
        }

        $qb = $this->createQueryBuilder('log')
            ->select('IDENTITY(log.utilisateur) AS userId, COUNT(log.id) AS printCount')
            ->join('log.machine', 'machine')
            ->andWhere('log.utilisateur IN (:userIds)')
            ->setParameter('userIds', $userIds)
            ->groupBy('log.utilisateur');

        $this->add3dPrinterMachineFilter($qb);
        $this->addDatePeriodFilter($qb, $startAt, $endAt);

        $counts = array_fill_keys($userIds, 0);
        foreach ($qb->getQuery()->getArrayResult() as $row) {
            $counts[(int) $row['userId']] = (int) $row['printCount'];
        }

        return $counts;
    }

    /** @param Utilisateur[] $users @return array<int, int> */
    public function countMachinesUsedByUser(array $users, ?\DateTimeImmutable $startAt = null, ?\DateTimeImmutable $endAt = null): array
    {
        $userIds = array_values(array_filter(array_map(
            static fn (Utilisateur $user): ?int => $user->getId(),
            $users
        )));

        if ($userIds === []) {
            return [];
        }

        $qb = $this->createQueryBuilder('log')
            ->select('IDENTITY(log.utilisateur) AS userId, COUNT(DISTINCT machine.id) AS machineCount')
            ->join('log.machine', 'machine')
            ->andWhere('log.utilisateur IN (:userIds)')
            ->setParameter('userIds', $userIds)
            ->groupBy('log.utilisateur');

        $this->addDatePeriodFilter($qb, $startAt, $endAt);

        $counts = array_fill_keys($userIds, 0);
        foreach ($qb->getQuery()->getArrayResult() as $row) {
            $counts[(int) $row['userId']] = (int) $row['machineCount'];
        }

        return $counts;
    }

    public function count3dPrintsForUser(Utilisateur $user, ?\DateTimeImmutable $startAt = null, ?\DateTimeImmutable $endAt = null): int
    {
        $qb = $this->createQueryBuilder('log')
            ->select('COUNT(log.id)')
            ->join('log.machine', 'machine')
            ->andWhere('log.utilisateur = :user')
            ->setParameter('user', $user);

        $this->add3dPrinterMachineFilter($qb);
        $this->addDatePeriodFilter($qb, $startAt, $endAt);

        return (int) $qb->getQuery()->getSingleScalarResult();
    }

    public function count3dPrints(?\DateTimeImmutable $startAt = null, ?\DateTimeImmutable $endAt = null): int
    {
        $qb = $this->createQueryBuilder('log')
            ->select('COUNT(log.id)')
            ->join('log.machine', 'machine');

        $this->add3dPrinterMachineFilter($qb);
        $this->addDatePeriodFilter($qb, $startAt, $endAt);

        return (int) $qb->getQuery()->getSingleScalarResult();
    }

    private function addDatePeriodFilter($qb, ?\DateTimeImmutable $startAt, ?\DateTimeImmutable $endAt): void
    {
        if ($startAt !== null) {
            $qb
                ->andWhere('log.dateDebut >= :leaderboardStartAt')
                ->setParameter('leaderboardStartAt', $startAt);
        }

        if ($endAt !== null) {
            $qb
                ->andWhere('log.dateDebut < :leaderboardEndAt')
                ->setParameter('leaderboardEndAt', $endAt);
        }
    }

    private function add3dPrinterMachineFilter($qb): void
    {
        $qb
            ->andWhere('LOWER(machine.nom) LIKE :printImprimante OR LOWER(machine.nom) LIKE :print3d OR LOWER(machine.nom) LIKE :printPrinter OR LOWER(machine.nom) LIKE :printPrusa OR LOWER(machine.nom) LIKE :printUltimaker OR LOWER(machine.machineToken) LIKE :printImprimante OR LOWER(machine.machineToken) LIKE :print3d OR LOWER(machine.machineToken) LIKE :printPrinter OR LOWER(machine.machineToken) LIKE :printPrusa OR LOWER(machine.machineToken) LIKE :printUltimaker')
            ->setParameter('printImprimante', '%imprimante%')
            ->setParameter('print3d', '%3d%')
            ->setParameter('printPrinter', '%printer%')
            ->setParameter('printPrusa', '%prusa%')
            ->setParameter('printUltimaker', '%ultimaker%');
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
