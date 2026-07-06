<?php

namespace App\Repository;

use App\Entity\Machine;
use App\Entity\MachineFavorite;
use App\Entity\Utilisateur;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<MachineFavorite>
 */
class MachineFavoriteRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, MachineFavorite::class);
    }

    public function isStorageReady(): bool
    {
        try {
            return $this->getEntityManager()->getConnection()->createSchemaManager()->tablesExist(['MACHINE_FAVORITE']);
        } catch (\Throwable) {
            return false;
        }
    }

    public function findOneForUserAndMachine(Utilisateur $user, Machine $machine): ?MachineFavorite
    {
        return $this->findOneBy(['utilisateur' => $user, 'machine' => $machine]);
    }

    /** @return int[] */
    public function findMachineIdsForUser(Utilisateur $user): array
    {
        if (!$this->isStorageReady()) {
            return [];
        }

        $rows = $this->createQueryBuilder('favorite')
            ->select('IDENTITY(favorite.machine) AS machineId')
            ->andWhere('favorite.utilisateur = :user')
            ->setParameter('user', $user)
            ->orderBy('favorite.createdAt', 'DESC')
            ->getQuery()
            ->getArrayResult();

        return array_map(static fn (array $row): int => (int) $row['machineId'], $rows);
    }


    /** @return Machine[] */
    public function findMachinesForUser(Utilisateur $user, ?int $limit = null): array
    {
        if (!$this->isStorageReady()) {
            return [];
        }

        $qb = $this->createQueryBuilder('favorite')
            ->join('favorite.machine', 'machine')
            ->addSelect('machine')
            ->andWhere('favorite.utilisateur = :user')
            ->setParameter('user', $user)
            ->orderBy('favorite.createdAt', 'DESC');

        if ($limit !== null) {
            $qb->setMaxResults($limit);
        }

        return array_map(
            static fn (MachineFavorite $favorite): Machine => $favorite->getMachine(),
            array_filter($qb->getQuery()->getResult(), static fn (MachineFavorite $favorite): bool => $favorite->getMachine() instanceof Machine)
        );
    }

    /** @return array<int, array{id: int, token: string, name: string}> */
    public function findMachineSummariesForUser(Utilisateur $user): array
    {
        if (!$this->isStorageReady()) {
            return [];
        }

        $rows = $this->createQueryBuilder('favorite')
            ->join('favorite.machine', 'machine')
            ->select('machine.id AS id, machine.machineToken AS token, machine.nom AS name')
            ->andWhere('favorite.utilisateur = :user')
            ->setParameter('user', $user)
            ->orderBy('favorite.createdAt', 'DESC')
            ->getQuery()
            ->getArrayResult();

        return array_map(static fn (array $row): array => [
            'id' => (int) $row['id'],
            'token' => (string) $row['token'],
            'name' => (string) $row['name'],
        ], $rows);
    }
}
