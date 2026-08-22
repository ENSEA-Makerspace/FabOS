<?php

namespace App\Repository;

use App\Entity\Material;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Material>
 */
class MaterialRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Material::class);
    }

    /**
     * All materials for the public catalogue, grouped-friendly (category then
     * name). Fail-safe: returns [] if the MATERIAL table doesn't exist yet
     * (deployed ahead of its migration) — mirrors SiteFeatureService's pattern.
     *
     * @return Material[]
     */
    public function findAllSafe(): array
    {
        try {
            return $this->createQueryBuilder('material')
                ->orderBy('material.category', 'ASC')
                ->addOrderBy('material.name', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * Materials that the given machine accepts (compatibility link). Fail-safe:
     * returns [] if the MATERIAL / MACHINE_MATERIAL tables don't exist yet, so
     * the public machine page never breaks when deployed ahead of the migration.
     *
     * @return Material[]
     */
    public function findByMachine(int $machineId): array
    {
        try {
            return $this->createQueryBuilder('material')
                ->andWhere('material.archivedAt IS NULL')
                ->join('material.machines', 'machine')
                ->andWhere('machine.id = :machineId')
                ->setParameter('machineId', $machineId)
                ->orderBy('material.category', 'ASC')
                ->addOrderBy('material.name', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * ⚠️ **S147, J-2 — la question du MEMBRE, pas celle de l'admin.**
     * `findAllSafe()` reste au-dessus et montre tout : c'est de la liste
     * d'administration qu'on restaure un matériau archivé. Cette variante-ci
     * alimente les surfaces qui PROPOSENT — le catalogue public et la recherche —
     * et un matériau archivé n'y a rien à faire. Même partage que
     * `MachineRepository::findLive()`.
     *
     * @return Material[]
     */
    public function findLiveSafe(): array
    {
        try {
            return $this->createQueryBuilder('material')
                ->andWhere('material.archivedAt IS NULL')
                ->orderBy('material.category', 'ASC')
                ->addOrderBy('material.name', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }
}
