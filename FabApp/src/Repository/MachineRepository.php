<?php

namespace App\Repository;

use App\Entity\Machine;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Machine>
 */
class MachineRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Machine::class);
    }

    public function findOneByMachineToken(string $machineToken): ?Machine
    {
        $machineToken = trim($machineToken);
        if ($machineToken === '') {
            return null;
        }

        return $this->findOneBy(['machineToken' => $machineToken]);
    }

    /** @return Machine[] */
    public function findForAdminFilters(array $filters): array
    {
        $qb = $this->createQueryBuilder('machine')
            ->leftJoin('machine.machineBadges', 'machineBadge')
            ->leftJoin('machineBadge.badge', 'badge')
            ->addSelect('machineBadge', 'badge')
            ->orderBy('machine.createdAt', 'DESC');

        $q = trim((string) ($filters['q'] ?? ''));
        if ($q !== '') {
            $qb
                ->andWhere('LOWER(machine.nom) LIKE :q OR LOWER(machine.description) LIKE :q OR LOWER(machine.localisation) LIKE :q OR LOWER(machine.machineToken) LIKE :q OR LOWER(machine.categoryLabel) LIKE :q OR LOWER(machine.categorySlug) LIKE :q')
                ->setParameter('q', '%' . self::escapeLike(mb_strtolower($q)) . '%');
        }

        $statut = trim((string) ($filters['statut'] ?? ''));
        if ($statut !== '' && $statut !== 'all') {
            $qb
                ->andWhere('machine.statut = :statut')
                ->setParameter('statut', $statut);
        }

        $niveau = trim((string) ($filters['niveau'] ?? ''));
        if ($niveau !== '' && $niveau !== 'all') {
            $qb
                ->andWhere('machine.levelSlug = :niveauSlug OR machine.levelLabel = :niveauLabel')
                ->setParameter('niveauSlug', 'niveau-' . $niveau)
                ->setParameter('niveauLabel', 'Niveau ' . $niveau);
        }

        $badge = trim((string) ($filters['badge'] ?? ''));
        if ($badge !== '' && $badge !== 'all' && ctype_digit($badge)) {
            $qb
                ->andWhere('badge.id = :badgeId')
                ->setParameter('badgeId', (int) $badge);
        }

        return $qb->getQuery()->getResult();
    }

    private static function escapeLike(string $value): string
    {
        return addcslashes($value, '%_');
    }
}
