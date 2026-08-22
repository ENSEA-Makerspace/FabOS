<?php

namespace App\Repository;

use App\Entity\Machine;
use App\Entity\Venue;
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
    /**
     * The machines still in the lab (S134b).
     *
     * 🔴 **`findBy()` is the ADMIN's question, this is everybody else's.** A machine
     * that left the lab is archived, never deleted, because `RESERVATION`,
     * `LOG_UTILISATION` and `ACCESS_RFID_LOG` point at it. So every surface that
     * OFFERS a machine — the catalogue, the calendar, the home deck, the API — must
     * ask this, and every surface that NAMES one already booked must not: a past
     * reservation whose machine vanished from the page is a hole in somebody's
     * history, not tidiness.
     *
     * ⚠️ The admin list deliberately keeps `findForAdminFilters()`: archiving must be
     * visible and reversible from the screen that did it.
     *
     * @param array<string, mixed> $criteria
     * @param array<string, string> $orderBy
     *
     * @return Machine[]
     */
    public function findLive(array $criteria = [], array $orderBy = ['nom' => 'ASC'], ?int $limit = null): array
    {
        $qb = $this->createQueryBuilder('m')->andWhere('m.archivedAt IS NULL');

        foreach ($criteria as $field => $value) {
            $qb->andWhere(sprintf('m.%s = :%s', $field, $field))->setParameter($field, $value);
        }

        foreach ($orderBy as $field => $direction) {
            $qb->addOrderBy('m.' . $field, strtoupper($direction) === 'DESC' ? 'DESC' : 'ASC');
        }

        if ($limit !== null) {
            $qb->setMaxResults($limit);
        }

        return $qb->getQuery()->getResult();
    }

    public function findForAdminFilters(array $filters, ?Venue $venue = null): array
    {
        $qb = $this->createQueryBuilder('machine')
            ->leftJoin('machine.machineBadges', 'machineBadge')
            ->leftJoin('machineBadge.badge', 'badge')
            ->addSelect('machineBadge', 'badge')
            ->orderBy('machine.createdAt', 'DESC');

        if ($venue !== null) {
            $qb->andWhere('machine.venue = :venue')->setParameter('venue', $venue);
        }

        $q = trim((string) ($filters['q'] ?? ''));
        if ($q !== '') {
            $qb
                ->andWhere('LOWER(machine.nom) LIKE :q OR LOWER(machine.description) LIKE :q OR LOWER(machine.localisation) LIKE :q OR LOWER(machine.machineToken) LIKE :q OR LOWER(machine.categoryLabel) LIKE :q OR LOWER(machine.categorySlug) LIKE :q')
                ->setParameter('q', '%' . self::escapeLike(mb_strtolower($q)) . '%');
        }

        $statut = trim((string) ($filters['statut'] ?? ''));
        if ($statut !== '' && $statut !== 'all') {
            // See Machine::statusFilterForKey — a display key matches every stored
            // word that maps to it; a raw value still matches exactly, so links and
            // bookmarks made before S134c keep working.
            $byKey = Machine::statusFilterForKey($statut);
            if ($byKey === null) {
                $qb
                    ->andWhere('machine.statut = :statut')
                    ->setParameter('statut', $statut);
            } elseif (isset($byKey['in'])) {
                $qb
                    ->andWhere('LOWER(machine.statut) IN (:statuts)')
                    ->setParameter('statuts', $byKey['in']);
            } else {
                $qb
                    ->andWhere('LOWER(machine.statut) NOT IN (:statuts)')
                    ->setParameter('statuts', $byKey['notIn']);
            }
        }

        $category = trim((string) ($filters['category'] ?? ''));
        if ($category !== '' && $category !== 'all') {
            $qb
                ->andWhere('machine.categorySlug = :category')
                ->setParameter('category', $category);
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
