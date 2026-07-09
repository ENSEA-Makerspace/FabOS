<?php

namespace App\Repository;

use App\Entity\Badge;
use App\Entity\Formation;
use App\Service\QuizCatalogService;
use App\Service\TrainingQualificationService;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Formation>
 */
class FormationRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Formation::class);
    }

    public function findVisibleByBadge(Badge $badge): ?Formation
    {
        return $this->createQueryBuilder('formation')
            ->andWhere('formation.badge = :badge')
            ->andWhere('formation.categorie IS NULL OR formation.categorie NOT IN (:internalCategories)')
            ->setParameter('badge', $badge)
            ->setParameter('internalCategories', [
                QuizCatalogService::INTERNAL_CATEGORY,
                TrainingQualificationService::PHYSICAL_CATEGORY,
            ])
            ->orderBy('formation.id', 'ASC')
            ->setMaxResults(1)
            ->getQuery()
            ->getOneOrNullResult();
    }

    public function findOneByNormalizedTitle(string $title): ?Formation
    {
        $normalizedTitle = mb_strtolower(trim($title));
        if ($normalizedTitle === '') {
            return null;
        }

        return $this->createQueryBuilder('formation')
            ->andWhere('LOWER(formation.titre) = :title')
            ->setParameter('title', $normalizedTitle)
            ->setMaxResults(1)
            ->getQuery()
            ->getOneOrNullResult();
    }

    /** @return Formation[] */
    public function findVisible(array $orderBy = ['id' => 'DESC'], ?int $limit = null): array
    {
        $qb = $this->createQueryBuilder('formation')
            ->andWhere('formation.categorie IS NULL OR formation.categorie NOT IN (:internalCategories)')
            ->setParameter('internalCategories', [
                QuizCatalogService::INTERNAL_CATEGORY,
                TrainingQualificationService::PHYSICAL_CATEGORY,
            ]);

        foreach ($orderBy as $field => $direction) {
            $qb->addOrderBy('formation.' . $field, strtoupper((string) $direction) === 'ASC' ? 'ASC' : 'DESC');
        }

        if ($limit !== null) {
            $qb->setMaxResults($limit);
        }

        return $qb->getQuery()->getResult();
    }

    public function countVisible(): int
    {
        return (int) $this->createQueryBuilder('formation')
            ->select('COUNT(formation.id)')
            ->andWhere('formation.categorie IS NULL OR formation.categorie NOT IN (:internalCategories)')
            ->setParameter('internalCategories', [
                QuizCatalogService::INTERNAL_CATEGORY,
                TrainingQualificationService::PHYSICAL_CATEGORY,
            ])
            ->getQuery()
            ->getSingleScalarResult();
    }

    /** @return Formation[] */
    public function findQuizFormationsForParent(int $parentFormationId): array
    {
        return $this->createQueryBuilder('formation')
            ->andWhere('formation.categorie = :internalCategory')
            ->andWhere('formation.prerequis LIKE :marker')
            ->setParameter('internalCategory', QuizCatalogService::INTERNAL_CATEGORY)
            ->setParameter('marker', '%FABOS_PARENT_FORMATION_ID=' . $parentFormationId . ';%')
            ->orderBy('formation.id', 'ASC')
            ->getQuery()
            ->getResult();
    }

    public function findPhysicalValidationForParent(int $parentFormationId): ?Formation
    {
        return $this->createQueryBuilder('formation')
            ->andWhere('formation.categorie = :physicalCategory')
            ->andWhere('formation.prerequis LIKE :marker')
            ->setParameter('physicalCategory', TrainingQualificationService::PHYSICAL_CATEGORY)
            ->setParameter('marker', '%FABOS_PHYSICAL_PARENT_FORMATION_ID=' . $parentFormationId . ';%')
            ->setMaxResults(1)
            ->getQuery()
            ->getOneOrNullResult();
    }

    /** @return Formation[] */
    public function findQuizFormationsForMachine(int $machineId): array
    {
        return $this->createQueryBuilder('formation')
            ->andWhere('formation.categorie = :internalCategory')
            ->andWhere('formation.prerequis LIKE :marker')
            ->setParameter('internalCategory', QuizCatalogService::INTERNAL_CATEGORY)
            ->setParameter('marker', '%FABOS_MACHINE_ID=' . $machineId . ';%')
            ->orderBy('formation.id', 'ASC')
            ->getQuery()
            ->getResult();
    }

    /** @return Formation[] */
    public function findForAdminFilters(array $filters): array
    {
        $qb = $this->createQueryBuilder('formation')
            ->leftJoin('formation.badge', 'badge')
            ->addSelect('badge')
            ->andWhere('formation.categorie IS NULL OR formation.categorie NOT IN (:internalCategories)')
            ->setParameter('internalCategories', [
                QuizCatalogService::INTERNAL_CATEGORY,
                TrainingQualificationService::PHYSICAL_CATEGORY,
            ])
            ->orderBy('formation.id', 'DESC');

        $q = trim((string) ($filters['q'] ?? ''));
        if ($q !== '') {
            $qb
                ->andWhere('LOWER(formation.titre) LIKE :q OR LOWER(formation.description) LIKE :q OR LOWER(formation.categorie) LIKE :q OR LOWER(formation.formateur) LIKE :q')
                ->setParameter('q', '%' . self::escapeLike(mb_strtolower($q)) . '%');
        }

        $niveau = trim((string) ($filters['niveau'] ?? ''));
        if ($niveau !== '' && $niveau !== 'all' && ctype_digit($niveau)) {
            $qb
                ->andWhere('formation.niveau = :niveau')
                ->setParameter('niveau', (int) $niveau);
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
