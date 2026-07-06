<?php

namespace App\Repository;

use App\Entity\Formation;
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
    public function findForAdminFilters(array $filters): array
    {
        $qb = $this->createQueryBuilder('formation')
            ->leftJoin('formation.badge', 'badge')
            ->addSelect('badge')
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
