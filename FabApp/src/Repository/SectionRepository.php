<?php

namespace App\Repository;

use App\Entity\Formation;
use App\Entity\Section;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Section>
 */
class SectionRepository extends ServiceEntityRepository
{
    public const PAGE_BLOCK_PREFIX = '__FABOS_PAGE__:';

    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Section::class);
    }

    /** @return Section[] */
    public function findJourneySections(Formation $formation): array
    {
        return $this->createQueryBuilder('section')
            ->andWhere('section.formation = :formation')
            ->andWhere('section.titre NOT LIKE :pagePrefix')
            ->setParameter('formation', $formation)
            ->setParameter('pagePrefix', self::PAGE_BLOCK_PREFIX . '%')
            ->orderBy('section.ordre', 'ASC')
            ->addOrderBy('section.id', 'ASC')
            ->getQuery()
            ->getResult();
    }

    /** @return Section[] */
    public function findPageContentBlocks(Formation $formation): array
    {
        return $this->createQueryBuilder('section')
            ->andWhere('section.formation = :formation')
            ->andWhere('section.titre LIKE :pagePrefix')
            ->setParameter('formation', $formation)
            ->setParameter('pagePrefix', self::PAGE_BLOCK_PREFIX . '%')
            ->orderBy('section.ordre', 'ASC')
            ->addOrderBy('section.id', 'ASC')
            ->getQuery()
            ->getResult();
    }

    public function findPageContentBlock(Formation $formation, string $key): ?Section
    {
        return $this->findOneBy([
            'formation' => $formation,
            'titre' => self::PAGE_BLOCK_PREFIX . $key,
        ]);
    }

    public function getNextJourneyOrder(Formation $formation): int
    {
        $value = $this->createQueryBuilder('section')
            ->select('MAX(section.ordre)')
            ->andWhere('section.formation = :formation')
            ->andWhere('section.titre NOT LIKE :pagePrefix')
            ->setParameter('formation', $formation)
            ->setParameter('pagePrefix', self::PAGE_BLOCK_PREFIX . '%')
            ->getQuery()
            ->getSingleScalarResult();

        return max(1, ((int) $value) + 1);
    }

    public static function isPageContentBlock(Section $section): bool
    {
        return str_starts_with($section->getTitre(), self::PAGE_BLOCK_PREFIX);
    }
}
