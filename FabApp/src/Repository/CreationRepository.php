<?php

namespace App\Repository;

use App\Entity\Creation;
use App\Entity\CreationVote;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Creation>
 */
class CreationRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Creation::class);
    }

    /** @return Creation[] */
    public function findPublishedForShowcase(int $limit = 12): array
    {
        return $this->createQueryBuilder('creation')
            ->leftJoin('creation.author', 'author')
            ->addSelect('author')
            ->andWhere('creation.isPublished = :published')
            ->setParameter('published', true)
            ->orderBy('creation.createdAt', 'DESC')
            ->setMaxResults($limit)
            ->getQuery()
            ->getResult();
    }

    /** @return Creation[] */
    public function findPublishedForGallery(string $sort = 'recent'): array
    {
        $qb = $this->createQueryBuilder('creation')
            ->leftJoin('creation.author', 'author')
            ->addSelect('author')
            ->andWhere('creation.isPublished = :published')
            ->setParameter('published', true);

        if ($sort === 'rating') {
            $qb
                ->leftJoin(CreationVote::class, 'galleryVote', 'WITH', 'galleryVote.creation = creation')
                ->addSelect('AVG(galleryVote.rating) AS HIDDEN averageRating')
                ->addSelect('COUNT(galleryVote.id) AS HIDDEN voteCount')
                ->groupBy('creation.id')
                ->orderBy('averageRating', 'DESC')
                ->addOrderBy('voteCount', 'DESC')
                ->addOrderBy('creation.createdAt', 'DESC');
        } else {
            $qb->orderBy('creation.createdAt', 'DESC');
        }

        return $qb->getQuery()->getResult();
    }

    /** @return Creation[] */
    public function findPublishedRanking(): array
    {
        return $this->createQueryBuilder('creation')
            ->leftJoin('creation.author', 'author')
            ->addSelect('author')
            ->leftJoin(CreationVote::class, 'rankingVote', 'WITH', 'rankingVote.creation = creation')
            ->addSelect('AVG(rankingVote.rating) AS HIDDEN averageRating')
            ->addSelect('COUNT(rankingVote.id) AS HIDDEN voteCount')
            ->andWhere('creation.isPublished = :published')
            ->setParameter('published', true)
            ->groupBy('creation.id')
            ->orderBy('averageRating', 'DESC')
            ->addOrderBy('voteCount', 'DESC')
            ->addOrderBy('creation.createdAt', 'DESC')
            ->getQuery()
            ->getResult();
    }

    /** @return Creation[] */
    public function findTopRatedPublished(int $limit = 3): array
    {
        return $this->createQueryBuilder('creation')
            ->leftJoin('creation.author', 'author')
            ->addSelect('author')
            ->leftJoin(CreationVote::class, 'topVote', 'WITH', 'topVote.creation = creation')
            ->addSelect('AVG(topVote.rating) AS HIDDEN averageRating')
            ->addSelect('COUNT(topVote.id) AS HIDDEN voteCount')
            ->andWhere('creation.isPublished = :published')
            ->setParameter('published', true)
            ->groupBy('creation.id')
            ->having('COUNT(topVote.id) > 0')
            ->orderBy('averageRating', 'DESC')
            ->addOrderBy('voteCount', 'DESC')
            ->addOrderBy('creation.createdAt', 'DESC')
            ->setMaxResults($limit)
            ->getQuery()
            ->getResult();
    }

    /** @return Creation[] */
    public function findAllForModeration(): array
    {
        return $this->createQueryBuilder('creation')
            ->leftJoin('creation.author', 'author')
            ->addSelect('author')
            ->orderBy('creation.createdAt', 'DESC')
            ->getQuery()
            ->getResult();
    }
}
