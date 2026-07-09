<?php

namespace App\Repository;

use App\Entity\Creation;
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
    public function findPublishedForGallery(): array
    {
        return $this->createQueryBuilder('creation')
            ->leftJoin('creation.author', 'author')
            ->addSelect('author')
            ->andWhere('creation.isPublished = :published')
            ->setParameter('published', true)
            ->orderBy('creation.createdAt', 'DESC')
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
