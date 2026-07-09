<?php

namespace App\Repository;

use App\Entity\Progression;
use App\Entity\Utilisateur;
use App\Service\QuizCatalogService;
use App\Service\TrainingQualificationService;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Progression>
 */
class ProgressionRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Progression::class);
    }

    /** @return Progression[] */
    public function findVisibleByUser(Utilisateur $user): array
    {
        return $this->createQueryBuilder('progression')
            ->innerJoin('progression.formation', 'formation')
            ->addSelect('formation')
            ->andWhere('progression.utilisateur = :user')
            ->andWhere('formation.categorie IS NULL OR formation.categorie NOT IN (:internalCategories)')
            ->setParameter('user', $user)
            ->setParameter('internalCategories', [
                QuizCatalogService::INTERNAL_CATEGORY,
                TrainingQualificationService::PHYSICAL_CATEGORY,
            ])
            ->orderBy('progression.dateDebut', 'DESC')
            ->getQuery()
            ->getResult();
    }

    public function countCompletedVisible(?Utilisateur $user = null): int
    {
        $qb = $this->createQueryBuilder('progression')
            ->select('COUNT(progression.id)')
            ->innerJoin('progression.formation', 'formation')
            ->andWhere('progression.completed = :completed')
            ->andWhere('formation.categorie IS NULL OR formation.categorie NOT IN (:internalCategories)')
            ->setParameter('completed', true)
            ->setParameter('internalCategories', [
                QuizCatalogService::INTERNAL_CATEGORY,
                TrainingQualificationService::PHYSICAL_CATEGORY,
            ]);

        if ($user instanceof Utilisateur) {
            $qb
                ->andWhere('progression.utilisateur = :user')
                ->setParameter('user', $user);
        }

        return (int) $qb->getQuery()->getSingleScalarResult();
    }
}
