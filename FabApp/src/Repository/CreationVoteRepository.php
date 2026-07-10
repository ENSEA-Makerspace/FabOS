<?php

namespace App\Repository;

use App\Entity\Creation;
use App\Entity\CreationVote;
use App\Entity\Utilisateur;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<CreationVote>
 */
class CreationVoteRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, CreationVote::class);
    }

    public function findUserRating(Creation $creation, Utilisateur $user): ?CreationVote
    {
        return $this->findOneBy(['creation' => $creation, 'user' => $user]);
    }

    public function hasUserVoted(Creation $creation, Utilisateur $user): bool
    {
        return $this->findUserRating($creation, $user) !== null;
    }

    /** @param Creation[] $creations @return array<int, int> */
    public function countByCreation(array $creations): array
    {
        $stats = $this->getStatsByCreation($creations);
        $counts = [];

        foreach ($stats as $creationId => $row) {
            $counts[$creationId] = $row['count'];
        }

        return $counts;
    }

    /** @param Creation[] $creations @return array<int, array{average: float|null, count: int}> */
    public function getStatsByCreation(array $creations): array
    {
        $creationIds = $this->extractCreationIds($creations);

        if ($creationIds === []) {
            return [];
        }

        $stats = [];
        foreach ($creationIds as $creationId) {
            $stats[$creationId] = ['average' => null, 'count' => 0];
        }

        $rows = $this->createQueryBuilder('vote')
            ->select('IDENTITY(vote.creation) AS creationId, AVG(vote.rating) AS averageRating, COUNT(vote.id) AS ratingCount')
            ->andWhere('vote.creation IN (:creationIds)')
            ->setParameter('creationIds', $creationIds)
            ->groupBy('vote.creation')
            ->getQuery()
            ->getArrayResult();

        foreach ($rows as $row) {
            $creationId = (int) $row['creationId'];
            $stats[$creationId] = [
                'average' => $row['averageRating'] !== null ? round((float) $row['averageRating'], 1) : null,
                'count' => (int) $row['ratingCount'],
            ];
        }

        return $stats;
    }

    /** @param Creation[] $creations @return array<int, float> */
    public function getUserRatingsByCreation(array $creations, Utilisateur $user): array
    {
        $creationIds = $this->extractCreationIds($creations);

        if ($creationIds === []) {
            return [];
        }

        $rows = $this->createQueryBuilder('vote')
            ->select('IDENTITY(vote.creation) AS creationId, vote.rating AS rating')
            ->andWhere('vote.creation IN (:creationIds)')
            ->andWhere('vote.user = :user')
            ->setParameter('creationIds', $creationIds)
            ->setParameter('user', $user)
            ->getQuery()
            ->getArrayResult();

        $ratings = [];
        foreach ($rows as $row) {
            $ratings[(int) $row['creationId']] = (float) $row['rating'];
        }

        return $ratings;
    }

    /** @param Creation[] $creations @return int[] */
    private function extractCreationIds(array $creations): array
    {
        return array_values(array_filter(array_map(
            static fn (Creation $creation): ?int => $creation->getId(),
            $creations
        )));
    }
}
