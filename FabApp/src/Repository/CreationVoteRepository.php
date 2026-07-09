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

    public function hasUserVoted(Creation $creation, Utilisateur $user): bool
    {
        return $this->findOneBy(['creation' => $creation, 'user' => $user]) !== null;
    }

    /** @param Creation[] $creations @return array<int, int> */
    public function countByCreation(array $creations): array
    {
        $creationIds = array_values(array_filter(array_map(
            static fn (Creation $creation): ?int => $creation->getId(),
            $creations
        )));

        if ($creationIds === []) {
            return [];
        }

        $qb = $this->createQueryBuilder('vote')
            ->select('IDENTITY(vote.creation) AS creationId, COUNT(vote.id) AS voteCount')
            ->andWhere('vote.creation IN (:creationIds)')
            ->setParameter('creationIds', $creationIds)
            ->groupBy('vote.creation');

        $counts = array_fill_keys($creationIds, 0);
        foreach ($qb->getQuery()->getArrayResult() as $row) {
            $counts[(int) $row['creationId']] = (int) $row['voteCount'];
        }

        return $counts;
    }
}
