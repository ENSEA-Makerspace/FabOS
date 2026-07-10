<?php

namespace App\Repository;

use App\Entity\HomepageUserPreference;
use App\Entity\Utilisateur;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<HomepageUserPreference>
 */
class HomepageUserPreferenceRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, HomepageUserPreference::class);
    }

    public function findOneForUser(Utilisateur $user): ?HomepageUserPreference
    {
        return $this->findOneBy(['user' => $user]);
    }
}
