<?php

namespace App\Repository;

use App\Entity\UserAvailability;
use App\Entity\Utilisateur;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * Reads are fail-safe: the person's page and the directories must keep working
 * on an instance where the USER_AVAILABILITY migration hasn't run yet — no
 * windows just means no offered slots.
 *
 * @extends ServiceEntityRepository<UserAvailability>
 */
class UserAvailabilityRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, UserAvailability::class);
    }

    /** @return UserAvailability[] */
    public function findForUser(Utilisateur $user): array
    {
        try {
            return $this->createQueryBuilder('availability')
                ->andWhere('availability.utilisateur = :user')
                ->setParameter('user', $user)
                ->orderBy('availability.dayOfWeek', 'ASC')
                ->addOrderBy('availability.startTime', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /** @return array<int, UserAvailability[]> windows grouped by ISO weekday */
    public function findForUserByDay(Utilisateur $user): array
    {
        $byDay = [];
        foreach ($this->findForUser($user) as $availability) {
            $byDay[$availability->getDayOfWeek()][] = $availability;
        }

        return $byDay;
    }

    /** Replace a person's whole weekly schedule in one go — the edit form posts all rows. */
    public function replaceForUser(Utilisateur $user, array $windows): void
    {
        $em = $this->getEntityManager();

        foreach ($this->findForUser($user) as $existing) {
            $em->remove($existing);
        }
        $em->flush();

        foreach ($windows as $window) {
            $window->setUtilisateur($user);
            $em->persist($window);
        }
        $em->flush();
    }
}
