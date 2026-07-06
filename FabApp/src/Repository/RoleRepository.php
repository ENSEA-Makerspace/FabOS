<?php

namespace App\Repository;

use App\Entity\Role;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Role>
 */
class RoleRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Role::class);
    }

    public function findOneBySecurityRole(string $securityRole): ?Role
    {
        $role = mb_strtolower(trim($securityRole));
        if ($role === '') {
            return null;
        }

        $shortRole = str_starts_with($role, 'role_') ? substr($role, 5) : $role;

        return $this->createQueryBuilder('role')
            ->andWhere('LOWER(role.nom) = :shortRole OR LOWER(role.nom) = :securityRole')
            ->setParameter('shortRole', $shortRole)
            ->setParameter('securityRole', 'role_' . $shortRole)
            ->setMaxResults(1)
            ->getQuery()
            ->getOneOrNullResult();
    }
}
