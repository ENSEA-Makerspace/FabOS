<?php

namespace App\Repository;

use App\Entity\Utilisateur;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Utilisateur>
 */
class UtilisateurRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, Utilisateur::class);
    }

    public function findOneByRfid(string $rfid): ?Utilisateur
    {
        $rfid = trim($rfid);
        if ($rfid === '') {
            return null;
        }

        return $this->findOneBy(['identifiantRfid' => $rfid]);
    }

    /** @return Utilisateur[] */
    public function findForAdminFilters(array $filters): array
    {
        $qb = $this->createQueryBuilder('utilisateur')
            ->leftJoin('utilisateur.utilisateurRoles', 'utilisateurRole')
            ->leftJoin('utilisateurRole.role', 'role')
            ->addSelect('utilisateurRole', 'role')
            ->orderBy('utilisateur.createdAt', 'DESC');

        $q = trim((string) ($filters['q'] ?? ''));
        if ($q !== '') {
            $qb
                ->andWhere('LOWER(utilisateur.firstName) LIKE :q OR LOWER(utilisateur.lastName) LIKE :q OR LOWER(utilisateur.email) LIKE :q OR LOWER(utilisateur.username) LIKE :q OR LOWER(utilisateur.identifiantRfid) LIKE :q')
                ->setParameter('q', '%' . self::escapeLike(mb_strtolower($q)) . '%');
        }

        $statut = trim((string) ($filters['statut'] ?? ''));
        if ($statut !== '' && $statut !== 'all') {
            $qb
                ->andWhere('utilisateur.statut = :statut')
                ->setParameter('statut', $statut);
        }

        $roleFilter = trim((string) ($filters['role'] ?? ''));
        if ($roleFilter !== '' && $roleFilter !== 'all') {
            $roleName = self::normalizeRoleFilter($roleFilter);
            $qb
                ->andWhere('LOWER(role.nom) = :roleName OR LOWER(role.nom) = :roleSecurityName')
                ->setParameter('roleName', $roleName)
                ->setParameter('roleSecurityName', 'role_' . $roleName);
        }

        return $qb->getQuery()->getResult();
    }

    private static function normalizeRoleFilter(string $role): string
    {
        $role = mb_strtolower(trim($role));

        return str_starts_with($role, 'role_') ? substr($role, 5) : $role;
    }

    private static function escapeLike(string $value): string
    {
        return addcslashes($value, '%_');
    }
}
