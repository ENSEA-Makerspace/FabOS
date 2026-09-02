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

    /**
     * Active users holding the staff role, for the Staff directory. The
     * person-type is stored as ROLE membership (no dedicated column), matching
     * the roadmap's role/enum framing. Fail-safe: returns [] on any query error.
     *
     * @return Utilisateur[]
     */
    public function findStaff(): array
    {
        return $this->findByRoleName('staff');
    }

    /**
     * Active users holding the trainer role, for the Trainers directory.
     *
     * @return Utilisateur[]
     */
    public function findTrainers(): array
    {
        return $this->findByRoleName('trainer');
    }

    /**
     * Les comptes actifs qui tiennent ce rôle — **par leur GROUPE** (S159f).
     *
     * 🔴 La requête joignait `UTILISATEUR_ROLE`, qui n'existe plus. Elle joint
     * désormais l'appartenance, qui est la source des rôles depuis la fusion.
     *
     * ⚠️ **`trainers` la clé, `trainer` le nom demandé** : la même différence de
     * nommage qu'ailleurs, traduite ici plutôt que laissée aux appelants —
     * `findTrainers()` demande « trainer » et doit continuer de le faire.
     *
     * ⚠️ Le `catch` reste : une installation sans la migration S133b n'a pas
     * `USER_GROUP`, et un annuaire vide vaut mieux qu'une page en erreur.
     *
     * @return Utilisateur[]
     */
    private function findByRoleName(string $roleName): array
    {
        $groupKey = strtolower($roleName) === 'trainer' ? 'trainers' : strtolower($roleName);

        try {
            return $this->createQueryBuilder('utilisateur')
                ->join('utilisateur.groupMemberships', 'membership')
                ->join('membership.group', 'userGroup')
                ->andWhere('userGroup.groupKey = :groupKey')
                ->andWhere("utilisateur.statut = 'actif'")
                ->setParameter('groupKey', $groupKey)
                ->orderBy('utilisateur.firstName', 'ASC')
                ->addOrderBy('utilisateur.lastName', 'ASC')
                ->getQuery()
                ->getResult();
        } catch (\Throwable) {
            return [];
        }
    }

    /** @return Utilisateur[] */
    public function findForAdminFilters(array $filters): array
    {
        // 🔴 **S159f — la jointure sur les rôles est partie avec la relation.**
        // `UTILISATEUR_ROLE` n'est plus lue : les rôles viennent des groupes, et
        // le filtre correspondant est le menu « Groupe », qui passe par
        // `AudienceResolver` parce que l'appartenance n'est pas une jointure.
        // ⚠️ Cette jointure préchargeait aussi les rôles pour la liste ; ce qui
        // les précharge désormais est `AudienceResolver::primeFor()`, appelé par
        // le contrôleur — une requête pour toute la page, comme avant.
        $qb = $this->createQueryBuilder('utilisateur')
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

        return $qb->getQuery()->getResult();
    }


    private static function escapeLike(string $value): string
    {
        return addcslashes($value, '%_');
    }
}
