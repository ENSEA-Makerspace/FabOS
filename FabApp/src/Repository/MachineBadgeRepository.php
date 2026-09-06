<?php

namespace App\Repository;

use App\Entity\Machine;
use App\Entity\MachineBadge;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<MachineBadge>
 */
class MachineBadgeRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, MachineBadge::class);
    }

    /**
     * @return MachineBadge[]
     */
    public function findRequiredForMachine(Machine $machine): array
    {
        return $this->findBy(['machine' => $machine, 'requiredForAccess' => true], ['id' => 'ASC']);
    }

    /**
     * Les machines qu'un badge OUVRE (S179).
     *
     * 🔴 **La question qu'un apprenant se pose en premier et que le produit ne
     * savait pas poser.** La relation existait — c'est elle qui décide de
     * l'accès à chaque scan — mais elle n'était lue que dans le sens
     * machine → badges. Une fiche de formation pouvait donc dire « vous
     * obtiendrez le badge Découpe laser » sans jamais dire **ce que ce badge
     * ouvre**, c'est-à-dire la seule raison de le vouloir.
     *
     * ⚠️ Les machines ARCHIVÉES sortent : promettre un accès à une machine qui a
     * quitté le labo, c'est vendre une porte qui n'existe plus.
     *
     * @return Machine[]
     */
    public function machinesOpenedBy(int $badgeId): array
    {
        /*
         * ⚠️ **`Machine` est la RACINE, `MachineBadge` passe en `EXISTS`.** Le
         * premier jet partait de `MachineBadge` et sélectionnait l'alias joint :
         * DQL refuse — « Cannot select entity through identification variables
         * without choosing at least one root entity alias ». On ne sélectionne
         * pas un alias joint tout seul.
         * ✅ Attrapé par `app:render` AVANT le redémarrage. Ni `lint:twig` ni
         * `lint:container` ne lisent le DQL : seule l'exécution le fait, et
         * c'est exactement pourquoi le rituel rend les pages avant de relancer.
         * ✅ Et l'`EXISTS` évite le doublon qu'une jointure produirait si deux
         * lignes liaient le même badge à la même machine.
         */
        return $this->getEntityManager()->createQuery(
            'SELECT m FROM App\\Entity\\Machine m
              WHERE m.archivedAt IS NULL
                AND EXISTS (
                    SELECT mb.id FROM App\\Entity\\MachineBadge mb
                     WHERE mb.machine = m
                       AND IDENTITY(mb.badge) = :badgeId
                       AND mb.requiredForAccess = true
                )
              ORDER BY m.nom ASC'
        )->setParameter('badgeId', $badgeId)->getResult();
    }
}
