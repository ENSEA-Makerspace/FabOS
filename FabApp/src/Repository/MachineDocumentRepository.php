<?php

namespace App\Repository;

use App\Entity\Machine;
use App\Entity\MachineDocument;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<MachineDocument>
 */
class MachineDocumentRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, MachineDocument::class);
    }

    /**
     * Les documents d'une machine, dans l'ordre voulu.
     *
     * ⚠️ `position` puis `id` : deux documents ajoutés sans qu'on touche à
     * l'ordre partagent la position 0, et il faut alors que le plus ancien
     * passe en premier plutôt qu'un ordre indéfini que la base choisit seule.
     *
     * @return list<MachineDocument>
     */
    public function forMachine(Machine $machine): array
    {
        return $this->createQueryBuilder('d')
            ->andWhere('d.machine = :machine')
            ->setParameter('machine', $machine)
            ->orderBy('d.position', 'ASC')
            ->addOrderBy('d.id', 'ASC')
            ->getQuery()
            ->getResult();
    }

    /**
     * La position à donner au prochain document de cette machine.
     *
     * ⚠️ Calculée plutôt que comptée : compter les lignes redonne une position
     * déjà prise dès qu'un document a été supprimé au milieu.
     */
    public function nextPosition(Machine $machine): int
    {
        $max = $this->createQueryBuilder('d')
            ->select('MAX(d.position)')
            ->andWhere('d.machine = :machine')
            ->setParameter('machine', $machine)
            ->getQuery()
            ->getSingleScalarResult();

        return $max === null ? 0 : ((int) $max) + 1;
    }
}
