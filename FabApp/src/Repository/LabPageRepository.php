<?php

namespace App\Repository;

use App\Entity\LabPage;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<LabPage>
 */
class LabPageRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, LabPage::class);
    }

    /** @return LabPage[] */
    public function findTopLevel(): array
    {
        return $this->createQueryBuilder('page')
            ->andWhere('page.parentPage IS NULL')
            ->orderBy('page.position', 'ASC')
            ->addOrderBy('page.titre', 'ASC')
            ->getQuery()
            ->getResult();
    }

    /**
     * Top-level pages with their children eagerly hydrated in a single query,
     * for rendering the Lab hierarchy in the nav bar without an N+1.
     *
     * @return LabPage[]
     */
    public function findTopLevelWithChildren(): array
    {
        return $this->createQueryBuilder('page')
            ->leftJoin('page.children', 'child')
            ->addSelect('child')
            ->andWhere('page.parentPage IS NULL')
            ->orderBy('page.position', 'ASC')
            ->addOrderBy('page.titre', 'ASC')
            ->addOrderBy('child.position', 'ASC')
            ->addOrderBy('child.titre', 'ASC')
            ->getQuery()
            ->getResult();
    }

    /**
     * ⚠️ **S147, J-2 — la variante que voient les visiteurs.** `findTopLevel()` et
     * `findTopLevelWithChildren()` restent la question de l'admin : la liste
     * d'administration doit montrer les pages archivées, sinon on ne peut plus les
     * restaurer. Le menu public, le catalogue et l'extension Twig lisent celles-ci.
     *
     * ⚠️ Le champ est `parentPage`, pas `parent` — premier essai raté. Et la variante
     * « with children » garde sa jointure et ses tris sur les enfants : sans eux ce
     * n'est plus la même requête et le menu perd sa hiérarchie. Écrites à l'identique
     * de leurs jumelles au-dessus, avec une condition de plus.
     *
     * @return LabPage[]
     */
    public function findTopLevelLive(): array
    {
        return $this->createQueryBuilder('page')
            ->andWhere('page.parentPage IS NULL')
            ->andWhere('page.archivedAt IS NULL')
            ->orderBy('page.position', 'ASC')
            ->addOrderBy('page.titre', 'ASC')
            ->getQuery()
            ->getResult();
    }

    /** @return LabPage[] */
    public function findTopLevelWithChildrenLive(): array
    {
        return $this->createQueryBuilder('page')
            ->leftJoin('page.children', 'child', 'WITH', 'child.archivedAt IS NULL')
            ->addSelect('child')
            ->andWhere('page.parentPage IS NULL')
            ->andWhere('page.archivedAt IS NULL')
            ->orderBy('page.position', 'ASC')
            ->addOrderBy('page.titre', 'ASC')
            ->addOrderBy('child.position', 'ASC')
            ->addOrderBy('child.titre', 'ASC')
            ->getQuery()
            ->getResult();
    }
}
