<?php

namespace App\Repository;

use App\Entity\HomepageSectionVisibility;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<HomepageSectionVisibility>
 */
class HomepageSectionVisibilityRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, HomepageSectionVisibility::class);
    }

    /** @return HomepageSectionVisibility[] */
    public function findOrdered(): array
    {
        return $this->findBy([], ['sortOrder' => 'ASC', 'id' => 'ASC']);
    }

    public function findOneBySectionKey(string $sectionKey): ?HomepageSectionVisibility
    {
        return $this->findOneBy(['sectionKey' => $sectionKey]);
    }
}
