<?php

namespace App\Repository;

use App\Entity\EventCategory;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<EventCategory>
 */
class EventCategoryRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, EventCategory::class);
    }

    /**
     * The categories an operator may still choose, in their own order.
     *
     * ⚠️ Archived rows are excluded here and NOWHERE else: an event that already
     * carries an archived category keeps displaying it. Filtering them out of the
     * display too would make an event's kind vanish from the page the day somebody
     * tidied the list.
     *
     * @return EventCategory[]
     */
    public function findSelectable(): array
    {
        return $this->createQueryBuilder('c')
            ->andWhere('c.archivedAt IS NULL')
            ->orderBy('c.position', 'ASC')
            ->addOrderBy('c.label', 'ASC')
            ->getQuery()
            ->getResult();
    }

    /** @return EventCategory[] */
    public function findAllOrdered(): array
    {
        return $this->createQueryBuilder('c')
            ->orderBy('c.archivedAt', 'ASC')
            ->addOrderBy('c.position', 'ASC')
            ->addOrderBy('c.label', 'ASC')
            ->getQuery()
            ->getResult();
    }

    public function findOneBySlug(string $slug): ?EventCategory
    {
        return $this->findOneBy(['slug' => $slug]);
    }

    /**
     * How many events carry each category, keyed by category id — for the admin
     * list, so archiving says what it affects instead of asking blind.
     *
     * @return array<int, int>
     */
    public function countEventsByCategory(): array
    {
        $rows = $this->getEntityManager()->createQuery(
            'SELECT IDENTITY(e.category) AS categoryId, COUNT(e.id) AS total
             FROM App\Entity\Event e
             WHERE e.category IS NOT NULL
             GROUP BY e.category',
        )->getArrayResult();

        $counts = [];
        foreach ($rows as $row) {
            $counts[(int) $row['categoryId']] = (int) $row['total'];
        }

        return $counts;
    }
}
