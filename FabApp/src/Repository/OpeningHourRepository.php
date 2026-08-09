<?php

namespace App\Repository;

use App\Entity\OpeningHour;
use App\Entity\Venue;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<OpeningHour>
 */
class OpeningHourRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, OpeningHour::class);
    }

    /** @return OpeningHour[] */
    public function findOrdered(Venue $venue): array
    {
        return $this->findBy(['venue' => $venue], ['sortOrder' => 'ASC', 'dayOfWeek' => 'ASC']);
    }

    public function findOneForDate(\DateTimeInterface $dateTime): ?OpeningHour
    {
        return $this->findOneBy(['dayOfWeek' => (int) $dateTime->format('N')]);
    }
}
