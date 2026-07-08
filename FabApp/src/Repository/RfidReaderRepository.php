<?php

namespace App\Repository;

use App\Entity\RfidReader;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<RfidReader>
 */
class RfidReaderRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry)
    {
        parent::__construct($registry, RfidReader::class);
    }

    public function findOneByReaderToken(string $readerToken): ?RfidReader
    {
        $readerToken = trim($readerToken);
        if ($readerToken === '') {
            return null;
        }

        return $this->findOneBy(['readerToken' => $readerToken]);
    }

    /** @return RfidReader[] */
    public function findForAdmin(): array
    {
        return $this->createQueryBuilder('reader')
            ->leftJoin('reader.machine', 'machine')
            ->addSelect('machine')
            ->orderBy('reader.createdAt', 'DESC')
            ->getQuery()
            ->getResult();
    }
}
