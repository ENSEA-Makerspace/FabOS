<?php

namespace App\UsageRights;

use App\Entity\Utilisateur;
use App\Portal\PortalContext;
use Doctrine\DBAL\Connection;

/**
 * DBAL storage for usage packages. Packages are permission blueprints; an
 * assignment is the time-bounded grant to one member. Keeping feature grants
 * in rows, instead of JSON copied onto users, makes one package edit apply to
 * every current assignment and keeps the feature registry the only catalogue
 * of available product features.
 */
final class UsagePackageRepository
{
    public function __construct(
        private readonly Connection $db,
        private readonly PortalContext $portals,
    ) {
    }

    /** @return list<array{id:int,name:string,description:string,active:bool,features:list<string>,assignments:int}> */
    public function findAll(): array
    {
        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT p.id, p.name, p.description, p.active, COUNT(DISTINCT a.id) AS assignments
                 FROM USAGE_PACKAGE p
                 LEFT JOIN USAGE_RIGHT_ASSIGNMENT a ON a.packageId = p.id AND a.revokedAt IS NULL
                 WHERE p.portalId = :portal
                 GROUP BY p.id
                 ORDER BY p.name ASC',
                ['portal' => $this->portals->scopeId()],
            );
        } catch (\Throwable) {
            return [];
        }

        $features = $this->featuresForPackages(array_map(static fn (array $row): int => (int) $row['id'], $rows));

        return array_map(static fn (array $row): array => [
            'id' => (int) $row['id'],
            'name' => (string) $row['name'],
            'description' => (string) ($row['description'] ?? ''),
            'active' => (bool) $row['active'],
            'features' => $features[(int) $row['id']] ?? [],
            'assignments' => (int) $row['assignments'],
        ], $rows);
    }

    /** @return array{id:int,name:string,description:string,active:bool,features:list<string>}|null */
    public function find(int $id): ?array
    {
        try {
            $row = $this->db->fetchAssociative(
                'SELECT id, name, description, active FROM USAGE_PACKAGE WHERE id = :id AND portalId = :portal',
                ['id' => $id, 'portal' => $this->portals->scopeId()],
            );
        } catch (\Throwable) {
            return null;
        }
        if (!is_array($row)) {
            return null;
        }

        return [
            'id' => (int) $row['id'],
            'name' => (string) $row['name'],
            'description' => (string) ($row['description'] ?? ''),
            'active' => (bool) $row['active'],
            'features' => $this->featuresForPackages([(int) $row['id']])[(int) $row['id']] ?? [],
        ];
    }

    /** @param list<string> $features */
    public function save(?int $id, string $name, string $description, bool $active, array $features): int
    {
        $name = mb_substr(trim($name), 0, 120);
        $description = mb_substr(trim($description), 0, 1000);
        if ($name === '') {
            throw new \InvalidArgumentException('Le nom du package est obligatoire.');
        }

        $features = array_values(array_unique(array_filter(array_map('strval', $features))));
        $portal = $this->portals->scopeId();

        $this->db->transactional(function () use (&$id, $name, $description, $active, $features, $portal): void {
            if ($id === null) {
                $this->db->insert('USAGE_PACKAGE', [
                    'portalId' => $portal, 'name' => $name, 'description' => $description,
                    'active' => $active ? 1 : 0, 'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
                    'updatedAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
                ]);
                $id = (int) $this->db->lastInsertId();
            } else {
                $updated = $this->db->update('USAGE_PACKAGE', [
                    'name' => $name, 'description' => $description, 'active' => $active ? 1 : 0,
                    'updatedAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
                ], ['id' => $id, 'portalId' => $portal]);
                if ($updated !== 1) {
                    throw new \InvalidArgumentException('Package introuvable.');
                }
                $this->db->delete('USAGE_PACKAGE_FEATURE', ['packageId' => $id]);
            }

            foreach ($features as $feature) {
                $this->db->insert('USAGE_PACKAGE_FEATURE', ['packageId' => $id, 'featureKey' => $feature]);
            }
        });

        return $id ?? throw new \LogicException('Package non créé.');
    }

    public function assign(int $packageId, Utilisateur $user, ?\DateTimeImmutable $from, ?\DateTimeImmutable $until, ?int $issuedById): void
    {
        if ($user->getId() === null || $this->find($packageId) === null) {
            throw new \InvalidArgumentException('Utilisateur ou package introuvable.');
        }
        if ($from !== null && $until !== null && $until <= $from) {
            throw new \InvalidArgumentException('La fin doit être après le début.');
        }

        $this->db->insert('USAGE_RIGHT_ASSIGNMENT', [
            'packageId' => $packageId, 'userId' => $user->getId(),
            'validFrom' => $from?->format('Y-m-d H:i:s'), 'validUntil' => $until?->format('Y-m-d H:i:s'),
            'issuedById' => $issuedById, 'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
        ]);
    }

    /** @return list<array{id:int,userId:int,name:string,email:string,validFrom:?string,validUntil:?string}> */
    public function assignmentsForPackage(int $packageId): array
    {
        try {
            return array_map(static fn (array $row): array => [
                'id' => (int) $row['id'], 'userId' => (int) $row['userId'],
                'name' => trim((string) ($row['firstName'] ?? '') . ' ' . (string) ($row['lastName'] ?? '')) ?: (string) $row['username'],
                'email' => (string) $row['email'],
                'validFrom' => $row['validFrom'] !== null ? (string) $row['validFrom'] : null,
                'validUntil' => $row['validUntil'] !== null ? (string) $row['validUntil'] : null,
            ], $this->db->fetchAllAssociative(
                'SELECT a.id, a.userId, a.validFrom, a.validUntil, u.firstName, u.lastName, u.username, u.email
                 FROM USAGE_RIGHT_ASSIGNMENT a INNER JOIN UTILISATEUR u ON u.id = a.userId
                 INNER JOIN USAGE_PACKAGE p ON p.id = a.packageId
                 WHERE a.packageId = :package AND p.portalId = :portal AND a.revokedAt IS NULL
                 ORDER BY u.lastName, u.firstName, u.username',
                ['package' => $packageId, 'portal' => $this->portals->scopeId()],
            ));
        } catch (\Throwable) {
            return [];
        }
    }

    /** @return list<array{id:int,packageId:int,name:string,features:list<string>,validFrom:?string,validUntil:?string}> */
    public function assignmentsForUser(Utilisateur $user): array
    {
        if ($user->getId() === null) {
            return [];
        }
        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT a.id, a.packageId, a.validFrom, a.validUntil, p.name
                 FROM USAGE_RIGHT_ASSIGNMENT a INNER JOIN USAGE_PACKAGE p ON p.id = a.packageId
                 WHERE a.userId = :user AND p.portalId = :portal AND p.active = 1 AND a.revokedAt IS NULL
                 ORDER BY p.name',
                ['user' => $user->getId(), 'portal' => $this->portals->scopeId()],
            );
        } catch (\Throwable) {
            return [];
        }
        $features = $this->featuresForPackages(array_values(array_unique(array_map(static fn (array $row): int => (int) $row['packageId'], $rows))));

        return array_map(static fn (array $row): array => [
            'id' => (int) $row['id'], 'packageId' => (int) $row['packageId'], 'name' => (string) $row['name'],
            'features' => $features[(int) $row['packageId']] ?? [],
            'validFrom' => $row['validFrom'] !== null ? (string) $row['validFrom'] : null,
            'validUntil' => $row['validUntil'] !== null ? (string) $row['validUntil'] : null,
        ], $rows);
    }

    public function revoke(int $assignmentId): void
    {
        $this->db->executeStatement(
            'UPDATE USAGE_RIGHT_ASSIGNMENT a INNER JOIN USAGE_PACKAGE p ON p.id = a.packageId
             SET a.revokedAt = :now WHERE a.id = :id AND p.portalId = :portal AND a.revokedAt IS NULL',
            ['id' => $assignmentId, 'portal' => $this->portals->scopeId(), 'now' => (new \DateTimeImmutable())->format('Y-m-d H:i:s')],
        );
    }

    public function allows(Utilisateur $user, string $feature, \DateTimeImmutable $now): bool
    {
        if ($user->getId() === null) {
            return false;
        }
        try {
            return (bool) $this->db->fetchOne(
                'SELECT 1 FROM USAGE_RIGHT_ASSIGNMENT a
                 INNER JOIN USAGE_PACKAGE p ON p.id = a.packageId
                 INNER JOIN USAGE_PACKAGE_FEATURE f ON f.packageId = p.id
                 WHERE a.userId = :user AND a.revokedAt IS NULL AND p.portalId = :portal AND p.active = 1
                   AND f.featureKey = :feature
                   AND (a.validFrom IS NULL OR a.validFrom <= :now)
                   AND (a.validUntil IS NULL OR a.validUntil >= :now)
                 LIMIT 1',
                ['user' => $user->getId(), 'portal' => $this->portals->scopeId(), 'feature' => $feature, 'now' => $now->format('Y-m-d H:i:s')],
            );
        } catch (\Throwable) {
            return false;
        }
    }

    /** @param list<int> $packageIds @return array<int,list<string>> */
    private function featuresForPackages(array $packageIds): array
    {
        if ($packageIds === []) {
            return [];
        }
        try {
            $rows = $this->db->executeQuery(
                'SELECT packageId, featureKey FROM USAGE_PACKAGE_FEATURE WHERE packageId IN (:ids) ORDER BY featureKey',
                ['ids' => $packageIds], ['ids' => Connection::PARAM_INT_ARRAY],
            )->fetchAllAssociative();
        } catch (\Throwable) {
            return [];
        }
        $features = [];
        foreach ($rows as $row) {
            $features[(int) $row['packageId']][] = (string) $row['featureKey'];
        }
        return $features;
    }
}
