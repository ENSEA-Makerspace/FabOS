<?php

namespace App\UsageRights;

use App\Entity\Utilisateur;
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
    ) {
    }

    /** @return list<array{id:int,name:string,description:string,active:bool,fullAccess:bool,features:list<string>,assignments:int}> */
    public function findAll(): array
    {
        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT p.id, p.name, p.description, p.active, p.fullAccess, COUNT(DISTINCT a.id) AS assignments
                 FROM USAGE_PACKAGE p
                 LEFT JOIN USAGE_RIGHT_ASSIGNMENT a ON a.packageId = p.id AND a.revokedAt IS NULL
                 GROUP BY p.id
                 ORDER BY p.name ASC',
                [],
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
            'fullAccess' => (bool) $row['fullAccess'],
            'features' => $features[(int) $row['id']] ?? [],
            'assignments' => (int) $row['assignments'],
        ], $rows);
    }

    /** @return array{id:int,name:string,description:string,active:bool,fullAccess:bool,features:list<string>}|null */
    public function find(int $id): ?array
    {
        try {
            $row = $this->db->fetchAssociative(
                'SELECT id, name, description, active, fullAccess FROM USAGE_PACKAGE WHERE id = :id',
                ['id' => $id],
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
            'fullAccess' => (bool) $row['fullAccess'],
            'features' => $this->featuresForPackages([(int) $row['id']])[(int) $row['id']] ?? [],
        ];
    }

    /** @param list<string> $features */
    public function save(?int $id, string $name, string $description, bool $active, bool $fullAccess, array $features): int
    {
        $name = mb_substr(trim($name), 0, 120);
        $description = mb_substr(trim($description), 0, 1000);
        if ($name === '') {
            throw new \InvalidArgumentException('Le nom du package est obligatoire.');
        }

        $features = array_values(array_unique(array_filter(array_map('strval', $features))));
        $this->db->transactional(function () use (&$id, $name, $description, $active, $fullAccess, $features): void {
            if ($id === null) {
                $this->db->insert('USAGE_PACKAGE', [
                    'name' => $name, 'description' => $description,
                    'active' => $active ? 1 : 0, 'fullAccess' => $fullAccess ? 1 : 0, 'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
                    'updatedAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
                ]);
                $id = (int) $this->db->lastInsertId();
            } else {
                $updated = $this->db->update('USAGE_PACKAGE', [
                    'name' => $name, 'description' => $description, 'active' => $active ? 1 : 0, 'fullAccess' => $fullAccess ? 1 : 0,
                    'updatedAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
                ], ['id' => $id]);
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

        $overlap = (bool) $this->db->fetchOne(
            'SELECT 1 FROM USAGE_RIGHT_ASSIGNMENT
             WHERE packageId = :package AND userId = :user AND revokedAt IS NULL
               AND (:until IS NULL OR validFrom IS NULL OR validFrom < :until)
               AND (:from IS NULL OR validUntil IS NULL OR validUntil > :from)
             LIMIT 1',
            [
                'package' => $packageId, 'user' => $user->getId(),
                'from' => $from?->format('Y-m-d H:i:s'), 'until' => $until?->format('Y-m-d H:i:s'),
            ],
        );
        if ($overlap) {
            throw new \InvalidArgumentException('Ce membre possède déjà ce package sur tout ou partie de cette période.');
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
                 WHERE a.packageId = :package AND a.revokedAt IS NULL
                 ORDER BY u.lastName, u.firstName, u.username',
                ['package' => $packageId],
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
                 WHERE a.userId = :user AND p.active = 1 AND a.revokedAt IS NULL
                 ORDER BY p.name',
                ['user' => $user->getId()],
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

    public function revoke(int $assignmentId, ?int $revokedById = null): void
    {
        $this->db->executeStatement(
            'UPDATE USAGE_RIGHT_ASSIGNMENT a INNER JOIN USAGE_PACKAGE p ON p.id = a.packageId
             SET a.revokedAt = :now, a.revokedById = :actor
             WHERE a.id = :id AND a.revokedAt IS NULL',
            ['id' => $assignmentId, 'actor' => $revokedById, 'now' => (new \DateTimeImmutable())->format('Y-m-d H:i:s')],
        );
    }

    /**
     * Small activation preflight. This deliberately counts effective, current
     * assignments rather than every historical row: the settings screen must
     * never imply that expired or future grants protect members today.
     *
     * @param list<string> $capabilities
     * @return array{packages:int,members:int,coverage:array<string,int>}
     */
    public function readiness(array $capabilities, \DateTimeImmutable $now): array
    {
        $moment = $now->format('Y-m-d H:i:s');
        try {
            $packages = (int) $this->db->fetchOne(
                'SELECT COUNT(*) FROM USAGE_PACKAGE WHERE active = 1',
            );
            $members = (int) $this->db->fetchOne(
                'SELECT COUNT(DISTINCT a.userId) FROM USAGE_RIGHT_ASSIGNMENT a
                 INNER JOIN USAGE_PACKAGE p ON p.id = a.packageId
                 WHERE p.active = 1 AND a.revokedAt IS NULL
                   AND (a.validFrom IS NULL OR a.validFrom <= :now)
                   AND (a.validUntil IS NULL OR a.validUntil >= :now)',
                ['now' => $moment],
            );
            $coverage = [];
            foreach ($capabilities as $capability) {
                $coverage[$capability] = (int) $this->db->fetchOne(
                    'SELECT COUNT(DISTINCT a.userId) FROM USAGE_RIGHT_ASSIGNMENT a
                     INNER JOIN USAGE_PACKAGE p ON p.id = a.packageId
                     LEFT JOIN USAGE_PACKAGE_FEATURE f ON f.packageId = p.id AND f.featureKey = :feature
                     WHERE p.active = 1 AND a.revokedAt IS NULL
                       AND (p.fullAccess = 1 OR f.featureKey IS NOT NULL)
                       AND (a.validFrom IS NULL OR a.validFrom <= :now)
                       AND (a.validUntil IS NULL OR a.validUntil >= :now)',
                    ['feature' => $capability, 'now' => $moment],
                );
            }

            return ['packages' => $packages, 'members' => $members, 'coverage' => $coverage];
        } catch (\Throwable) {
            return ['packages' => 0, 'members' => 0, 'coverage' => array_fill_keys($capabilities, 0)];
        }
    }

    /** @return list<string> package names granting the capability right now */
    public function grantingPackages(Utilisateur $user, string $feature, \DateTimeImmutable $from, ?\DateTimeImmutable $until = null): array
    {
        if ($user->getId() === null) {
            return [];
        }
        try {
            return array_values(array_map('strval', $this->db->fetchFirstColumn(
                'SELECT DISTINCT p.name FROM USAGE_RIGHT_ASSIGNMENT a
                 INNER JOIN USAGE_PACKAGE p ON p.id = a.packageId
                 LEFT JOIN USAGE_PACKAGE_FEATURE f ON f.packageId = p.id AND f.featureKey = :feature
                 WHERE a.userId = :user AND a.revokedAt IS NULL AND p.active = 1
                   AND (p.fullAccess = 1 OR f.featureKey IS NOT NULL)
                   AND (a.validFrom IS NULL OR a.validFrom <= :from)
                   AND (a.validUntil IS NULL OR a.validUntil >= :until)
                 ORDER BY p.name',
                [
                    'user' => $user->getId(), 'feature' => $feature,
                    'from' => $from->format('Y-m-d H:i:s'), 'until' => ($until ?? $from)->format('Y-m-d H:i:s'),
                ],
            )));
        } catch (\Throwable) {
            return [];
        }
    }

    public function allows(Utilisateur $user, string $feature, \DateTimeImmutable $now): bool
    {
        if ($user->getId() === null) {
            return false;
        }
        return $this->grantingPackages($user, $feature, $now) !== [];
    }

    /**
     * S111 shadow read. It intentionally has no caller in the live gate: a
     * package can reach a person directly or through any current role, and all
     * requested dimensions must be covered by the same grant row.
     *
     * @return list<string>
     */
    public function v2GrantingPackages(Utilisateur $user, string $feature, UsageGrantAction $action, ?int $venueId, \DateTimeImmutable $at): array
    {
        if ($user->getId() === null) {
            return [];
        }

        try {
            return array_values(array_map('strval', $this->db->fetchFirstColumn(
                "SELECT DISTINCT p.name
                 FROM USAGE_PACKAGE_GRANT g
                 INNER JOIN USAGE_PACKAGE p ON p.id = g.packageId
                 LEFT JOIN USAGE_RIGHT_ASSIGNMENT directAssignment ON directAssignment.packageId = p.id
                   AND directAssignment.userId = :user AND directAssignment.revokedAt IS NULL
                 LEFT JOIN USAGE_PACKAGE_GROUP_ASSIGNMENT groupAssignment ON groupAssignment.packageId = p.id
                   AND groupAssignment.revokedAt IS NULL
                 LEFT JOIN UTILISATEUR_ROLE membership ON membership.roleId = groupAssignment.roleId
                   AND membership.utilisateurId = :user
                 WHERE p.active = 1 AND g.featureKey = :feature AND g.action = :action
                   AND (:venue IS NULL OR g.venueId IS NULL OR g.venueId = :venue)
                   AND ((directAssignment.id IS NOT NULL AND (directAssignment.validFrom IS NULL OR directAssignment.validFrom <= :at) AND (directAssignment.validUntil IS NULL OR directAssignment.validUntil >= :at))
                     OR (membership.utilisateurId IS NOT NULL AND (groupAssignment.validFrom IS NULL OR groupAssignment.validFrom <= :at) AND (groupAssignment.validUntil IS NULL OR groupAssignment.validUntil >= :at)))
                 ORDER BY p.name",
                ['user' => $user->getId(), 'feature' => $feature, 'action' => $action->value, 'venue' => $venueId, 'at' => $at->format('Y-m-d H:i:s')],
            )));
        } catch (\Throwable) {
            return [];
        }
    }

    public function assignGroup(int $packageId, int $roleId, ?\DateTimeImmutable $from, ?\DateTimeImmutable $until, ?int $issuedById): void
    {
        if ($from !== null && $until !== null && $until <= $from) {
            throw new \InvalidArgumentException('La fin doit être après le début.');
        }
        $this->db->insert('USAGE_PACKAGE_GROUP_ASSIGNMENT', [
            'packageId' => $packageId, 'roleId' => $roleId,
            'validFrom' => $from?->format('Y-m-d H:i:s'), 'validUntil' => $until?->format('Y-m-d H:i:s'),
            'issuedById' => $issuedById, 'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
        ]);
    }

    /**
     * The grants of one package, for the editor (S134b).
     *
     * ⚠️ Ordered so an unscoped grant sorts before the scoped ones of the same
     * feature: an unscoped `use` grant makes every venue-scoped `use` grant on the
     * same feature redundant, and reading them in that order is what lets the
     * screen say so instead of leaving an operator to work it out.
     *
     * @return list<array{id:int,featureKey:string,sectionKey:?string,action:string,venueId:?int,venueName:?string}>
     */
    public function grantsFor(int $packageId): array
    {
        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT g.id, g.featureKey, g.sectionKey, g.action, g.venueId, v.name AS venueName
                 FROM USAGE_PACKAGE_GRANT g
                 LEFT JOIN VENUE v ON v.id = g.venueId
                 WHERE g.packageId = :package
                 ORDER BY g.featureKey, g.action, g.venueId IS NOT NULL, v.name',
                ['package' => $packageId],
            );
        } catch (\Throwable) {
            return [];
        }

        return array_map(static fn (array $row): array => [
            'id' => (int) $row['id'],
            'featureKey' => (string) $row['featureKey'],
            'sectionKey' => $row['sectionKey'] !== null ? (string) $row['sectionKey'] : null,
            'action' => (string) $row['action'],
            'venueId' => $row['venueId'] !== null ? (int) $row['venueId'] : null,
            'venueName' => $row['venueName'] !== null ? (string) $row['venueName'] : null,
        ], $rows);
    }

    /**
     * ⚠️ **Adding a grant can only ever widen access, so it needs no preflight.**
     * Grants combine with OR and nothing here subtracts — the roadmap's rule that
     * no package removes a right is a property of the model, not a check.
     * Removing one can narrow it, which is why `deleteGrant()` is the call the
     * screen puts behind a confirmation.
     */
    public function addGrant(int $packageId, string $featureKey, UsageGrantAction $action, ?int $venueId, ?string $sectionKey): void
    {
        $duplicate = (bool) $this->db->fetchOne(
            'SELECT 1 FROM USAGE_PACKAGE_GRANT
             WHERE packageId = :package AND featureKey = :feature AND action = :action
               AND venueId <=> :venue AND sectionKey <=> :section LIMIT 1',
            ['package' => $packageId, 'feature' => $featureKey, 'action' => $action->value, 'venue' => $venueId, 'section' => $sectionKey],
        );
        if ($duplicate) {
            throw new \InvalidArgumentException('Ce grant existe déjà dans ce package.');
        }

        $this->db->insert('USAGE_PACKAGE_GRANT', [
            'packageId' => $packageId, 'featureKey' => $featureKey, 'sectionKey' => $sectionKey,
            'action' => $action->value, 'venueId' => $venueId,
            'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
        ]);
    }

    public function deleteGrant(int $packageId, int $grantId): void
    {
        // Scoped by package as well as by id: a grant id arriving from a form on
        // another package's page must not delete across the boundary.
        $this->db->executeStatement(
            'DELETE FROM USAGE_PACKAGE_GRANT WHERE id = :id AND packageId = :package',
            ['id' => $grantId, 'package' => $packageId],
        );
    }

    /** @param list<ShadowUsageGrant> $grants */
    public function replaceV2Grants(int $packageId, array $grants): void
    {
        $this->db->transactional(function () use ($packageId, $grants): void {
            $this->db->delete('USAGE_PACKAGE_GRANT', ['packageId' => $packageId]);
            foreach ($grants as $grant) {
                $venueIds = $grant->scopes['venue'] ?? [null];
                foreach ($venueIds as $venueId) {
                    $this->db->insert('USAGE_PACKAGE_GRANT', [
                        'packageId' => $packageId, 'featureKey' => $grant->feature, 'sectionKey' => $grant->section,
                        'action' => $grant->action->value, 'venueId' => $venueId === null ? null : (int) $venueId,
                        'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
                    ]);
                }
            }
        });
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
