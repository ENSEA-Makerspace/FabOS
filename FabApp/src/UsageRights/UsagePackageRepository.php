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
        private readonly UsageGrantSchema $schema,
        // S153 — `hasUnrestrictedAccess()` doit voir les attributions par GROUPE
        // autant que les personnelles ; c'est ce résolveur qui dit à quels
        // groupes une personne appartient, et il est déjà la source unique de
        // cette réponse pour `UsageGrantRepository`.
        private readonly AudienceResolver $audiences,
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

    /**
     * Écrire l'IDENTITÉ d'un package et rien d'autre : son nom, sa description,
     * son état actif (S153b).
     *
     * 🔴 **Pourquoi ce n'est pas `save()` avec les mêmes valeurs.** `save()`
     * réécrit AUSSI `fullAccess` et les lignes de `USAGE_PACKAGE_FEATURE`, donc
     * la carte d'identité devait lui repasser ce que le package portait — lu au
     * chargement de la page. Une soumission partie d'un onglet ouvert avant une
     * modification de la saisie réécrivait alors l'ancienne liste : renommer un
     * package RESSUSCITAIT des droits qu'on venait de lui retirer, sans un mot.
     * Mesuré en base sur un package réel : quatre lignes de feature pour deux
     * grants, dont la saisie ne pouvait pas être la cause.
     *
     * ⚠️ C'est la même famille que les « deux vérités » que toute cette phase
     * range, prise par un autre bout : un écran qui n'a rien à dire sur les
     * droits ne doit pas les ÉCRIRE, fût-ce en recopiant.
     */
    public function saveIdentity(int $id, string $name, string $description, bool $active): void
    {
        $name = mb_substr(trim($name), 0, 120);
        $description = mb_substr(trim($description), 0, 1000);
        if ($name === '') {
            throw new \InvalidArgumentException('Le nom du package est obligatoire.');
        }

        $this->db->update('USAGE_PACKAGE', [
            'name' => $name,
            'description' => $description,
            'active' => $active ? 1 : 0,
            'updatedAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
        ], ['id' => $id]);

        // ⚠️ Pas de test sur `affected_rows` — voir la note dans `save()` : MySQL
        // rend 0 quand rien ne change, et enregistrer deux fois la même identité
        // n'est pas une erreur.
        if (!$this->db->fetchOne('SELECT 1 FROM USAGE_PACKAGE WHERE id = :id', ['id' => $id])) {
            throw new \InvalidArgumentException('Package introuvable.');
        }
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
                // 🔴 **`affected_rows` n'est pas « la ligne existe »** (S153).
                // MySQL rend 0 quand l'UPDATE ne CHANGE rien — mêmes nom,
                // description, actif, et un `updatedAt` identique à la seconde
                // près. Enregistrer deux fois dans la même seconde levait donc
                // « Package introuvable » sur un package parfaitement présent,
                // ce que la sonde a attrapé au premier essai. La question posée
                // est l'existence, alors c'est elle qu'on pose.
                if ($updated !== 1 && !$this->db->fetchOne('SELECT 1 FROM USAGE_PACKAGE WHERE id = :id', ['id' => $id])) {
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

    /**
     * Everyone this package currently reaches — members **and** groups (S144a).
     *
     * 🔴 It was an `INNER JOIN UTILISATEUR`, so a group assignment was invisible
     * on the only screen that can revoke one. The migration had already written
     * such rows; they were held by the model, enforced by the reader, and shown
     * nowhere. A right you cannot see is a right you cannot take back.
     *
     * @return list<array{id:int,kind:string,userId:?int,name:string,detail:string,members:?int,validFrom:?string,validUntil:?string}>
     */
    public function assignmentsForPackage(int $packageId): array
    {
        try {
            return array_map(static function (array $row): array {
                $isGroup = $row['groupId'] !== null;

                return [
                    'id' => (int) $row['id'],
                    'kind' => $isGroup ? 'group' : 'member',
                    'userId' => $row['userId'] !== null ? (int) $row['userId'] : null,
                    'name' => $isGroup
                        ? (string) $row['groupLabel']
                        : (trim((string) ($row['firstName'] ?? '') . ' ' . (string) ($row['lastName'] ?? '')) ?: (string) $row['username']),
                    'detail' => $isGroup ? (string) $row['groupKey'] : (string) $row['email'],
                    // Null for a virtual audience: `user` and `guest` never have
                    // membership rows, so "0 members" would be a lie rather than
                    // a count. The screen says "every account" instead.
                    'members' => $isGroup && !(bool) $row['groupVirtual'] ? (int) $row['groupMembers'] : null,
                    'validFrom' => $row['validFrom'] !== null ? (string) $row['validFrom'] : null,
                    'validUntil' => $row['validUntil'] !== null ? (string) $row['validUntil'] : null,
                ];
            }, $this->db->fetchAllAssociative(
                'SELECT a.id, a.userId, a.groupId, a.validFrom, a.validUntil,
                        u.firstName, u.lastName, u.username, u.email,
                        g.label AS groupLabel, g.groupKey, g.virtual AS groupVirtual,
                        (SELECT COUNT(*) FROM USER_GROUP_MEMBER m WHERE m.groupId = g.id) AS groupMembers
                 FROM USAGE_RIGHT_ASSIGNMENT a
                 LEFT JOIN UTILISATEUR u ON u.id = a.userId
                 LEFT JOIN USER_GROUP g ON g.id = a.groupId
                 INNER JOIN USAGE_PACKAGE p ON p.id = a.packageId
                 WHERE a.packageId = :package AND a.revokedAt IS NULL
                   AND (a.userId IS NOT NULL OR a.groupId IS NOT NULL)
                 ORDER BY a.groupId IS NULL, g.label, u.lastName, u.firstName, u.username',
                ['package' => $packageId],
            ));
        } catch (\Throwable) {
            // ⚠️ An install that has not run the S133b migration has no
            // `USER_GROUP`, and the catch that used to mean "no group rows yet"
            // would now hide the MEMBER rows as well — the screen would report a
            // package as reaching nobody while it reached people. Fall back to
            // the question that can still be answered.
            return $this->memberAssignmentsOnly($packageId);
        }
    }

    /** @return list<array{id:int,kind:string,userId:?int,name:string,detail:string,members:?int,validFrom:?string,validUntil:?string}> */
    private function memberAssignmentsOnly(int $packageId): array
    {
        try {
            return array_map(static fn (array $row): array => [
                'id' => (int) $row['id'],
                'kind' => 'member',
                'userId' => (int) $row['userId'],
                'name' => trim((string) ($row['firstName'] ?? '') . ' ' . (string) ($row['lastName'] ?? '')) ?: (string) $row['username'],
                'detail' => (string) $row['email'],
                'members' => null,
                'validFrom' => $row['validFrom'] !== null ? (string) $row['validFrom'] : null,
                'validUntil' => $row['validUntil'] !== null ? (string) $row['validUntil'] : null,
            ], $this->db->fetchAllAssociative(
                'SELECT a.id, a.userId, a.validFrom, a.validUntil, u.firstName, u.lastName, u.username, u.email
                 FROM USAGE_RIGHT_ASSIGNMENT a INNER JOIN UTILISATEUR u ON u.id = a.userId
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
     * 🔴 **It counts DIRECT assignments only, and since S144a that is a gap the
     * screen has to admit.** `COUNT(DISTINCT a.userId)` skips every group row —
     * their `userId` is NULL — so a package held only by "the trainers" reports
     * zero members protected. The honest count needs the exact inverse of
     * `AudienceResolver::keysFor()`: role-seeded audiences, plus stored
     * `USER_GROUP_MEMBER` rows, plus the virtual `user` audience meaning every
     * active account. ⚠️ **Do not rewrite that mapping in SQL here** — a second
     * copy of "who is in this audience" is how the two answers drift apart, which
     * is the whole reason `AudienceResolver` exists. Until it is done, the
     * sentence on `/admin/settings` says which assignments it counted, because a
     * safety preflight that quietly measures half the model is worse than one
     * that measures half and says so. Tracked as S144e in ROADMAP.md.
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
     * Give a package to a GROUP rather than to one member (S144a).
     *
     * 🔴 **`USAGE_RIGHT_ASSIGNMENT.groupId` was written by nothing for two
     * sessions.** S133b added the column, `Version20260816140000` backfilled the
     * role-based assignments into it, and `UsageGrantRepository::paths()` has
     * read it ever since — but no screen could produce a row, so the only way to
     * give a package to "the trainers" was an `INSERT` by hand. A half-model with
     * no write surface reads as a feature and behaves as an absence; this is the
     * same fault as the venue parameter S134b fixed, one table along.
     *
     * ⚠️ **`guest` is refused on purpose.** `verdict()` consults grants only for a
     * signed-in `Utilisateur`, so an assignment to the anonymous audience could
     * never grant anything — it would be a control that does nothing. `user` is
     * accepted and means every active account, which is exactly how that audience
     * resolves in `AudienceResolver::keysFor()`.
     */
    public function assignGroup(int $packageId, string $groupKey, ?\DateTimeImmutable $from, ?\DateTimeImmutable $until, ?int $issuedById): void
    {
        if ($this->find($packageId) === null) {
            throw new \InvalidArgumentException('Package introuvable.');
        }
        if ($from !== null && $until !== null && $until <= $from) {
            throw new \InvalidArgumentException('La fin doit être après le début.');
        }
        if ($groupKey === AudienceResolver::GUEST) {
            throw new \InvalidArgumentException("L'audience « invité » n'a pas de compte : un package ne peut rien lui accorder.");
        }

        $groupId = $this->db->fetchOne('SELECT id FROM USER_GROUP WHERE groupKey = :key', ['key' => $groupKey]);
        if ($groupId === false || $groupId === null) {
            throw new \InvalidArgumentException('Groupe introuvable.');
        }

        $overlap = (bool) $this->db->fetchOne(
            'SELECT 1 FROM USAGE_RIGHT_ASSIGNMENT
             WHERE packageId = :package AND groupId = :group AND revokedAt IS NULL
               AND (:until IS NULL OR validFrom IS NULL OR validFrom < :until)
               AND (:from IS NULL OR validUntil IS NULL OR validUntil > :from)
             LIMIT 1',
            [
                'package' => $packageId, 'group' => (int) $groupId,
                'from' => $from?->format('Y-m-d H:i:s'), 'until' => $until?->format('Y-m-d H:i:s'),
            ],
        );
        if ($overlap) {
            throw new \InvalidArgumentException('Ce groupe possède déjà ce package sur tout ou partie de cette période.');
        }

        // ⚠️ `userId` stays NULL: the row belongs to the group, and writing both
        // would make a revocation ambiguous — revoking the group would appear to
        // revoke one member, or the other way round, depending on which query
        // read it.
        $this->db->insert('USAGE_RIGHT_ASSIGNMENT', [
            'packageId' => $packageId, 'userId' => null, 'groupId' => (int) $groupId,
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
     * @return list<array{id:int,featureKey:string,sectionKey:?string,action:string,venueId:?int,venueName:?string,reservableType:?string,reservableId:?int,categoryLabel:?string,windows:list<array{id:int,dayOfWeek:int,label:string}>}>
     */
    public function grantsFor(int $packageId): array
    {
        $scoped = $this->schema->hasScopeColumns();
        $scopeColumns = $scoped ? ', g.reservableType, g.reservableId, g.categoryLabel' : '';

        try {
            $rows = $this->db->fetchAllAssociative(
                "SELECT g.id, g.featureKey, g.sectionKey, g.action, g.venueId, v.name AS venueName{$scopeColumns}
                 FROM USAGE_PACKAGE_GRANT g
                 LEFT JOIN VENUE v ON v.id = g.venueId
                 WHERE g.packageId = :package
                 ORDER BY g.featureKey, g.action, g.venueId IS NOT NULL, v.name",
                ['package' => $packageId],
            );
        } catch (\Throwable) {
            return [];
        }

        $windows = $this->windowsFor(array_map(static fn (array $row): int => (int) $row['id'], $rows));

        return array_map(static fn (array $row): array => [
            'id' => (int) $row['id'],
            'featureKey' => (string) $row['featureKey'],
            'sectionKey' => $row['sectionKey'] !== null ? (string) $row['sectionKey'] : null,
            'action' => (string) $row['action'],
            'venueId' => $row['venueId'] !== null ? (int) $row['venueId'] : null,
            'venueName' => $row['venueName'] !== null ? (string) $row['venueName'] : null,
            'reservableType' => isset($row['reservableType']) && $row['reservableType'] !== null ? (string) $row['reservableType'] : null,
            'reservableId' => isset($row['reservableId']) && $row['reservableId'] !== null ? (int) $row['reservableId'] : null,
            'categoryLabel' => isset($row['categoryLabel']) && $row['categoryLabel'] !== null ? (string) $row['categoryLabel'] : null,
            'windows' => $windows[(int) $row['id']] ?? [],
        ], $rows);
    }

    /**
     * @param list<int> $grantIds
     * @return array<int, list<array{id:int,dayOfWeek:int,label:string}>>
     */
    private function windowsFor(array $grantIds): array
    {
        $grantIds = array_values(array_unique(array_filter($grantIds)));
        if ($grantIds === [] || !$this->schema->hasWindowTable()) {
            return [];
        }

        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT id, grantId, dayOfWeek, startMinute, endMinute FROM USAGE_GRANT_WINDOW
                 WHERE grantId IN (:ids) ORDER BY dayOfWeek, startMinute',
                ['ids' => $grantIds],
                ['ids' => \Doctrine\DBAL\ArrayParameterType::INTEGER],
            );
        } catch (\Throwable) {
            return [];
        }

        $windows = [];
        foreach ($rows as $row) {
            $windows[(int) $row['grantId']][] = [
                'id' => (int) $row['id'],
                'dayOfWeek' => (int) $row['dayOfWeek'],
                'label' => GrantWindow::fromRow($row)->label(),
            ];
        }

        return $windows;
    }

    /**
     * ⚠️ **Adding a grant can only ever widen access, so it needs no preflight.**
     * Grants combine with OR and nothing here subtracts — the roadmap's rule that
     * no package removes a right is a property of the model, not a check.
     * Removing one can narrow it, which is why `deleteGrant()` is the call the
     * screen puts behind a confirmation.
     */
    public function addGrant(
        int $packageId,
        string $featureKey,
        UsageGrantAction $action,
        ?int $venueId,
        ?string $sectionKey,
        ?string $reservableType = null,
        ?int $reservableId = null,
        ?string $categoryLabel = null,
        /**
         * ⚠️ **S147, J-21 — l'identité de la catégorie, écrite EN PLUS du libellé.**
         * Les deux cohabitent pendant la transition : le lecteur exige les deux
         * quand les deux sont posés, et un grant ancien qui n'a que le libellé
         * continue de fonctionner. Retirer le libellé est une étape de contract,
         * après un backfill vérifiable.
         */
        ?int $categoryId = null,
    ): int {
        // ⚠️ The scope columns join the identity of a grant only where they
        // exist. On an install that has the code and not the S144b migration, two
        // grants differing only by machine would collide as duplicates — but that
        // install cannot store the difference anyway, so refusing the second one
        // is the honest answer rather than a silent half-write.
        $scoped = $this->schema->hasScopeColumns();
        $categoryScoped = $scoped && $this->schema->hasCategoryIdColumn();
        $duplicate = (bool) $this->db->fetchOne(
            'SELECT 1 FROM USAGE_PACKAGE_GRANT
             WHERE packageId = :package AND featureKey = :feature AND action = :action
               AND venueId <=> :venue AND sectionKey <=> :section'
            . ($scoped ? ' AND reservableType <=> :rtype AND reservableId <=> :rid AND categoryLabel <=> :category' : '')
            . ' LIMIT 1',
            array_merge(
                ['package' => $packageId, 'feature' => $featureKey, 'action' => $action->value, 'venue' => $venueId, 'section' => $sectionKey],
                $scoped ? ['rtype' => $reservableType, 'rid' => $reservableId, 'category' => $categoryLabel] : [],
            ),
        );
        if ($duplicate) {
            throw new \InvalidArgumentException('Ce grant existe déjà dans ce package.');
        }

        $this->db->insert('USAGE_PACKAGE_GRANT', array_merge([
            'packageId' => $packageId, 'featureKey' => $featureKey, 'sectionKey' => $sectionKey,
            'action' => $action->value, 'venueId' => $venueId,
            'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
        ], $scoped ? [
            'reservableType' => $reservableType,
            'reservableId' => $reservableId,
            'categoryLabel' => $categoryLabel,
        ] : [], $categoryScoped ? ['categoryId' => $categoryId] : []));

        return (int) $this->db->lastInsertId();
    }

    /**
     * One weekly window of a grant — "Monday 14:00 to 18:00" (S144b).
     *
     * ⚠️ **Adding the FIRST window to a grant narrows it**, and that is the one
     * place in this model where writing something makes access smaller. Every
     * other write here only ever widens: a grant with no windows is awake all
     * week, so the first one turns "always" into "Mondays". The screen says so
     * before the first one is added rather than after.
     */
    public function addWindow(int $packageId, int $grantId, GrantWindow $window): void
    {
        if (!$this->schema->hasWindowTable()) {
            throw new \RuntimeException("La migration des créneaux n'a pas encore été exécutée sur cette installation.");
        }
        if (!$window->isValid()) {
            throw new \InvalidArgumentException('Créneau invalide : la fin doit être après le début, dans la même journée.');
        }
        // Scoped by package as well as by grant id, for the same reason
        // `deleteGrant()` is: an id arriving from another package's form must not
        // reach across the boundary.
        if (!$this->grantBelongsTo($packageId, $grantId)) {
            throw new \InvalidArgumentException('Grant introuvable dans ce package.');
        }

        try {
            $this->db->insert('USAGE_GRANT_WINDOW', [
                'grantId' => $grantId,
                'dayOfWeek' => $window->dayOfWeek,
                'startMinute' => $window->startMinute,
                'endMinute' => $window->endMinute,
                'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
            ]);
        } catch (\Throwable) {
            // The unique index is the guard; a repeat is not an error worth
            // stopping an operator with.
            throw new \InvalidArgumentException('Ce créneau existe déjà sur ce grant.');
        }
    }

    /**
     * ⚠️ **Deleting the LAST window of a grant widens it back to the whole week.**
     * The opposite of the usual direction, so the screen confirms it and says
     * which way it goes — "no window" means "any time", never "never".
     */
    public function deleteWindow(int $packageId, int $windowId): void
    {
        if (!$this->schema->hasWindowTable()) {
            return;
        }

        $this->db->executeStatement(
            'DELETE w FROM USAGE_GRANT_WINDOW w
             INNER JOIN USAGE_PACKAGE_GRANT g ON g.id = w.grantId
             WHERE w.id = :id AND g.packageId = :package',
            ['id' => $windowId, 'package' => $packageId],
        );
    }

    private function grantBelongsTo(int $packageId, int $grantId): bool
    {
        return (bool) $this->db->fetchOne(
            'SELECT 1 FROM USAGE_PACKAGE_GRANT WHERE id = :id AND packageId = :package LIMIT 1',
            ['id' => $grantId, 'package' => $packageId],
        );
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

    /**
     * Réécrire d'un coup TOUS les grants d'un package, fenêtres comprises (S153).
     *
     * 🔴 **C'est le point d'écriture du compilateur, et il remplace au lieu
     * d'ajouter — parce que la SAISIE décrit le package entier.** L'éditeur
     * grant-par-grant ajoute ; un formulaire qui dit « ce package donne accès à
     * X, le jeudi » doit produire exactement ça, sinon décocher une case ne
     * retirerait jamais rien et l'écran mentirait dans le sens le plus
     * dangereux : celui qui laisse un droit en place.
     *
     * ⚠️ **Transactionnel, et les fenêtres partent avec leurs grants** — la
     * contrainte `FK_USAGE_GRANT_WINDOW_GRANT` est `ON DELETE CASCADE`. Un
     * remplacement à moitié écrit serait un package qui n'autorise plus rien.
     *
     * ⚠️ **Les fenêtres ne sont écrites que si la table répond.** Sur une
     * installation qui a le code sans la migration, le grant est écrit sans sa
     * restriction horaire : plus large que demandé, jamais plus étroit. C'est la
     * direction de repli de tout ce fichier — voir `UsageGrantSchema`.
     *
     * @param list<array{featureKey:string,action:UsageGrantAction,venueId:?int,sectionKey:?string,reservableType:?string,reservableId:?int,categoryLabel:?string,categoryId:?int,windows:list<GrantWindow>}> $grants
     */
    public function replaceGrants(int $packageId, array $grants): void
    {
        $scoped = $this->schema->hasScopeColumns();
        $categoryScoped = $scoped && $this->schema->hasCategoryIdColumn();
        $windowed = $this->schema->hasWindowTable();

        $this->db->transactional(function () use ($packageId, $grants, $scoped, $categoryScoped, $windowed): void {
            $this->db->delete('USAGE_PACKAGE_GRANT', ['packageId' => $packageId]);
            foreach ($grants as $grant) {
                $now = (new \DateTimeImmutable())->format('Y-m-d H:i:s');
                $this->db->insert('USAGE_PACKAGE_GRANT', array_merge([
                    'packageId' => $packageId,
                    'featureKey' => $grant['featureKey'],
                    'sectionKey' => $grant['sectionKey'],
                    'action' => $grant['action']->value,
                    'venueId' => $grant['venueId'],
                    'createdAt' => $now,
                ], $scoped ? [
                    'reservableType' => $grant['reservableType'],
                    'reservableId' => $grant['reservableId'],
                    'categoryLabel' => $grant['categoryLabel'],
                ] : [], $categoryScoped ? ['categoryId' => $grant['categoryId']] : []));

                if (!$windowed) {
                    continue;
                }
                $grantId = (int) $this->db->lastInsertId();
                foreach ($grant['windows'] as $window) {
                    if (!$window->isValid()) {
                        throw new \InvalidArgumentException('Créneau invalide : la fin doit être après le début, dans la même journée.');
                    }
                    $this->db->insert('USAGE_GRANT_WINDOW', [
                        'grantId' => $grantId,
                        'dayOfWeek' => $window->dayOfWeek,
                        'startMinute' => $window->startMinute,
                        'endMinute' => $window->endMinute,
                        'createdAt' => $now,
                    ]);
                }
            }
        });
    }

    /**
     * Cette personne détient-elle, à cet instant, un package SANS AUCUNE
     * RESTRICTION ? (S153)
     *
     * 🔴 **C'est ce qui porte la case « sans limite d'horaires ».** Elle
     * réutilise `USAGE_PACKAGE.fullAccess` — donc aucune migration — et la
     * colonne y gagne son sens complet : *ce package ne connaît aucune
     * restriction*, ni de feature, ni de lieu, ni d'heure. Le compilateur ne
     * l'écrit QUE quand les quatre axes sont sur « aucune restriction » : un
     * seul bit ne peut pas dire « tout sauf le jeudi », et le formulaire n'offre
     * la case que dans ce cas.
     *
     * ⚠️ **Les deux sources d'attribution, comme partout ailleurs** : la
     * personne, et les groupes dont elle fait partie. `grantingPackages()` ne
     * regarde que `a.userId` et c'est une limite connue de la v1 ; ici la
     * question décide d'un accès hors horaires, donc elle se pose comme
     * `UsageGrantRepository::grantRows()` la pose.
     */
    public function hasUnrestrictedAccess(?Utilisateur $user, \DateTimeImmutable $at): bool
    {
        if (!$user instanceof Utilisateur) {
            return false;
        }

        $keys = $this->audiences->keysFor($user);

        try {
            return (bool) $this->db->fetchOne(
                'SELECT 1 FROM USAGE_RIGHT_ASSIGNMENT a
                 INNER JOIN USAGE_PACKAGE p ON p.id = a.packageId AND p.active = 1 AND p.fullAccess = 1
                 LEFT JOIN USER_GROUP ug ON ug.id = a.groupId
                 WHERE a.revokedAt IS NULL
                   AND (a.validFrom IS NULL OR a.validFrom <= :moment)
                   AND (a.validUntil IS NULL OR a.validUntil > :moment)
                   AND (
                         (:userId IS NOT NULL AND a.userId = :userId)
                      OR (a.groupId IS NOT NULL AND ug.groupKey IN (:keys))
                   )
                 LIMIT 1',
                [
                    'moment' => $at->format('Y-m-d H:i:s'),
                    'userId' => $user->getId(),
                    'keys' => $keys === [] ? [''] : $keys,
                ],
                ['keys' => \Doctrine\DBAL\ArrayParameterType::STRING],
            );
        } catch (\Throwable) {
            // ⚠️ Le repli est le comportement d'AVANT : pas d'exemption. Une
            // requête qui échoue ne doit jamais ouvrir le lab.
            return false;
        }
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
