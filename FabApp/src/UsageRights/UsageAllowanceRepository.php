<?php

declare(strict_types=1);

namespace App\UsageRights;

use App\Entity\Utilisateur;
use Doctrine\DBAL\Connection;

/**
 * Storage for package allowances (S144c).
 *
 * ⚠️ **Reads reach a member the same two ways a grant does** — a personal
 * assignment or a group they are in. Writing a second membership rule here is
 * how the two would drift; the join is a copy of the one in
 * `UsageGrantRepository::paths()` and must stay one.
 *
 * ⚠️ **Fail-safe means "no allowance"**, which means "no ceiling". Every catch
 * here returns an empty list on purpose: this table can only ever refuse, so a
 * schema or query failure must lose the restriction rather than invent one.
 */
final class UsageAllowanceRepository
{
    private ?bool $tableExists = null;

    public function __construct(
        private readonly Connection $db,
        private readonly AudienceResolver $audiences,
    ) {
    }

    /**
     * Every allowance currently reaching this member.
     *
     * @return list<UsageAllowance>
     */
    public function activeFor(Utilisateur $user, \DateTimeImmutable $at): array
    {
        if ($user->getId() === null || !$this->tableExists()) {
            return [];
        }

        $keys = $this->audiences->keysFor($user);

        try {
            $rows = $this->db->fetchAllAssociative(
                <<<'SQL'
                SELECT DISTINCT al.id, al.packageId, al.featureKey, al.reservableType, al.unit, al.amount, al.period
                FROM USAGE_PACKAGE_ALLOWANCE al
                INNER JOIN USAGE_PACKAGE p ON p.id = al.packageId AND p.active = 1
                INNER JOIN USAGE_RIGHT_ASSIGNMENT a ON a.packageId = p.id
                LEFT JOIN USER_GROUP ug ON ug.id = a.groupId
                WHERE a.revokedAt IS NULL
                  AND (a.validFrom IS NULL OR a.validFrom <= :moment)
                  AND (a.validUntil IS NULL OR a.validUntil > :moment)
                  AND (
                        a.userId = :userId
                     OR (a.groupId IS NOT NULL AND ug.groupKey IN (:keys))
                  )
                ORDER BY al.id
                SQL,
                [
                    'moment' => $at->format('Y-m-d H:i:s'),
                    'userId' => $user->getId(),
                    'keys' => $keys === [] ? [''] : $keys,
                ],
                ['keys' => \Doctrine\DBAL\ArrayParameterType::STRING],
            );
        } catch (\Throwable) {
            return [];
        }

        return array_map(static fn (array $row): UsageAllowance => UsageAllowance::fromRow($row), $rows);
    }

    /** @return list<UsageAllowance> */
    public function forPackage(int $packageId): array
    {
        if (!$this->tableExists()) {
            return [];
        }

        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT id, packageId, featureKey, reservableType, unit, amount, period
                 FROM USAGE_PACKAGE_ALLOWANCE WHERE packageId = :package ORDER BY period, unit, id',
                ['package' => $packageId],
            );
        } catch (\Throwable) {
            return [];
        }

        return array_map(static fn (array $row): UsageAllowance => UsageAllowance::fromRow($row), $rows);
    }

    /**
     * ⚠️ **Adding an allowance NARROWS access**, unlike every other write in the
     * package editor. A package with no allowance is unlimited, so the first row
     * turns "as much as you like" into "ten hours a week" — the screen says so
     * before the first one is added, not after somebody is refused.
     */
    public function add(int $packageId, ?string $featureKey, ?string $reservableType, string $unit, int $amount, string $period): void
    {
        if (!$this->tableExists()) {
            throw new \RuntimeException("La migration des allocations n'a pas encore été exécutée sur cette installation.");
        }
        if (!UsageAllowance::isValidUnit($unit) || !UsageAllowance::isValidPeriod($period)) {
            throw new \InvalidArgumentException('Unité ou période inconnue.');
        }
        if ($amount <= 0) {
            // ⚠️ Zero is refused rather than stored. "Nobody may book anything"
            // is expressible by removing the grant, and an allowance of zero on a
            // package that grants access is a contradiction an operator would
            // read as "unlimited".
            throw new \InvalidArgumentException('Une allocation doit être strictement positive.');
        }

        $this->db->insert('USAGE_PACKAGE_ALLOWANCE', [
            'packageId' => $packageId,
            'featureKey' => $featureKey,
            'reservableType' => $reservableType,
            'unit' => $unit,
            'amount' => $amount,
            'period' => $period,
            'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
        ]);
    }

    /**
     * Réécrire d'un coup TOUTES les allocations d'un package (S153).
     *
     * 🔴 **Remplacer, pas ajouter** — même raison que
     * `UsagePackageRepository::replaceGrants()` : la saisie décrit le package
     * entier, donc recocher « sans limite » doit RETIRER le quota. Un ajout
     * laisserait le plafond en place au-dessus d'une phrase qui affirme qu'il
     * n'y en a pas, et c'est le sens du mensonge qui refuse.
     *
     * ⚠️ **Une table absente n'est pas une erreur quand il n'y a rien à
     * écrire.** Sur une installation sans la migration, un package « sans
     * limite » se compile sans bruit ; c'est seulement écrire un quota qui
     * demande la table, et ça, on le dit.
     *
     * @param list<array{featureKey:?string,reservableType:?string,unit:string,amount:int,period:string}> $rows
     */
    public function replaceForPackage(int $packageId, array $rows): void
    {
        if (!$this->tableExists()) {
            if ($rows === []) {
                return;
            }

            throw new \RuntimeException("La migration des allocations n'a pas encore été exécutée sur cette installation.");
        }

        $this->db->transactional(function () use ($packageId, $rows): void {
            $this->db->delete('USAGE_PACKAGE_ALLOWANCE', ['packageId' => $packageId]);
            foreach ($rows as $row) {
                if (!UsageAllowance::isValidUnit($row['unit']) || !UsageAllowance::isValidPeriod($row['period'])) {
                    throw new \InvalidArgumentException('Unité ou période inconnue.');
                }
                if ($row['amount'] <= 0) {
                    throw new \InvalidArgumentException('Une allocation doit être strictement positive.');
                }
                $this->db->insert('USAGE_PACKAGE_ALLOWANCE', [
                    'packageId' => $packageId,
                    'featureKey' => $row['featureKey'],
                    'reservableType' => $row['reservableType'],
                    'unit' => $row['unit'],
                    'amount' => $row['amount'],
                    'period' => $row['period'],
                    'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
                ]);
            }
        });
    }

    public function delete(int $packageId, int $allowanceId): void
    {
        if (!$this->tableExists()) {
            return;
        }

        // Scoped by package, like every other delete in this editor: an id from
        // another package's form must not reach across.
        $this->db->executeStatement(
            'DELETE FROM USAGE_PACKAGE_ALLOWANCE WHERE id = :id AND packageId = :package',
            ['id' => $allowanceId, 'package' => $packageId],
        );
    }

    public function tableExists(): bool
    {
        if ($this->tableExists === null) {
            try {
                $this->db->fetchOne('SELECT COUNT(*) FROM USAGE_PACKAGE_ALLOWANCE');
                $this->tableExists = true;
            } catch (\Throwable) {
                $this->tableExists = false;
            }
        }

        return $this->tableExists;
    }
}
