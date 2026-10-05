<?php

declare(strict_types=1);

namespace App\Service;

use Doctrine\DBAL\Connection;

/**
 * S208 — le registre d'avertissements par usager (`USER_WARNING`) et la liste de
 * motifs que le lab règle lui-même (`WARNING_REASON`).
 *
 * C'est un REGISTRE : aucune ligne ici ne change un droit, et la personne
 * concernée ne la voit pas. Un avertissement n'est jamais effacé, il est LEVÉ.
 *
 * ⚠️ Fail-safe (modèle : `PlaceBadges`) : tant que la migration manque,
 * `isReady()` est faux et la fonctionnalité est inerte. Horodatages en UTC.
 */
final class UserWarnings
{
    private ?bool $ready = null;

    public function __construct(private readonly Connection $db)
    {
    }

    public function isReady(): bool
    {
        if ($this->ready === null) {
            try {
                $this->db->fetchOne('SELECT 1 FROM USER_WARNING LIMIT 1');
                $this->db->fetchOne('SELECT 1 FROM WARNING_REASON LIMIT 1');
                $this->ready = true;
            } catch (\Throwable) {
                $this->ready = false;
            }
        }

        return $this->ready;
    }

    /** @return list<array{id: int, label: string, position: int, active: bool}> */
    public function reasons(bool $activeOnly = false): array
    {
        if (!$this->isReady()) {
            return [];
        }
        $rows = $this->db->fetchAllAssociative('SELECT id, label, position, active FROM WARNING_REASON' . ($activeOnly ? ' WHERE active = 1' : '') . ' ORDER BY position, label');

        return array_map(static fn (array $r): array => [
            'id' => (int) $r['id'], 'label' => (string) $r['label'], 'position' => (int) $r['position'], 'active' => (bool) $r['active'],
        ], $rows);
    }

    public function addReason(string $label): bool
    {
        $label = mb_substr(trim($label), 0, 120);
        if (!$this->isReady() || $label === '') {
            return false;
        }
        $exists = (bool) $this->db->fetchOne('SELECT 1 FROM WARNING_REASON WHERE label = ?', [$label]);
        if ($exists) {
            return false;
        }
        $position = (int) $this->db->fetchOne('SELECT COALESCE(MAX(position), 0) + 1 FROM WARNING_REASON');
        $this->db->insert('WARNING_REASON', ['label' => $label, 'position' => $position, 'active' => 1]);

        return true;
    }

    public function setReasonActive(int $id, bool $active): bool
    {
        return $this->isReady() && $this->db->update('WARNING_REASON', ['active' => $active ? 1 : 0], ['id' => $id]) > 0;
    }

    /** @return int|null l'id du nouvel avertissement, ou null (motif inconnu, registre indisponible) */
    public function add(int $userId, int $reasonId, string $note, ?int $issuedBy): ?int
    {
        if (!$this->isReady() || !$this->db->fetchOne('SELECT 1 FROM WARNING_REASON WHERE id = ? AND active = 1', [$reasonId])) {
            return null;
        }
        $this->db->insert('USER_WARNING', [
            'userId' => $userId,
            'reasonId' => $reasonId,
            'note' => mb_substr(trim($note), 0, 2000) ?: null,
            'issuedBy' => $issuedBy,
            'createdAt' => (new \DateTimeImmutable('now', new \DateTimeZone('UTC')))->format('Y-m-d H:i:s'),
        ]);

        return (int) $this->db->lastInsertId();
    }

    /** Lève un avertissement encore actif. Rend faux s'il n'existe pas ou est déjà levé. */
    public function lift(int $id): bool
    {
        if (!$this->isReady()) {
            return false;
        }

        return $this->db->executeStatement(
            'UPDATE USER_WARNING SET liftedAt = ? WHERE id = ? AND liftedAt IS NULL',
            [(new \DateTimeImmutable('now', new \DateTimeZone('UTC')))->format('Y-m-d H:i:s'), $id],
        ) > 0;
    }

    /** @return int|null le compte visé par cet avertissement */
    public function userIdOf(int $id): ?int
    {
        $v = $this->isReady() ? $this->db->fetchOne('SELECT userId FROM USER_WARNING WHERE id = ?', [$id]) : false;

        return $v === false ? null : (int) $v;
    }

    /**
     * Les avertissements d'UNE personne, les plus récents d'abord.
     *
     * @return list<array<string, mixed>>
     */
    public function forUser(int $userId): array
    {
        return $this->rows('w.userId = ?', [$userId]);
    }

    /**
     * Tous les avertissements, avec la personne visée.
     *
     * @param 'active'|'lifted'|'' $state
     *
     * @return list<array<string, mixed>>
     */
    public function all(string $state = ''): array
    {
        return $this->rows(match ($state) {
            'active' => 'w.liftedAt IS NULL',
            'lifted' => 'w.liftedAt IS NOT NULL',
            default => '1 = 1',
        }, []);
    }

    /** @return array{active: int, lifted: int} */
    public function counts(): array
    {
        if (!$this->isReady()) {
            return ['active' => 0, 'lifted' => 0];
        }

        return [
            'active' => (int) $this->db->fetchOne('SELECT COUNT(*) FROM USER_WARNING WHERE liftedAt IS NULL'),
            'lifted' => (int) $this->db->fetchOne('SELECT COUNT(*) FROM USER_WARNING WHERE liftedAt IS NOT NULL'),
        ];
    }

    /**
     * @param list<mixed> $params
     *
     * @return list<array<string, mixed>>
     */
    private function rows(string $where, array $params): array
    {
        if (!$this->isReady()) {
            return [];
        }
        $rows = $this->db->fetchAllAssociative(
            'SELECT w.id, w.userId, w.note, w.createdAt, w.liftedAt, r.label AS reason,
                    u.firstName AS uFirst, u.lastName AS uLast, u.username AS uName,
                    a.firstName AS aFirst, a.lastName AS aLast, a.username AS aName
             FROM USER_WARNING w
             LEFT JOIN WARNING_REASON r ON r.id = w.reasonId
             LEFT JOIN UTILISATEUR u ON u.id = w.userId
             LEFT JOIN UTILISATEUR a ON a.id = w.issuedBy
             WHERE ' . $where . ' ORDER BY w.createdAt DESC, w.id DESC',
            $params,
        );
        $utc = new \DateTimeZone('UTC');
        $name = static fn (?string $first, ?string $last, ?string $user): string => trim(($first ?? '') . ' ' . ($last ?? '')) ?: (string) $user;

        return array_map(static fn (array $r): array => [
            'id' => (int) $r['id'],
            'userId' => (int) $r['userId'],
            'user' => $name($r['uFirst'], $r['uLast'], $r['uName']),
            'reason' => (string) ($r['reason'] ?? ''),
            'note' => (string) ($r['note'] ?? ''),
            'by' => $r['aName'] === null ? '' : $name($r['aFirst'], $r['aLast'], $r['aName']),
            'at' => new \DateTimeImmutable((string) $r['createdAt'], $utc),
            'liftedAt' => $r['liftedAt'] === null ? null : new \DateTimeImmutable((string) $r['liftedAt'], $utc),
        ], $rows);
    }
}
