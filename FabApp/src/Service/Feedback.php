<?php

declare(strict_types=1);

namespace App\Service;

use Doctrine\DBAL\Connection;

/**
 * S210 — les retours des usagers (`FEEDBACK`) : bug, gêne d'ergonomie, idée.
 *
 * ⚠️ Fail-safe : tant que la migration manque, `isReady()` est faux, l'entrée
 * de l'en-tête n'apparaît pas et l'écran d'équipe répond 404.
 * Horodatages en UTC (affichés `|lab_date`).
 */
final class Feedback
{
    public const KINDS = ['bug', 'ux', 'idea'];

    /** Au plus N envois par personne et par fenêtre (limite de débit, sans service de plus). */
    public const RATE_MAX = 5;
    public const RATE_MINUTES = 10;

    private ?bool $ready = null;

    public function __construct(private readonly Connection $db)
    {
    }

    public function isReady(): bool
    {
        if ($this->ready === null) {
            try {
                $this->db->fetchOne('SELECT 1 FROM FEEDBACK LIMIT 1');
                $this->ready = true;
            } catch (\Throwable) {
                $this->ready = false;
            }
        }

        return $this->ready;
    }

    /** Trop d'envois récents pour cette personne ? */
    public function tooMany(int $userId): bool
    {
        return (int) $this->db->fetchOne(
            'SELECT COUNT(*) FROM FEEDBACK WHERE userId = ? AND createdAt > ?',
            [$userId, (new \DateTimeImmutable('-' . self::RATE_MINUTES . ' minutes', new \DateTimeZone('UTC')))->format('Y-m-d H:i:s')],
        ) >= self::RATE_MAX;
    }

    public function add(int $userId, string $kind, string $message, string $pageUrl, ?string $userAgent, ?string $appVersion): int
    {
        $this->db->insert('FEEDBACK', [
            'userId' => $userId,
            'kind' => $kind,
            'message' => $message,
            'pageUrl' => $pageUrl,
            'userAgent' => $userAgent,
            'appVersion' => $appVersion,
            'status' => 'open',
            'createdAt' => (new \DateTimeImmutable('now', new \DateTimeZone('UTC')))->format('Y-m-d H:i:s'),
        ]);

        return (int) $this->db->lastInsertId();
    }

    /** @return array<string, mixed>|null */
    public function find(int $id): ?array
    {
        $row = $this->db->fetchAssociative('SELECT * FROM FEEDBACK WHERE id = ?', [$id]);

        return $row === false ? null : $row;
    }

    /**
     * @return list<array<string, mixed>> du plus récent au plus ancien, avec le nom du compte
     */
    public function list(string $status, string $kind = ''): array
    {
        $sql = "SELECT f.*, TRIM(CONCAT(COALESCE(u.firstName, ''), ' ', COALESCE(u.lastName, ''))) AS authorName, u.email AS authorEmail
                FROM FEEDBACK f LEFT JOIN UTILISATEUR u ON u.id = f.userId WHERE f.status = ?";
        $params = [$status];
        if (in_array($kind, self::KINDS, true)) {
            $sql .= ' AND f.kind = ?';
            $params[] = $kind;
        }

        return $this->db->fetchAllAssociative($sql . ' ORDER BY f.createdAt DESC, f.id DESC LIMIT 300', $params);
    }

    /** @return array{open: int, done: int, kinds: array<string, int>} les ouverts par type, et les traités */
    public function counts(): array
    {
        $out = ['open' => 0, 'done' => 0, 'kinds' => array_fill_keys(self::KINDS, 0)];
        foreach ($this->db->fetchAllAssociative('SELECT status, kind, COUNT(*) AS n FROM FEEDBACK GROUP BY status, kind') as $row) {
            if ($row['status'] === 'open') {
                $out['open'] += (int) $row['n'];
                $out['kinds'][$row['kind']] = (int) $row['n'];
            } else {
                $out['done'] += (int) $row['n'];
            }
        }

        return $out;
    }

    public function setDone(int $id, bool $done): void
    {
        $this->db->update('FEEDBACK', [
            'status' => $done ? 'done' : 'open',
            'doneAt' => $done ? (new \DateTimeImmutable('now', new \DateTimeZone('UTC')))->format('Y-m-d H:i:s') : null,
        ], ['id' => $id]);
    }

    /** @return list<int> l'équipe qui reçoit chaque retour : administrateurs et équipe, comptes actifs */
    public function teamIds(): array
    {
        return array_map('intval', $this->db->fetchFirstColumn(
            "SELECT DISTINCT m.userId FROM USER_GROUP_MEMBER m
             JOIN USER_GROUP g ON g.id = m.groupId
             JOIN UTILISATEUR u ON u.id = m.userId
             WHERE g.groupKey IN ('admin', 'staff') AND u.statut = 'actif'",
        ));
    }
}
