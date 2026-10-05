<?php

declare(strict_types=1);

namespace App\Service;

use Doctrine\DBAL\Connection;

/**
 * S205 — les signalements de panne (`MACHINE_REPORT`).
 *
 * Une personne SANS compte scanne le QR collé sur la machine et décrit ce qui
 * ne va pas ; l'équipe le résout, le rouvre ou le supprime. ⚠️ Un signalement ne
 * change JAMAIS l'état de la machine : c'est l'équipe qui décide (feuille de
 * route S205).
 *
 * ⚠️ Fail-safe (modèle : `PlaceBadges`) : tant que la migration manque,
 * `isReady()` est faux, aucune lecture ne lève et rien ne s'affiche.
 *
 * Horodatages écrits en UTC ; les lignes rendues portent des
 * `DateTimeImmutable` UTC, à afficher par `|lab_date`.
 */
final class MachineReports
{
    public const OPEN = 'open';
    public const RESOLVED = 'resolved';

    /** Longueurs acceptées — la page est publique, on borne tout. */
    public const DESCRIPTION_MIN = 5;
    public const DESCRIPTION_MAX = 1000;
    public const CONTACT_MAX = 190;
    public const NOTE_MAX = 1000;

    private ?bool $ready = null;

    public function __construct(private readonly Connection $db)
    {
    }

    public function isReady(): bool
    {
        if ($this->ready === null) {
            try {
                $this->db->fetchOne('SELECT 1 FROM MACHINE_REPORT LIMIT 1');
                $this->ready = true;
            } catch (\Throwable) {
                $this->ready = false;
            }
        }

        return $this->ready;
    }

    /** @return int|null l'identifiant créé, ou null si la table manque */
    public function create(int $machineId, string $description, ?string $photo, ?string $contact, ?string $ipHash): ?int
    {
        if (!$this->isReady()) {
            return null;
        }
        $this->db->insert('MACHINE_REPORT', [
            'machineId' => $machineId,
            'description' => mb_substr(trim($description), 0, self::DESCRIPTION_MAX),
            'photo' => $photo,
            'contact' => $contact !== null && trim($contact) !== '' ? mb_substr(trim($contact), 0, self::CONTACT_MAX) : null,
            'status' => self::OPEN,
            'ipHash' => $ipHash,
            'createdAt' => gmdate('Y-m-d H:i:s'),
        ]);

        return (int) $this->db->lastInsertId();
    }

    /** Signalements récents d'une même empreinte — la limite de débit de la page publique. */
    public function countSince(\DateTimeImmutable $since, ?string $ipHash = null): int
    {
        if (!$this->isReady()) {
            return 0;
        }
        $utc = $since->setTimezone(new \DateTimeZone('UTC'))->format('Y-m-d H:i:s');

        return $ipHash === null
            ? (int) $this->db->fetchOne('SELECT COUNT(*) FROM MACHINE_REPORT WHERE createdAt >= ?', [$utc])
            : (int) $this->db->fetchOne('SELECT COUNT(*) FROM MACHINE_REPORT WHERE ipHash = ? AND createdAt >= ?', [$ipHash, $utc]);
    }

    /** @return array<string, mixed>|null */
    public function find(int $id): ?array
    {
        if (!$this->isReady()) {
            return null;
        }
        $row = $this->db->fetchAssociative(
            'SELECT r.*, m.nom AS machineName FROM MACHINE_REPORT r LEFT JOIN MACHINE m ON m.id = r.machineId WHERE r.id = ?',
            [$id],
        );

        return $row === false ? null : $this->hydrate($row);
    }

    /**
     * @param 'open'|'resolved'|null $status null = tous
     *
     * @return list<array<string, mixed>> les plus récents d'abord
     */
    public function all(?string $status = null, ?int $machineId = null, int $limit = 500): array
    {
        if (!$this->isReady()) {
            return [];
        }
        $where = [];
        $params = [];
        if ($status !== null) {
            $where[] = 'r.status = ?';
            $params[] = $status;
        }
        if ($machineId !== null) {
            $where[] = 'r.machineId = ?';
            $params[] = $machineId;
        }
        $sql = 'SELECT r.*, m.nom AS machineName FROM MACHINE_REPORT r LEFT JOIN MACHINE m ON m.id = r.machineId'
            . ($where === [] ? '' : ' WHERE ' . implode(' AND ', $where))
            . ' ORDER BY r.createdAt DESC, r.id DESC LIMIT ' . max(1, $limit);

        return array_map($this->hydrate(...), $this->db->fetchAllAssociative($sql, $params));
    }

    /** @return list<array<string, mixed>> */
    public function openForMachine(int $machineId): array
    {
        return $this->all(self::OPEN, $machineId, 20);
    }

    /** @return array{open: int, resolved: int} */
    public function counts(): array
    {
        $counts = ['open' => 0, 'resolved' => 0];
        if (!$this->isReady()) {
            return $counts;
        }
        foreach ($this->db->fetchAllAssociative('SELECT status, COUNT(*) AS n FROM MACHINE_REPORT GROUP BY status') as $row) {
            if (isset($counts[$row['status']])) {
                $counts[$row['status']] = (int) $row['n'];
            }
        }

        return $counts;
    }

    public function resolve(int $id, ?string $note, ?int $userId): bool
    {
        if (!$this->isReady()) {
            return false;
        }
        $note = $note !== null && trim($note) !== '' ? mb_substr(trim($note), 0, self::NOTE_MAX) : null;

        return $this->db->executeStatement(
            'UPDATE MACHINE_REPORT SET status = ?, resolutionNote = ?, resolvedAt = ?, resolvedBy = ? WHERE id = ? AND status = ?',
            [self::RESOLVED, $note, gmdate('Y-m-d H:i:s'), $userId, $id, self::OPEN],
        ) > 0;
    }

    public function reopen(int $id): bool
    {
        if (!$this->isReady()) {
            return false;
        }

        return $this->db->executeStatement(
            'UPDATE MACHINE_REPORT SET status = ?, resolutionNote = NULL, resolvedAt = NULL, resolvedBy = NULL WHERE id = ? AND status = ?',
            [self::OPEN, $id, self::RESOLVED],
        ) > 0;
    }

    /** @return string|null le nom de fichier de la photo supprimée avec la ligne (à effacer du disque) */
    public function delete(int $id): ?string
    {
        $row = $this->find($id);
        if ($row === null) {
            return null;
        }
        $this->db->delete('MACHINE_REPORT', ['id' => $id]);

        return $row['photo'] ?: null;
    }

    /**
     * @param array<string, mixed> $row
     *
     * @return array<string, mixed>
     */
    private function hydrate(array $row): array
    {
        $utc = new \DateTimeZone('UTC');
        $row['id'] = (int) $row['id'];
        $row['machineId'] = (int) $row['machineId'];
        $row['createdAt'] = new \DateTimeImmutable((string) $row['createdAt'], $utc);
        $row['resolvedAt'] = !empty($row['resolvedAt']) ? new \DateTimeImmutable((string) $row['resolvedAt'], $utc) : null;
        $row['isOpen'] = $row['status'] === self::OPEN;
        unset($row['ipHash']);

        return $row;
    }
}
