<?php

declare(strict_types=1);

namespace App\Service;

use App\Entity\Utilisateur;
use App\Schedule\ScheduleResolver;
use Doctrine\DBAL\Connection;

/**
 * S207 — Check-in à paliers (`CHECKIN`, `CHECKIN_REASON`).
 *
 * 🔴 **Rien à planifier : la fin d'une visite se CALCULE à la lecture.** Une visite
 * ouverte (`endedAt` nul) dont le lieu a fermé est finie à l'heure de fermeture de
 * son jour ; si ce jour n'a pas d'horaires, finie dès que le jour est passé. Aucune
 * tâche ne tourne la nuit, donc aucune ne peut tomber en panne.
 *
 * ⚠️ Fail-safe : tant que la migration manque, `isReady()` est faux et tout se tait.
 * ⚠️ Horodatages en UTC en base ; `labDay()` convertit pour décider « quel jour ».
 */
final class Checkins
{
    public const SOURCES = ['kiosk', 'rfid', 'self'];
    /** La courte liste du palier 2 : des CLÉS (traduites à l'écran), jamais des phrases. */
    public const VISITOR_TYPES = ['student', 'staff', 'visitor', 'other'];

    private ?bool $ready = null;

    public function __construct(
        private readonly Connection $db,
        private readonly ScheduleResolver $schedule,
        private readonly SiteSettingService $siteSettings,
    ) {
    }

    public function isReady(): bool
    {
        if ($this->ready === null) {
            try {
                $this->db->fetchOne('SELECT 1 FROM CHECKIN LIMIT 1');
                $this->db->fetchOne('SELECT 1 FROM CHECKIN_REASON LIMIT 1');
                $this->ready = true;
            } catch (\Throwable) {
                $this->ready = false;
            }
        }

        return $this->ready;
    }

    // ── Présence ────────────────────────────────────────────────────────────

    /** La visite OUVERTE de cette personne (encore là), ou null. */
    public function openFor(Utilisateur $user, ?\DateTimeImmutable $now = null): ?array
    {
        if (!$this->isReady() || $user->getId() === null) {
            return null;
        }
        $now ??= new \DateTimeImmutable('now', new \DateTimeZone('UTC'));
        foreach ($this->db->fetchAllAssociative('SELECT * FROM CHECKIN WHERE userId = ? AND endedAt IS NULL ORDER BY startedAt DESC', [$user->getId()]) as $row) {
            if ($this->effectiveEnd($row, $now) === null) {
                return $this->decorate($row);
            }
        }

        return null;
    }

    /**
     * J'arrive. Idempotent : déjà présent, rien n'est ajouté (on peut taper deux
     * fois, ou badger puis ouvrir la page). Une note ou un motif donnés alors sont
     * ajoutés à la visite en cours.
     *
     * @return int l'id de la visite
     */
    public function arrive(Utilisateur $user, string $source, ?string $reason = null, ?string $note = null, ?\DateTimeImmutable $now = null): int
    {
        $now ??= new \DateTimeImmutable('now', new \DateTimeZone('UTC'));
        $source = \in_array($source, self::SOURCES, true) ? $source : 'self';
        $this->closeStale($user, $now);
        $open = $this->openFor($user, $now);
        if ($open !== null) {
            $this->annotate((int) $open['id'], $reason, $note);

            return (int) $open['id'];
        }
        $this->db->insert('CHECKIN', [
            'userId' => $user->getId(),
            'reason' => self::clip($reason, 120),
            'projectNote' => self::clip($note, 500),
            'source' => $source,
            'startedAt' => $now->format('Y-m-d H:i:s'),
        ]);

        return (int) $this->db->lastInsertId();
    }

    /** Je pars. Rend vrai s'il y avait une visite à fermer. */
    public function leave(Utilisateur $user, ?\DateTimeImmutable $now = null): bool
    {
        $now ??= new \DateTimeImmutable('now', new \DateTimeZone('UTC'));
        $open = $this->openFor($user, $now);
        if ($open === null) {
            return false;
        }
        $this->db->update('CHECKIN', ['endedAt' => $now->format('Y-m-d H:i:s')], ['id' => $open['id']]);

        return true;
    }

    /** Un visiteur sans compte (palier 2) : il n'a pas de « Je pars », sa visite finit à la fermeture. */
    public function arriveVisitor(string $name, string $type, ?string $reason = null, ?string $note = null, ?\DateTimeImmutable $now = null): int
    {
        $now ??= new \DateTimeImmutable('now', new \DateTimeZone('UTC'));
        $this->db->insert('CHECKIN', [
            'visitorName' => self::clip($name, 120),
            'visitorType' => \in_array($type, self::VISITOR_TYPES, true) ? $type : 'other',
            'reason' => self::clip($reason, 120),
            'projectNote' => self::clip($note, 500),
            'source' => 'kiosk',
            'startedAt' => $now->format('Y-m-d H:i:s'),
        ]);

        return (int) $this->db->lastInsertId();
    }

    /** Limite de débit du formulaire public : combien de visiteurs ces dernières 60 secondes. */
    public function recentVisitorCount(?\DateTimeImmutable $now = null): int
    {
        $now ??= new \DateTimeImmutable('now', new \DateTimeZone('UTC'));

        return (int) $this->db->fetchOne('SELECT COUNT(*) FROM CHECKIN WHERE userId IS NULL AND startedAt >= ?', [$now->modify('-60 seconds')->format('Y-m-d H:i:s')]);
    }

    private function annotate(int $id, ?string $reason, ?string $note): void
    {
        $set = array_filter(['reason' => self::clip($reason, 120), 'projectNote' => self::clip($note, 500)], static fn ($v) => $v !== null);
        if ($set !== []) {
            $this->db->update('CHECKIN', $set, ['id' => $id]);
        }
    }

    /** Écrit la fin calculée des visites ouvertes périmées d'une personne (avant d'en ouvrir une neuve). */
    private function closeStale(Utilisateur $user, \DateTimeImmutable $now): void
    {
        foreach ($this->db->fetchAllAssociative('SELECT * FROM CHECKIN WHERE userId = ? AND endedAt IS NULL', [$user->getId()]) as $row) {
            $end = $this->effectiveEnd($row, $now);
            if ($end !== null) {
                $this->db->update('CHECKIN', ['endedAt' => $end->format('Y-m-d H:i:s')], ['id' => $row['id']]);
            }
        }
    }

    // ── La fin d'une visite, calculée ───────────────────────────────────────

    /**
     * L'instant (UTC) où la visite est finie, ou null si la personne est encore là.
     *
     * @param array<string, mixed> $row
     */
    public function effectiveEnd(array $row, ?\DateTimeImmutable $now = null): ?\DateTimeImmutable
    {
        $utc = new \DateTimeZone('UTC');
        if (!empty($row['endedAt'])) {
            return new \DateTimeImmutable((string) $row['endedAt'], $utc);
        }
        $now ??= new \DateTimeImmutable('now', $utc);
        $start = new \DateTimeImmutable((string) $row['startedAt'], $utc);
        $tz = new \DateTimeZone($this->siteSettings->getTimezone());
        $localStart = $start->setTimezone($tz);
        $closing = null;
        foreach ($this->schedule->openIntervalsFor($row['venueId'] !== null ? (int) $row['venueId'] : null, $localStart) as $interval) {
            $closing = max($closing ?? 0, (int) $interval['end']);
        }
        if ($closing !== null) {
            $closeAt = $localStart->setTime(0, 0)->modify('+' . $closing . ' minutes')->setTimezone($utc);
            // 🔴 2026-10-06 — arrivé APRÈS la fermeture (séance tardive, équipe) : la
            // visite ne finit pas à l'instant où elle commence — on retombe sur la
            // règle « sans horaires » : elle dure jusqu'à la fin de la journée.
            if ($closeAt > $start) {
                return $now >= $closeAt ? $closeAt : null;
            }
        }
        // Pas d'horaires ce jour-là, ou arrivée après la fermeture : la visite finit
        // avec le jour.
        $dayEnd = $localStart->setTime(23, 59, 59)->setTimezone($utc);

        return $now > $dayEnd ? $dayEnd : null;
    }

    // ── Lectures de l'équipe ────────────────────────────────────────────────

    /** @return list<array<string, mixed>> les personnes présentes maintenant, la plus récente d'abord */
    public function present(?\DateTimeImmutable $now = null): array
    {
        if (!$this->isReady()) {
            return [];
        }
        $now ??= new \DateTimeImmutable('now', new \DateTimeZone('UTC'));
        $rows = [];
        foreach ($this->db->fetchAllAssociative($this->selectVisits() . ' WHERE c.endedAt IS NULL ORDER BY c.startedAt DESC') as $row) {
            if ($this->effectiveEnd($row, $now) === null) {
                $rows[] = $this->decorate($row);
            }
        }

        return $rows;
    }

    /**
     * Les visites depuis `$from` (UTC), la plus récente d'abord.
     *
     * @return list<array<string, mixed>>
     */
    public function since(\DateTimeImmutable $from, ?\DateTimeImmutable $now = null): array
    {
        if (!$this->isReady()) {
            return [];
        }
        $now ??= new \DateTimeImmutable('now', new \DateTimeZone('UTC'));
        $rows = [];
        foreach ($this->db->fetchAllAssociative($this->selectVisits() . ' WHERE c.startedAt >= ? ORDER BY c.startedAt DESC', [$from->format('Y-m-d H:i:s')]) as $row) {
            $row['end'] = $this->effectiveEnd($row, $now);
            $rows[] = $this->decorate($row);
        }

        return $rows;
    }

    /** Début (UTC) d'une période : 'today' = minuit du lab, sinon N jours glissants. */
    public function periodStart(string $period, ?\DateTimeImmutable $now = null): \DateTimeImmutable
    {
        $now ??= new \DateTimeImmutable('now', new \DateTimeZone('UTC'));
        $local = $now->setTimezone(new \DateTimeZone($this->siteSettings->getTimezone()))->setTime(0, 0);
        $days = match ($period) {
            '7' => 6,
            '30' => 29,
            default => 0,
        };

        return $local->modify('-' . $days . ' days')->setTimezone(new \DateTimeZone('UTC'));
    }

    public function countSince(\DateTimeImmutable $from): int
    {
        return $this->isReady() ? (int) $this->db->fetchOne('SELECT COUNT(*) FROM CHECKIN WHERE startedAt >= ?', [$from->format('Y-m-d H:i:s')]) : 0;
    }

    private function selectVisits(): string
    {
        return 'SELECT c.*, TRIM(CONCAT(COALESCE(u.firstName, \'\'), \' \', COALESCE(u.lastName, \'\'))) AS memberName, u.username AS memberUsername FROM CHECKIN c LEFT JOIN UTILISATEUR u ON u.id = c.userId';
    }

    /**
     * @param array<string, mixed> $row
     *
     * @return array<string, mixed>
     */
    private function decorate(array $row): array
    {
        $utc = new \DateTimeZone('UTC');
        $row['start'] = new \DateTimeImmutable((string) $row['startedAt'], $utc);
        $row['name'] = !empty($row['userId']) ? (trim((string) ($row['memberName'] ?? '')) ?: (string) ($row['memberUsername'] ?? '')) : (string) ($row['visitorName'] ?? '');

        return $row;
    }

    // ── Les motifs (palier 3) ───────────────────────────────────────────────

    /** @return list<array{id: int, label: string, position: int, active: bool}> */
    public function reasons(bool $activeOnly = false): array
    {
        if (!$this->isReady()) {
            return [];
        }
        $rows = $this->db->fetchAllAssociative('SELECT id, label, `position`, active FROM CHECKIN_REASON' . ($activeOnly ? ' WHERE active = 1' : '') . ' ORDER BY `position`, id');

        return array_map(static fn (array $r): array => ['id' => (int) $r['id'], 'label' => (string) $r['label'], 'position' => (int) $r['position'], 'active' => (bool) $r['active']], $rows);
    }

    /** Le libellé d'un motif choisi, s'il est actif ; null sinon (jamais de texte libre venu du client). */
    public function reasonLabel(int $id): ?string
    {
        $label = $this->isReady() ? $this->db->fetchOne('SELECT label FROM CHECKIN_REASON WHERE id = ? AND active = 1', [$id]) : false;

        return \is_string($label) ? $label : null;
    }

    public function addReason(string $label): bool
    {
        $label = trim($label);
        if ($label === '' || !$this->isReady()) {
            return false;
        }
        $this->db->insert('CHECKIN_REASON', ['label' => self::clip($label, 120), 'position' => (int) $this->db->fetchOne('SELECT COALESCE(MAX(`position`), 0) + 1 FROM CHECKIN_REASON'), 'active' => 1]);

        return true;
    }

    public function renameReason(int $id, string $label): bool
    {
        $label = trim($label);

        return $label !== '' && $this->db->update('CHECKIN_REASON', ['label' => self::clip($label, 120)], ['id' => $id]) >= 0;
    }

    public function setReasonActive(int $id, bool $active): void
    {
        $this->db->update('CHECKIN_REASON', ['active' => $active ? 1 : 0], ['id' => $id]);
    }

    /** Échange la place d'un motif avec son voisin ; `$delta` vaut -1 (monter) ou +1 (descendre). */
    public function moveReason(int $id, int $delta): void
    {
        $ids = array_column($this->reasons(), 'id');
        $i = array_search($id, $ids, true);
        $j = $i === false ? false : $i + ($delta < 0 ? -1 : 1);
        if ($i === false || !isset($ids[$j])) {
            return;
        }
        [$ids[$i], $ids[$j]] = [$ids[$j], $ids[$i]];
        foreach ($ids as $position => $reasonId) {
            $this->db->update('CHECKIN_REASON', ['position' => $position + 1], ['id' => $reasonId]);
        }
    }

    private static function clip(?string $value, int $max): ?string
    {
        $value = trim((string) $value);

        return $value === '' ? null : mb_substr($value, 0, $max);
    }
}
