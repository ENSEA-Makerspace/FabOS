<?php

declare(strict_types=1);

namespace App\Training;

use App\Entity\Badge;
use App\Entity\Utilisateur;
use Doctrine\DBAL\Connection;

/**
 * S202 — donner et retirer un badge à la main : QUI, QUAND, POURQUOI.
 *
 * Le badge DÉTENU est la ligne de `UTILISATEUR_BADGE` — c'est elle que lisent
 * le lecteur, la réservation, « Mes badges ». `BADGE_GRANT` en est le journal :
 * d'où vient un badge donné à la main, et qui en a retiré un, quand, pourquoi.
 *
 * 🔴 **Décision opérateur : un badge n'est jamais effacé sans trace, et un
 * retrait n'est jamais « réactivé ».** Retirer écrit (ou ferme) une ligne qui
 * reste ; redonner en écrit une neuve.
 * 🔴 **Un badge retiré ne revient pas tout seul** : sans ce verrou, la
 * formation validée le redonnait à la sauvegarde suivante d'une progression
 * (`ProgressionBadgeSubscriber`). Seul « Attribuer » le rend.
 *
 * ⚠️ Fail-safe : tant que la migration S202 manque, `isReady()` est faux, les
 * écrans ne montrent rien et le reste de l'application ne change pas.
 */
final class BadgeGrants
{
    public const ORIGIN_MANUAL = 'manual';
    public const ORIGIN_FORMATION = 'formation';

    private ?bool $ready = null;

    public function __construct(private readonly Connection $db)
    {
    }

    public function isReady(): bool
    {
        if ($this->ready === null) {
            try {
                $this->db->fetchOne('SELECT 1 FROM BADGE_GRANT LIMIT 1');
                $this->ready = true;
            } catch (\Throwable) {
                $this->ready = false;
            }
        }

        return $this->ready;
    }

    public function holds(Utilisateur $user, Badge $badge): bool
    {
        return (bool) $this->db->fetchOne(
            'SELECT 1 FROM UTILISATEUR_BADGE WHERE utilisateurId = ? AND badgeId = ?',
            [$user->getId(), $badge->getId()],
        );
    }

    /** @return bool false si la personne le détenait déjà (rien n'est écrit) */
    public function grant(Utilisateur $user, Badge $badge, Utilisateur $actor, string $reason): bool
    {
        if (!$this->isReady() || $this->holds($user, $badge)) {
            return false;
        }

        $this->db->transactional(function () use ($user, $badge, $actor, $reason): void {
            $this->db->executeStatement(
                'INSERT INTO UTILISATEUR_BADGE (utilisateurId, badgeId, dateObtention) VALUES (?, ?, NOW())',
                [$user->getId(), $badge->getId()],
            );
            $this->db->executeStatement(
                'INSERT INTO BADGE_GRANT (userId, badgeId, origin, grantedById, grantedAt, reason) VALUES (?, ?, ?, ?, NOW(), ?)',
                [$user->getId(), $badge->getId(), self::ORIGIN_MANUAL, $actor->getId(), $reason],
            );
        });

        return true;
    }

    /** @return bool false si la personne ne le détenait pas */
    public function revoke(Utilisateur $user, Badge $badge, Utilisateur $actor, string $reason): bool
    {
        if (!$this->isReady() || !$this->holds($user, $badge)) {
            return false;
        }

        $this->db->transactional(function () use ($user, $badge, $actor, $reason): void {
            $open = $this->openRow((int) $user->getId(), (int) $badge->getId());
            if ($open === null) {
                // Un badge venu d'une formation (ou d'avant S202) n'a pas de
                // ligne : on l'écrit au retrait, avec sa date d'obtention.
                $this->db->executeStatement(
                    'INSERT INTO BADGE_GRANT (userId, badgeId, origin, grantedAt)'
                    . ' SELECT utilisateurId, badgeId, ?, dateObtention FROM UTILISATEUR_BADGE WHERE utilisateurId = ? AND badgeId = ?',
                    [self::ORIGIN_FORMATION, $user->getId(), $badge->getId()],
                );
                $open = (int) $this->db->lastInsertId();
            }
            $this->db->executeStatement(
                'UPDATE BADGE_GRANT SET revokedById = ?, revokedAt = NOW(), revokeReason = ? WHERE id = ?',
                [$actor->getId(), $reason, $open],
            );
            $this->db->executeStatement(
                'DELETE FROM UTILISATEUR_BADGE WHERE utilisateurId = ? AND badgeId = ?',
                [$user->getId(), $badge->getId()],
            );
        });

        return true;
    }

    /**
     * Le verrou : la dernière ligne de ce badge pour cette personne est un
     * retrait, et personne ne l'a redonné depuis.
     */
    public function isRevoked(int $userId, int $badgeId): bool
    {
        if (!$this->isReady()) {
            return false;
        }
        $last = $this->db->fetchOne(
            'SELECT revokedAt FROM BADGE_GRANT WHERE userId = ? AND badgeId = ? ORDER BY id DESC LIMIT 1',
            [$userId, $badgeId],
        );

        return $last !== false && $last !== null;
    }

    /**
     * Les badges DÉTENUS que quelqu'un a donnés à la main, par badge.
     *
     * @return array<int, array{by: ?string, at: \DateTimeImmutable, reason: ?string}>
     */
    public function manualGrantsFor(Utilisateur $user): array
    {
        if (!$this->isReady()) {
            return [];
        }
        $rows = $this->db->fetchAllAssociative(
            'SELECT g.badgeId, g.grantedAt, g.reason, g.grantedById, a.firstName, a.lastName, a.username'
            . ' FROM BADGE_GRANT g LEFT JOIN UTILISATEUR a ON a.id = g.grantedById'
            . ' WHERE g.userId = ? AND g.origin = ? AND g.revokedAt IS NULL ORDER BY g.id',
            [$user->getId(), self::ORIGIN_MANUAL],
        );
        $out = [];
        foreach ($rows as $row) {
            $out[(int) $row['badgeId']] = [
                'by' => self::name($row),
                'at' => new \DateTimeImmutable((string) $row['grantedAt']),
                'reason' => $row['reason'] !== null ? (string) $row['reason'] : null,
            ];
        }

        return $out;
    }

    /**
     * Les retraits, du plus récent au plus ancien — ce que la fiche admin garde
     * sous les yeux pour qu'un retrait ne soit jamais une disparition.
     *
     * @return list<array{badgeId: int, badge: string, by: ?string, at: \DateTimeImmutable, reason: ?string}>
     */
    public function revocationsFor(Utilisateur $user): array
    {
        if (!$this->isReady()) {
            return [];
        }
        $rows = $this->db->fetchAllAssociative(
            'SELECT g.badgeId, b.nom AS badge, g.revokedAt, g.revokeReason, a.firstName, a.lastName, a.username'
            . ' FROM BADGE_GRANT g JOIN BADGE b ON b.id = g.badgeId LEFT JOIN UTILISATEUR a ON a.id = g.revokedById'
            . ' WHERE g.userId = ? AND g.revokedAt IS NOT NULL ORDER BY g.revokedAt DESC, g.id DESC',
            [$user->getId()],
        );

        return array_map(static fn (array $row): array => [
            'badgeId' => (int) $row['badgeId'],
            'badge' => (string) $row['badge'],
            'by' => self::name($row),
            'at' => new \DateTimeImmutable((string) $row['revokedAt']),
            'reason' => $row['revokeReason'] !== null ? (string) $row['revokeReason'] : null,
        ], $rows);
    }

    /**
     * L'anonymisation : les lignes restent (les statistiques de badges), les
     * MOTIFS partent — un motif écrit à la main peut nommer la personne.
     */
    public function forget(int $userId): void
    {
        if ($this->isReady()) {
            $this->db->executeStatement('UPDATE BADGE_GRANT SET reason = NULL, revokeReason = NULL WHERE userId = ?', [$userId]);
        }
    }

    private function openRow(int $userId, int $badgeId): ?int
    {
        $id = $this->db->fetchOne(
            'SELECT id FROM BADGE_GRANT WHERE userId = ? AND badgeId = ? AND revokedAt IS NULL ORDER BY id DESC LIMIT 1',
            [$userId, $badgeId],
        );

        return $id === false ? null : (int) $id;
    }

    /** @param array<string, mixed> $row */
    private static function name(array $row): ?string
    {
        $name = trim(((string) ($row['firstName'] ?? '')) . ' ' . ((string) ($row['lastName'] ?? '')));

        return $name !== '' ? $name : ($row['username'] !== null ? (string) $row['username'] : null);
    }
}
