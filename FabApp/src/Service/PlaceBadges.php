<?php

declare(strict_types=1);

namespace App\Service;

use App\Entity\Badge;
use App\Entity\Place;
use App\Entity\Utilisateur;
use Doctrine\DBAL\Connection;

/**
 * S204 — les badges qui ouvrent une pièce (`PLACE_BADGE`).
 *
 * Même logique qu'aux machines : détenir UN des badges exigés suffit (le
 * lecteur d'une machine fait pareil). Une pièce sans badge exigé se comporte
 * comme avant S204.
 *
 * ⚠️ Fail-safe : tant que la migration manque, `isReady()` est faux, aucune
 * pièce n'exige rien et l'écran ne propose pas le réglage.
 */
final class PlaceBadges
{
    private ?bool $ready = null;

    public function __construct(private readonly Connection $db)
    {
    }

    public function isReady(): bool
    {
        if ($this->ready === null) {
            try {
                $this->db->fetchOne('SELECT 1 FROM PLACE_BADGE LIMIT 1');
                $this->ready = true;
            } catch (\Throwable) {
                $this->ready = false;
            }
        }

        return $this->ready;
    }

    /** @return list<int> les badges exigés (actifs seulement) */
    public function requiredIds(Place $place): array
    {
        if (!$this->isReady() || $place->getId() === null) {
            return [];
        }

        return array_map('intval', $this->db->fetchFirstColumn(
            'SELECT pb.badgeId FROM PLACE_BADGE pb JOIN BADGE b ON b.id = pb.badgeId WHERE pb.placeId = ? AND b.archivedAt IS NULL ORDER BY b.nom',
            [$place->getId()],
        ));
    }

    /** @return list<string> leurs noms, pour les écrans et les refus */
    public function requiredNames(Place $place): array
    {
        if ($this->requiredIds($place) === []) {
            return [];
        }

        return $this->db->fetchFirstColumn(
            'SELECT b.nom FROM PLACE_BADGE pb JOIN BADGE b ON b.id = pb.badgeId WHERE pb.placeId = ? AND b.archivedAt IS NULL ORDER BY b.nom',
            [$place->getId()],
        );
    }

    /** La personne détient-elle l'un des badges exigés ? (vrai si la pièce n'en exige aucun) */
    public function qualifies(Place $place, Utilisateur $user): bool
    {
        $required = $this->requiredIds($place);
        if ($required === []) {
            return true;
        }

        return (bool) $this->db->fetchOne(
            'SELECT 1 FROM UTILISATEUR_BADGE WHERE utilisateurId = ? AND badgeId IN (?) LIMIT 1',
            [$user->getId(), $required],
            [\Doctrine\DBAL\ParameterType::INTEGER, \Doctrine\DBAL\ArrayParameterType::INTEGER],
        );
    }

    /** @param list<Badge> $badges remplace la liste */
    public function set(Place $place, array $badges): void
    {
        if (!$this->isReady() || $place->getId() === null) {
            return;
        }
        $this->db->transactional(function () use ($place, $badges): void {
            $this->db->executeStatement('DELETE FROM PLACE_BADGE WHERE placeId = ?', [$place->getId()]);
            foreach ($badges as $badge) {
                if ($badge->getId() !== null) {
                    $this->db->executeStatement('INSERT IGNORE INTO PLACE_BADGE (placeId, badgeId) VALUES (?, ?)', [$place->getId(), $badge->getId()]);
                }
            }
        });
    }
}
