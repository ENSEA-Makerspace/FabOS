<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\Badge;
use App\Entity\Place;
use App\Entity\Utilisateur;
use App\Repository\BadgeRepository;
use App\Repository\FormationRepository;
use App\Repository\MachineBadgeRepository;
use App\Repository\PlaceRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Service\PlaceBadges;
use App\Training\LearnerJourney;
use Doctrine\DBAL\Connection;

/**
 * « Un badge, vu par qui le détient » (proposition du 2026-10-01, planche
 * `lms-training-certificate-badge.png`). Lecture seule, rien de recalculé :
 * - les machines : `MachineBadgeRepository::machinesOpenedBy()` (la relation qui
 *   décide de l'accès à chaque scan) ;
 * - les pièces : `PLACE_BADGE` lu dans le sens badge → pièces (`PlaceBadges` ne
 *   sait que pièce → badges, et reste le seul à l'écrire) ;
 * - le parcours : `LearnerJourney` pour chaque formation qui donne ce badge.
 */
final class BadgeHeld
{
    public function __construct(
        private readonly BadgeRepository $badges,
        private readonly MachineBadgeRepository $machineBadges,
        private readonly UtilisateurBadgeRepository $held,
        private readonly FormationRepository $formations,
        private readonly PlaceRepository $places,
        private readonly PlaceBadges $placeBadges,
        private readonly LearnerJourney $journey,
        private readonly Connection $db,
    ) {
    }

    /**
     * @return array{badge: Badge, owned: bool, obtainedAt: ?\DateTimeImmutable, machines: list<\App\Entity\Machine>, places: list<Place>, trainings: list<array{formation: \App\Entity\Formation, journey: array<string, mixed>}>}|null
     */
    public function for(?int $badgeId, ?Utilisateur $user): ?array
    {
        $badge = $badgeId !== null ? $this->badges->find($badgeId) : $this->firstOpeningSomething();
        if (!$badge instanceof Badge || $badge->getId() === null) {
            return null;
        }

        $record = $user !== null ? $this->held->findOneBy(['utilisateur' => $user, 'badge' => $badge]) : null;

        $trainings = [];
        foreach ($this->formations->findBy(['badge' => $badge], ['titre' => 'ASC']) as $formation) {
            $trainings[] = ['formation' => $formation, 'journey' => $this->journey->of($formation, $user)];
        }

        return [
            'badge' => $badge,
            'owned' => $record !== null,
            'obtainedAt' => $record?->getDateObtention(),
            'machines' => $this->machineBadges->machinesOpenedBy($badge->getId()),
            'places' => $this->placesOpenedBy($badge),
            'trainings' => $trainings,
        ];
    }

    private function firstOpeningSomething(): ?Badge
    {
        foreach ($this->badges->findBy(['archivedAt' => null], ['nom' => 'ASC']) as $badge) {
            if ($badge->getId() !== null && $this->machineBadges->machinesOpenedBy($badge->getId()) !== []) {
                return $badge;
            }
        }

        return null;
    }

    /** @return list<Place> */
    private function placesOpenedBy(Badge $badge): array
    {
        if (!$this->placeBadges->isReady()) {
            return [];
        }
        $ids = array_map('intval', $this->db->fetchFirstColumn('SELECT placeId FROM PLACE_BADGE WHERE badgeId = ?', [$badge->getId()]));
        $places = [];
        foreach ($ids as $id) {
            $place = $this->places->find($id);
            if ($place instanceof Place && !$place->isArchived()) {
                $places[] = $place;
            }
        }
        usort($places, static fn (Place $a, Place $b): int => strcmp($a->getNom(), $b->getNom()));

        return $places;
    }
}
