<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\Formation;
use App\Entity\Progression;
use App\Entity\Utilisateur;
use App\Repository\ProgressionRepository;
use App\Repository\SectionRepository;
use App\Training\LearnerJourney;
use App\Training\PracticalQueue;

/**
 * Le dossier d'UNE validation pratique (proposition `validation-pratique`,
 * 2026-10-01), pour une personne × une formation.
 *
 * ⚠️ LECTURE SEULE. La file est celle de `PracticalQueue::pending()`, le
 * parcours celui de `LearnerJourney::of()` : rien n'est recalculé ici.
 * ⚠️ « À observer » : AUCUN modèle de checklist n'existe. Source, dans l'ordre :
 * le champ texte `objectifs` de la formation (pratique, sinon parente), coupé en
 * lignes ; à défaut les SECTIONS de la formation ; à défaut rien.
 */
final class PracticalValidation
{
    public function __construct(
        private readonly PracticalQueue $queue,
        private readonly LearnerJourney $journey,
        private readonly SectionRepository $sections,
        private readonly ProgressionRepository $progressions,
    ) {
    }

    /**
     * @return array{
     *   dossier: int, count: int, fallback: bool,
     *   user: Utilisateur, formation: Formation, physical: Formation,
     *   since: ?\DateTimeImmutable, journey: array<string, mixed>,
     *   points: list<string>, pointsSource: string,
     * }|null
     */
    public function build(int $dossier): ?array
    {
        $pending = $this->queue->pending();
        $fallback = false;

        if ($pending !== []) {
            $index = max(0, min($dossier, \count($pending) - 1));
            $row = $pending[$index];
            $user = $row['user'];
            $formation = $row['formation'];
            $physical = $row['physical'];
            $since = $row['since'];
        } else {
            // Personne en attente : première progression non complétée d'une
            // formation qui exige une validation pratique (démo seulement).
            $found = null;
            foreach ($this->progressions->findAll() as $progression) {
                $f = $progression->getFormation();
                $u = $progression->getUtilisateur();
                if ($progression instanceof Progression && !$progression->isCompleted() && $f !== null && $u !== null && $f->getRequiresPractical()) {
                    $found = $progression;
                    break;
                }
            }
            if ($found === null) {
                return null;
            }
            $fallback = true;
            $index = 0;
            $user = $found->getUtilisateur();
            $formation = $found->getFormation();
            $physical = $formation;
            $since = $found->getDateEnd() ?? $found->getDateDebut();
        }

        $points = $this->lines($physical->getObjectifs());
        $source = 'objectifs';
        if ($points === [] && $physical !== $formation) {
            $points = $this->lines($formation->getObjectifs());
        }
        if ($points === []) {
            $source = 'sections';
            foreach ($this->sections->findJourneySections($formation) as $section) {
                $points[] = $section->getTitre();
            }
        }
        if ($points === []) {
            $source = 'aucun';
        }

        return [
            'dossier' => $index,
            'count' => $pending === [] ? 1 : \count($pending),
            'fallback' => $fallback,
            'user' => $user,
            'formation' => $formation,
            'physical' => $physical,
            'since' => $since,
            'journey' => $this->journey->of($formation, $user),
            'points' => $points,
            'pointsSource' => $source,
        ];
    }

    /** @return list<string> */
    private function lines(?string $text): array
    {
        if ($text === null || trim($text) === '') {
            return [];
        }
        // Stocké une ligne par ligne, ou joint par « . » (voir l'éditeur).
        $parts = preg_split('/\R|\.\s+/u', $text) ?: [];

        return array_values(array_filter(array_map(static fn (string $p): string => trim($p, " \t.-•"), $parts), static fn (string $p): bool => $p !== ''));
    }
}
