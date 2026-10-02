<?php

declare(strict_types=1);

namespace App\Page;

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
     * Le dossier d'une personne × une formation EN ATTENTE (la vraie page).
     * `null` si cette personne n'attend pas de validation pour cette formation
     * (déjà validée, théorie non finie, formation inconnue) : la page rend 404.
     *
     * @return array<string, mixed>|null même forme que `build()`, `fallback` toujours faux
     */
    public function for(int $userId, int $formationId): ?array
    {
        $pending = $this->queue->pending();
        foreach ($pending as $index => $row) {
            if ($row['user']->getId() === $userId && $row['formation']->getId() === $formationId) {
                return $this->compose($index, \count($pending), false, $row['user'], $row['formation'], $row['physical'], $row['since']);
            }
        }

        return null;
    }

    /** @return array<string, mixed> */
    private function compose(int $index, int $count, bool $fallback, Utilisateur $user, Formation $formation, Formation $physical, ?\DateTimeImmutable $since): array
    {
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
            'count' => $count,
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
