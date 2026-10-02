<?php

declare(strict_types=1);

namespace App\Page;

use App\Entity\Formation;
use App\Entity\Utilisateur;
use App\Repository\FormationRepository;
use App\Repository\SectionRepository;
use App\Service\GuidedTrainingService;

/**
 * « Lire une section » (proposition du 2026-10-01, planche `lms-course-module.png`) :
 * UNE étape du parcours guidé en page de lecture. Rien n'est recalculé : les
 * étapes, leur contenu, leur quiz et leur état viennent de
 * `GuidedTrainingService::buildJourney()`, celui de la fiche formation. Seule
 * la durée de lecture est calculée ici (aucun champ ne la porte).
 */
final class ModuleReading
{
    private const WORDS_PER_MINUTE = 200;

    public function __construct(
        private readonly FormationRepository $formations,
        private readonly SectionRepository $sections,
        private readonly GuidedTrainingService $guided,
    ) {
    }

    /** @return array<string, mixed>|null null si aucune formation n'a de parcours */
    public function build(?Utilisateur $user, ?int $formationId, int $position): ?array
    {
        $formation = $formationId !== null ? $this->formations->find($formationId) : $this->richest();
        if (!$formation instanceof Formation) {
            return null;
        }

        $journey = $this->guided->buildJourney($formation, $user);
        $items = $journey['items'];
        if ($items === []) {
            return null;
        }

        $index = max(1, min($position, \count($items))) - 1;
        $item = $items[$index];
        $content = $item['content'];

        $words = str_word_count(strip_tags(implode(' ', array_merge(
            [$content['intro']],
            $content['objectives'],
            array_map(static fn (array $s): string => $s['title'] . ' ' . $s['text'], $content['steps']),
            array_map(static fn (array $c): string => $c['title'] . ' ' . $c['text'], $content['callouts']),
        ))), 0, 'àâçéèêëîïôûùüÿœÀÉÈÊ');

        return [
            'formation' => $formation,
            'journey' => $journey,
            'item' => $item,
            'index' => $index,
            'total' => \count($items),
            'previous' => $index > 0 ? $index : null,
            'next' => $index < \count($items) - 1 ? $index + 2 : null,
            'minutes' => max(1, (int) ceil($words / self::WORDS_PER_MINUTE)),
            'user' => $user,
        ];
    }

    /** La formation visible qui a le plus d'étapes (défaut de la démo). */
    private function richest(): ?Formation
    {
        $best = null;
        $bestCount = 0;
        foreach ($this->formations->findVisible(['id' => 'DESC']) as $formation) {
            $count = \count($this->sections->findJourneySections($formation));
            if ($count > $bestCount) {
                $best = $formation;
                $bestCount = $count;
            }
        }

        return $best;
    }
}
