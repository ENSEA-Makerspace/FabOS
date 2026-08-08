<?php

namespace App\Feature;

/**
 * What a feature leans on, and what quietly stops working when the companion is off.
 *
 * **Warn, never block.** Every combination expressible here is a legitimate way to
 * run a place — an ungated community workshop is a real thing — so the software's
 * job is to say what the operator is giving up, not to refuse the choice.
 *
 * Each relationship in the registry was checked against the source rather than
 * assumed. One of them contradicts what the plan expected; see the note on
 * `machines` in SiteFeatureRegistry.
 */
final class FeatureAdvice
{
    public function __construct(
        private readonly SiteFeatureRegistry $registry,
        private readonly SiteFeatureService $features,
    ) {
    }

    /**
     * Live warnings: a feature that is ON, leaning on a companion that is OFF.
     *
     * A recommendation made by a feature that is itself switched off is not a
     * warning about anything, because nobody is relying on it.
     *
     * @return list<array{feature: string, featureLabel: string, missing: string, missingLabel: string, cost: string}>
     */
    public function warnings(): array
    {
        $warnings = [];

        foreach ($this->registry->all() as $key => $feature) {
            if (!$this->features->isEnabled($key)) {
                continue;
            }

            foreach ($feature->recommends as $advice) {
                $companion = $this->registry->get($advice['feature']);
                if ($companion === null || $this->features->isEnabled($companion->key)) {
                    continue;
                }

                $warnings[] = [
                    'feature' => $key,
                    'featureLabel' => $feature->label,
                    'missing' => $companion->key,
                    'missingLabel' => $companion->label,
                    'cost' => $advice['cost'],
                ];
            }
        }

        return $warnings;
    }

    /**
     * Grouped by the feature that raised them, so the screen can print each one on
     * the card whose switch caused it rather than in a list at the top.
     *
     * @return array<string, list<array{missing: string, missingLabel: string, cost: string}>>
     */
    public function byFeature(): array
    {
        $grouped = [];
        foreach ($this->warnings() as $warning) {
            $grouped[$warning['feature']][] = [
                'missing' => $warning['missing'],
                'missingLabel' => $warning['missingLabel'],
                'cost' => $warning['cost'],
            ];
        }

        return $grouped;
    }
}
