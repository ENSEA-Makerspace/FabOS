<?php

namespace App\Feature;

use App\Nav\NavBuilder;

/**
 * What a feature actually holds up — measured, never declared.
 *
 * **Why this exists (S132).** The features screen was built on the catalogue's
 * own three-way split — resource / activity / directory — with a prose sentence
 * per feature saying what it gives you. Two problems, and the second is the one
 * that matters:
 *
 *  1. Those three groups are a **second taxonomy** beside the workspaces the
 *     operator navigates all day. An operator who wants to know where events are
 *     managed reads "Activités" and is no wiser.
 *  2. The prose is a **second description of the behaviour**, free to drift from
 *     the gates. It had: `machines`' own note records that switching badges off
 *     does *not* re-open equipment, which is the opposite of what the plan
 *     assumed until somebody read the source.
 *
 * So nothing here is written down. The navigation is built twice — once with
 * everything on, once with one feature off — and the difference *is* the answer.
 * A destination that stops being reachable is a destination this switch removes,
 * and `NavBuilder` is the only thing that gets a say, exactly as it is for the
 * sidebar the operator is looking at while they read this.
 *
 * ⚠️ **Navigable surfaces, not permissions.** This says what disappears from the
 * menus. Route gates and voters are what actually refuse a request, and a screen
 * that vanishes here may still be reachable by address — that is `NavBuilder`'s
 * rule 3, and it is not this class's job to restate it.
 */
final class FeatureSurfaces
{
    /** @var array<string, array<string, mixed>>|null */
    private ?array $cache = null;

    public function __construct(
        private readonly SiteFeatureRegistry $registry,
        private readonly SiteFeatureService $features,
        private readonly FeatureWorkspaceRegistry $workspaces,
        private readonly NavBuilder $nav,
    ) {
    }

    /**
     * One entry per feature, in the order an operator meets the workspaces.
     *
     * @return array<string, array{
     *     feature: SiteFeature,
     *     workspace: ?string,
     *     admin: list<array{label: string, translate: bool, section: ?string}>,
     *     public: list<array{label: string, translate: bool}>,
     *     dependents: list<string>
     * }>
     */
    public function all(): array
    {
        if ($this->cache !== null) {
            return $this->cache;
        }

        $keys = $this->registry->keys();
        $allOn = array_fill_keys($keys, true);
        $baseline = $this->features->simulate($allOn, fn (): array => $this->destinations());

        $map = [];
        foreach ($this->order() as $key) {
            $feature = $this->registry->get($key);
            if ($feature === null) {
                continue;
            }

            $without = $this->features->simulate([...$allOn, $key => false], fn (): array => $this->destinations());

            $map[$key] = [
                'feature' => $feature,
                'workspace' => $this->workspaceOf($key),
                'admin' => $this->lost($baseline['admin'], $without['admin']),
                'public' => $this->lost($baseline['public'], $without['public']),
                // Add-ons go down with their parent. Naming them on the parent's
                // card is the difference between "seven screens" and "seven
                // screens and the maintenance log you forgot was in here".
                'dependents' => array_keys($this->registry->addonsOf($key)),
            ];
        }

        return $this->cache = $map;
    }

    /**
     * Features in workspace order, then whatever the workspaces do not claim.
     *
     * ⚠️ The tail is not an oversight and must not be hidden: `leaderboard`,
     * `person_booking` and the two directories are real features that own no
     * operator workspace, because there is nothing to administer about them. A
     * page that only listed workspace-owning features would silently lose four
     * switches.
     *
     * @return list<string>
     */
    private function order(): array
    {
        $ordered = [];
        foreach ($this->workspaces->all() as $workspace) {
            $gate = $workspace['featureGate'];
            if (is_string($gate) && $this->registry->has($gate) && !in_array($gate, $ordered, true)) {
                $ordered[] = $gate;
            }
        }

        foreach ($this->registry->keys() as $key) {
            if (!in_array($key, $ordered, true)) {
                $ordered[] = $key;
            }
        }

        // Add-ons are drawn nested inside their parent's card, so they are not
        // top-level rows of their own.
        return array_values(array_filter(
            $ordered,
            fn (string $key): bool => !($this->registry->get($key)?->isAddon() ?? false),
        ));
    }

    private function workspaceOf(string $key): ?string
    {
        foreach ($this->workspaces->all() as $workspace) {
            if ($workspace['featureGate'] === $key) {
                return (string) $workspace['label'];
            }
        }

        return null;
    }

    /**
     * Every navigable destination this installation currently offers, keyed by
     * address so two builds can be compared.
     *
     * @return array{admin: array<string, array<string, mixed>>, public: array<string, array<string, mixed>>}
     */
    private function destinations(): array
    {
        $admin = [];
        foreach ($this->nav->admin() as $section) {
            foreach ($section['items'] as $item) {
                $admin[$this->address($item['route'], $item['params'])] = [
                    'label' => $item['label'],
                    'translate' => true,
                    'section' => $section['label'],
                ];
            }
        }

        $public = [];
        foreach ($this->nav->header() as $entry) {
            foreach ($entry['items'] ?: [$entry] as $item) {
                $public[$this->address($item['route'], $item['params'])] = [
                    'label' => $item['label'],
                    'translate' => $item['translate'],
                ];
            }
        }

        return ['admin' => $admin, 'public' => $public];
    }

    /**
     * @param array<string, array<string, mixed>> $baseline
     * @param array<string, array<string, mixed>> $without
     *
     * @return list<array<string, mixed>>
     */
    private function lost(array $baseline, array $without): array
    {
        return array_values(array_diff_key($baseline, $without));
    }

    /** @param array<string, mixed> $params */
    private function address(string $route, array $params): string
    {
        return $route . '?' . http_build_query($params);
    }
}
