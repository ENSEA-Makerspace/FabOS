<?php

namespace App\Twig;

use App\Feature\FeatureWorkspaceRegistry;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

final class WorkspaceExtension extends AbstractExtension
{
    public function __construct(private readonly FeatureWorkspaceRegistry $registry)
    {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('feature_workspace', $this->registry->forRoute(...)),
            new TwigFunction('workspace_filter_chips', $this->filterChips(...)),
        ];
    }

    /**
     * The shell displays only known scalar list parameters. This keeps the
     * shared component bounded even when a crafted URL contains arrays or a
     * large number of unknown keys; repositories remain responsible for their
     * own query validation.
     *
     * @param array<string, mixed> $query
     * @return list<array{label_key: string, value: string, remove_query: array<string, string>}>
     */
    public function filterChips(array $query): array
    {
        $filterKeys = ['q', 'statut', 'niveau', 'badge', 'category', 'role', 'manager', 'department', 'dateFrom', 'dateTo', 'reservableType', 'type'];
        $preserved = [];
        foreach ($query as $key => $value) {
            if (is_string($key) && is_scalar($value) && in_array($key, [...$filterKeys, 'location', 'page'], true)) {
                $preserved[$key] = mb_substr(trim((string) $value), 0, 80);
            }
        }

        $chips = [];
        foreach ($filterKeys as $key) {
            if (!isset($preserved[$key]) || $preserved[$key] === '') {
                continue;
            }
            $removeQuery = $preserved;
            unset($removeQuery[$key], $removeQuery['page']);
            $chips[] = ['label_key' => 'workspace.filter.' . $key, 'value' => $preserved[$key], 'remove_query' => $removeQuery];
            if (count($chips) === 6) {
                break;
            }
        }

        return $chips;
    }
}
