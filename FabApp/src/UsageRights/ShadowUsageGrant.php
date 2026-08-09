<?php

declare(strict_types=1);

namespace App\UsageRights;

/**
 * In-memory S110 grant shape. S111 persists it; this object cannot authorize
 * a route and therefore cannot accidentally change the live security model.
 */
final readonly class ShadowUsageGrant
{
    /** @param array<string, list<string>> $scopes */
    public function __construct(
        public string $feature,
        public ?string $section,
        public UsageGrantAction $action,
        public array $scopes = [],
    ) {}

    public function covers(string $feature, ?string $section, UsageGrantAction $action, array $requiredScopes = []): bool
    {
        if ($this->feature !== $feature || $this->section !== $section || !$this->action->covers($action)) {
            return false;
        }

        foreach ($requiredScopes as $dimension => $value) {
            $allowed = $this->scopes[$dimension] ?? [];
            if ($allowed !== [] && !in_array((string) $value, $allowed, true)) {
                return false;
            }
        }

        return true;
    }
}
