<?php

namespace App\Portal;

/**
 * A branded "front door" onto the one shared FabOS dataset — e.g. a teachers'
 * portal on its own hostname, with its own module subset and settings.
 *
 * Portals are lenses, not tenants: the data stays shared, only configuration is
 * scoped. The row flagged $isDefault is the site as it has always been — it owns
 * no overrides of its own, it *is* the global scope (see PortalContext::scopeId).
 */
final readonly class Portal
{
    public function __construct(
        public int $id,
        public string $slug,
        public string $name,
        public ?string $hostname,
        public bool $isDefault,
    ) {
    }

    /** @param array<string, mixed> $row */
    public static function fromRow(array $row): self
    {
        return new self(
            (int) $row['id'],
            (string) $row['slug'],
            (string) $row['name'],
            isset($row['hostname']) && $row['hostname'] !== '' ? (string) $row['hostname'] : null,
            (bool) $row['isDefault'],
        );
    }
}
