<?php

namespace App\Portal;

use Doctrine\DBAL\Connection;

/**
 * Reads the PORTAL table over raw DBAL, like the config stores it serves
 * (SiteFeatureService, SiteSettingService) — no entity, so nothing hydrates it by
 * accident. Fail-safe throughout: a missing table resolves to "no portals",
 * which lands every caller on the global scope, i.e. today's behaviour.
 */
final class PortalRepository
{
    /** @var array<int, Portal|null> */
    private array $byId = [];

    public function __construct(private readonly Connection $db)
    {
    }

    public function findByHostname(string $hostname): ?Portal
    {
        return $this->fetchOnePortal(
            'SELECT id, slug, name, hostname, isDefault FROM PORTAL WHERE hostname = :h LIMIT 1',
            ['h' => $hostname],
        );
    }

    public function findBySlug(string $slug): ?Portal
    {
        return $this->fetchOnePortal(
            'SELECT id, slug, name, hostname, isDefault FROM PORTAL WHERE slug = :s LIMIT 1',
            ['s' => $slug],
        );
    }

    public function find(int $id): ?Portal
    {
        return $this->byId[$id] ??= $this->fetchOnePortal(
            'SELECT id, slug, name, hostname, isDefault FROM PORTAL WHERE id = :i',
            ['i' => $id],
        );
    }

    /**
     * The portal standing in for the global scope. Prefers the flagged default and
     * falls back to the oldest row, so a database whose flag was cleared by hand
     * still resolves to something rather than to null.
     */
    public function findDefault(): ?Portal
    {
        return $this->fetchOnePortal('SELECT id, slug, name, hostname, isDefault FROM PORTAL ORDER BY isDefault DESC, id ASC LIMIT 1');
    }

    /** @return Portal[] */
    public function all(): array
    {
        try {
            $rows = $this->db->fetchAllAssociative('SELECT id, slug, name, hostname, isDefault FROM PORTAL ORDER BY isDefault DESC, name ASC');
        } catch (\Throwable) {
            return [];
        }

        return array_map(Portal::fromRow(...), $rows);
    }

    /** @param array<string, mixed> $params */
    private function fetchOnePortal(string $sql, array $params = []): ?Portal
    {
        try {
            $row = $this->db->fetchAssociative($sql, $params);
        } catch (\Throwable) {
            return null;
        }

        return is_array($row) ? Portal::fromRow($row) : null;
    }
}
