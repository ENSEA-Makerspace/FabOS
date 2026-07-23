<?php

namespace App\Service;

use Doctrine\DBAL\Connection;

/**
 * On/off state for optional site modules, stored in the SITE_MODULE table.
 * A module with no row (or if the table doesn't exist yet) is treated as ENABLED,
 * so the site keeps working even before the table is created.
 */
final class ModuleService
{
    /** @var string[] */
    public const MODULES = ['leaderboard', 'projects', 'badges', 'formations', 'lab_pages', 'places', 'events', 'staff', 'trainers', 'materials', 'loans', 'maintenance'];

    /** @var array<string, bool>|null */
    private ?array $cache = null;

    public function __construct(private readonly Connection $db)
    {
    }

    /** @return array<string, bool> */
    public function all(): array
    {
        if ($this->cache !== null) {
            return $this->cache;
        }

        $state = array_fill_keys(self::MODULES, true);

        try {
            foreach ($this->db->fetchAllAssociative('SELECT moduleKey, enabled FROM SITE_MODULE') as $row) {
                $state[(string) $row['moduleKey']] = (bool) $row['enabled'];
            }
        } catch (\Throwable) {
            // Table not created yet: fall back to "everything enabled".
        }

        return $this->cache = $state;
    }

    public function isEnabled(string $key): bool
    {
        return $this->all()[$key] ?? true;
    }

    public function setEnabled(string $key, bool $enabled): void
    {
        if (!in_array($key, self::MODULES, true)) {
            return;
        }

        $this->db->executeStatement(
            'INSERT INTO SITE_MODULE (moduleKey, enabled) VALUES (:k, :e) ON DUPLICATE KEY UPDATE enabled = :e',
            ['k' => $key, 'e' => $enabled ? 1 : 0],
        );

        $this->cache = null;
    }
}
