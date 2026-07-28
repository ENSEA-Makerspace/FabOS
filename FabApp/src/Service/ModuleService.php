<?php

namespace App\Service;

use App\Portal\PortalContext;
use Doctrine\DBAL\Connection;

/**
 * On/off state for optional site modules, stored in the SITE_MODULE table.
 * A module with no row (or if the table doesn't exist yet) is treated as ENABLED,
 * so the site keeps working even before the table is created.
 *
 * Rows are scoped by portal: portalId 0 is the site-wide state, a portal's own row
 * overrides it for that portal only (see PortalContext). With no portal resolved —
 * the normal case today — only the global rows are read and written.
 */
final class ModuleService
{
    /** @var string[] */
    public const MODULES = ['leaderboard', 'projects', 'badges', 'formations', 'lab_pages', 'places', 'events', 'staff', 'trainers', 'materials', 'loans', 'maintenance'];

    // 'emails' was removed here: mail is kernel infrastructure, not a feature.
    // See Mailer::isOperational(). A leftover SITE_MODULE row for it is inert,
    // since nothing reads that key any more.

    /** @var array<string, bool>|null */
    private ?array $cache = null;

    public function __construct(
        private readonly Connection $db,
        private readonly PortalContext $portals,
    ) {
    }

    /** @return array<string, bool> */
    public function all(): array
    {
        if ($this->cache !== null) {
            return $this->cache;
        }

        $state = array_fill_keys(self::MODULES, true);

        try {
            // Ascending portalId applies the global rows first, then lets the current
            // portal's rows overwrite the keys it actually overrides.
            $rows = $this->db->fetchAllAssociative(
                'SELECT moduleKey, enabled FROM SITE_MODULE WHERE portalId IN (:g, :p) ORDER BY portalId ASC',
                ['g' => PortalContext::GLOBAL_SCOPE, 'p' => $this->portals->scopeId()],
            );
            foreach ($rows as $row) {
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
            'INSERT INTO SITE_MODULE (moduleKey, portalId, enabled) VALUES (:k, :p, :e) ON DUPLICATE KEY UPDATE enabled = :e',
            ['k' => $key, 'p' => $this->portals->scopeId(), 'e' => $enabled ? 1 : 0],
        );

        $this->cache = null;
    }
}
