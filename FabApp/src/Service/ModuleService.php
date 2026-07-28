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
    public const MODULES = ['machines', 'leaderboard', 'projects', 'badges', 'formations', 'lab_pages', 'places', 'events', 'staff', 'trainers', 'materials', 'loans', 'maintenance'];

    // 'emails' was removed here: mail is kernel infrastructure, not a feature.
    // See Mailer::isOperational(). Its leftover SITE_MODULE row is ignored by
    // all() rather than adopted, so it no longer surfaces as a dead switch.

    /**
     * The modules that contribute a bookable layer to the shared calendar.
     *
     * The calendar page is a *surface* over these, not a feature of its own, so
     * it stands down when none of them is on rather than rendering an empty
     * grid. Kept here so the route gate, the header and the footer all ask the
     * same question of the same list.
     *
     * The user resource layer (bookable people) is deliberately absent: it has
     * no module today, and it lives on its own pages rather than on this grid.
     *
     * @var string[]
     */
    public const RESOURCE_MODULES = ['machines', 'places'];

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
                $key = (string) $row['moduleKey'];
                // Rows for retired keys are ignored rather than adopted. `emails`
                // left one behind when mail became kernel, and adopting it put a
                // live-looking switch on the admin screen that controlled nothing.
                if (array_key_exists($key, $state)) {
                    $state[$key] = (bool) $row['enabled'];
                }
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

    /** Whether anything at all can be booked on the shared calendar. */
    public function hasResourceLayer(): bool
    {
        foreach (self::RESOURCE_MODULES as $key) {
            if ($this->isEnabled($key)) {
                return true;
            }
        }

        return false;
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
