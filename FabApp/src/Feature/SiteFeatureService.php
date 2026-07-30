<?php

namespace App\Feature;

use App\Portal\PortalContext;
use App\Reservation\ReservableType;
use Doctrine\DBAL\Connection;

/**
 * On/off state for site features, stored in the SITE_MODULE table.
 *
 * A feature with no row (or if the table doesn't exist yet) is treated as
 * ENABLED, so the site keeps working even before the table is created — and so a
 * feature added by a later release arrives on rather than silently disabled.
 *
 * Rows are scoped by portal: portalId 0 is the site-wide state, a portal's own row
 * overrides it for that portal only (see PortalContext). With no portal resolved —
 * the normal case today — only the global rows are read and written.
 *
 * The table and its keys keep the old `SITE_MODULE` / "module" names: renaming
 * them would need a migration for no functional gain. Everything an operator
 * reads says *feature*.
 */
final class SiteFeatureService
{
    /** @var array<string, bool>|null */
    private ?array $cache = null;

    public function __construct(
        private readonly Connection $db,
        private readonly PortalContext $portals,
        private readonly SiteFeatureRegistry $registry,
    ) {
    }

    /** @return array<string, bool> */
    public function all(): array
    {
        if ($this->cache !== null) {
            return $this->cache;
        }

        $state = array_fill_keys($this->registry->keys(), true);

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

        // An add-on is off whenever its parent is, whatever its own row says — so
        // switching a parent back on restores its add-ons as they were left.
        foreach ($this->registry->all() as $feature) {
            if ($feature->isAddon() && ($state[$feature->parent] ?? false) === false) {
                $state[$feature->key] = false;
            }
        }

        return $this->cache = $state;
    }

    public function isEnabled(string $key): bool
    {
        return $this->all()[$key] ?? true;
    }

    public function setEnabled(string $key, bool $enabled): void
    {
        if (!$this->registry->has($key)) {
            return;
        }

        $this->db->executeStatement(
            'INSERT INTO SITE_MODULE (moduleKey, portalId, enabled) VALUES (:k, :p, :e) ON DUPLICATE KEY UPDATE enabled = :e',
            ['k' => $key, 'p' => $this->portals->scopeId(), 'e' => $enabled ? 1 : 0],
        );

        $this->cache = null;
    }

    /** Whether anything at all is drawn on the shared calendar grid. */
    public function hasCalendarLayer(): bool
    {
        foreach ($this->registry->calendarLayers() as $key) {
            if ($this->isEnabled($key)) {
                return true;
            }
        }

        return false;
    }

    /** Whether new bookings of this kind are accepted at all. */
    public function allowsReservable(ReservableType $type): bool
    {
        $feature = $this->registry->featureForReservable($type);

        return $feature === null || $this->isEnabled($feature->key);
    }

    /**
     * State grouped for display, in catalogue order, add-ons excluded — the screen
     * nests those under their parent.
     *
     * @return array<string, array<string, bool>>
     */
    public function groupedState(): array
    {
        $state = $this->all();
        $groups = [];

        foreach ($this->registry->roots() as $key => $feature) {
            $groups[$feature->group][$key] = $state[$key] ?? true;
        }

        return $groups;
    }
}
