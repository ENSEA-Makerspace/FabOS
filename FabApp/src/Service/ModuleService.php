<?php

namespace App\Service;

use App\Portal\PortalContext;
use App\Reservation\ReservableType;
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
    /**
     * What kind of thing each module is. Three different answers had been hiding
     * behind the one word "module", and conflating them is what let turning off
     * a *directory page* threaten the staff desk.
     *
     *  - **resource** — a bookable kind. Owns a `ReservableType` and decides
     *    whether new bookings of that kind are accepted at all
     *    (`ReservationService::book()`).
     *  - **activity** — a feature domain with its own pages and data.
     *  - **directory** — display only. A directory module owns *a page and a
     *    menu entry*, nothing else. The people it lists, their roles and their
     *    authorisation are **kernel** and are not switchable by anything here.
     *
     * @var array<string, string[]>
     */
    public const LAYERS = [
        'resource' => ['machines', 'places', 'person_booking'],
        'activity' => ['events', 'formations', 'badges', 'projects', 'leaderboard', 'lab_pages', 'materials', 'loans', 'maintenance'],
        'directory' => ['staff', 'trainers'],
    ];

    /** @var string[] */
    public const MODULES = [
        'machines', 'places', 'person_booking',
        'events', 'formations', 'badges', 'projects', 'leaderboard', 'lab_pages', 'materials', 'loans', 'maintenance',
        'staff', 'trainers',
    ];

    // 'emails' was removed here: mail is kernel infrastructure, not a feature.
    // See Mailer::isOperational(). Its leftover SITE_MODULE row is ignored by
    // all() rather than adopted, so it no longer surfaces as a dead switch.

    /**
     * The resource modules that draw a layer on the *shared calendar grid*.
     *
     * Narrower than `LAYERS['resource']` on purpose, and named for the grid
     * rather than for booking: `person_booking` is every bit a resource layer,
     * but people are booked from their own pages and no column is drawn for
     * them. Listing it here would make `/calendrier` come back as an empty grid
     * for a deployment that books only people — the exact failure this constant
     * exists to prevent. Add it the day people appear on the grid, not before.
     *
     * @var string[]
     */
    public const CALENDAR_LAYERS = ['machines', 'places'];

    /**
     * Which module owns each bookable kind, keyed by `ReservableType::value`.
     * Read by the booking chokepoint and by the booking reminder scanner, so a
     * disabled layer stops taking bookings *and* stops mailing about them.
     *
     * @var array<string, string>
     */
    public const MODULE_BY_RESERVABLE = [
        ReservableType::Machine->value => 'machines',
        ReservableType::Place->value => 'places',
        ReservableType::User->value => 'person_booking',
    ];

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

    /** Whether anything at all is drawn on the shared calendar grid. */
    public function hasCalendarLayer(): bool
    {
        foreach (self::CALENDAR_LAYERS as $key) {
            if ($this->isEnabled($key)) {
                return true;
            }
        }

        return false;
    }

    /** Whether new bookings of this kind are accepted at all. */
    public function allowsReservable(ReservableType $type): bool
    {
        $module = self::MODULE_BY_RESERVABLE[$type->value] ?? null;

        return $module === null || $this->isEnabled($module);
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
