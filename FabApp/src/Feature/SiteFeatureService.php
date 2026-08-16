<?php

namespace App\Feature;

use App\Reservation\ReservableType;
use Doctrine\DBAL\Connection;

/**
 * On/off state for site features, stored in the SITE_MODULE table.
 *
 * A feature with no row (or if the table doesn't exist yet) is treated as
 * ENABLED, so the site keeps working even before the table is created — and so a
 * feature added by a later release arrives on rather than silently disabled.
 *
 * One row per feature controls the whole instance.
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
            $rows = $this->db->fetchAllAssociative(
                'SELECT moduleKey, enabled FROM SITE_MODULE',
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

    /**
     * Answer every question as if these features had these values, run `$fn`, then
     * put the real state back.
     *
     * ⚠️ **This is how the features screen knows what a switch costs** (S132). It
     * used to *describe* the effect of turning something off, in prose, in a
     * catalogue kept beside the one that gates the routes — so the description was
     * free to drift from the behaviour, and it had. The screen builds the
     * navigation twice instead, once with everything on and once with one thing
     * off, and shows the difference. Nothing is written down twice, so nothing can
     * disagree.
     *
     * Read-only and scoped: the override lives in the resolved-state cache and is
     * restored in a `finally`, so a throw inside `$fn` cannot leave the request
     * answering questions about an installation that does not exist. It never
     * touches `SITE_MODULE` — `setEnabled()` is still the only writer.
     *
     * @param array<string, bool> $overrides
     */
    public function simulate(array $overrides, callable $fn): mixed
    {
        $saved = $this->cache;

        try {
            $state = $this->all();
            foreach ($overrides as $key => $enabled) {
                if (array_key_exists($key, $state)) {
                    $state[$key] = $enabled;
                }
            }

            // The add-on rule again, and it has to be re-applied rather than
            // inherited: simulating "machines off" must take maintenance with it,
            // exactly as saving that choice would.
            foreach ($this->registry->all() as $feature) {
                if ($feature->isAddon() && ($state[$feature->parent] ?? false) === false) {
                    $state[$feature->key] = false;
                }
            }

            $this->cache = $state;

            return $fn();
        } finally {
            $this->cache = $saved;
        }
    }

    public function setEnabled(string $key, bool $enabled): void
    {
        if (!$this->registry->has($key)) {
            return;
        }

        $this->db->executeStatement(
            'INSERT INTO SITE_MODULE (moduleKey, enabled) VALUES (:k, :e) ON DUPLICATE KEY UPDATE enabled = :e',
            ['k' => $key, 'e' => $enabled ? 1 : 0],
        );

        $this->cache = null;
    }

    /** Whether anything at all is drawn on the shared calendar grid. */
    /**
     * Whether a **navigable surface** gated on `$feature` should be offered.
     *
     * ⚠️ `bookings` is not a registry key and never was — bookings are
     * polymorphic since S8–S10, so booking screens gate on "any bookable layer
     * at all" and not on `machines`, or a spaces-only deployment would lose its
     * reservations screen. That rule lived privately in `NavBuilder`, which was
     * fine while the navigation was the only thing asking; `SiteSearch` asks the
     * same question now, and `isEnabled('bookings')` **fails open** — it would
     * have answered a confident `true` on an install with no bookable layer at
     * all and offered a link to an empty screen. One word, one meaning, one
     * place, as the note in `NavBuilder` already said.
     */
    public function allowsSurface(string $feature): bool
    {
        return $feature === 'bookings'
            ? $this->hasCalendarLayer()
            : $this->isEnabled($feature);
    }

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
