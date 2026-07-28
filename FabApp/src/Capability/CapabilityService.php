<?php

namespace App\Capability;

use App\Service\ModuleService;
use App\Service\SiteSettingService;

/**
 * Capability state, and the relationship between capabilities and modules.
 *
 * ## There is exactly one source of truth for "is this module on": SITE_MODULE
 *
 * Capabilities are the operator's *intent*, persisted alongside; toggling one
 * writes module rows. Module state is never recomputed from capabilities on read.
 *
 * The plan called for the reverse — module rows kept only as deviations from what
 * the capabilities imply. That was rejected for two reasons. First, every existing
 * install already has an explicit row for every module (the old admin form wrote
 * them all on save), so on the day this shipped every module would have read as a
 * deviation. Second, deriving on read means an untick in the Advanced panel has
 * nowhere to live, which is precisely the "I unticked it and it came back" bug the
 * plan was trying to avoid.
 *
 * What the operator sees is unchanged either way: capabilities, an Advanced panel
 * showing what they imply with the differences marked, and a reset action. A
 * "deviation" here is a *diff* between intent and reality — informative, not
 * authoritative.
 *
 * ## Empty means "as before"
 *
 * Until the capability screen is saved once, nothing is stored and nothing is
 * derived: capability state is read *back* from the current module state, so the
 * screen opens showing the install as it actually is and saving it changes
 * nothing. An install that never visits this screen behaves exactly as it did.
 *
 * ## Changes are applied as a delta, never as a wholesale overwrite
 *
 * Saving capabilities writes only the modules whose *implication* changed. A
 * module the operator has deliberately overridden in the Advanced panel therefore
 * survives unrelated capability changes, and is only overwritten when a capability
 * that actually owns it is toggled — where the operator's newer, explicit choice
 * should win.
 */
final class CapabilityService
{
    private const CONFIGURED_KEY = 'capabilities_configured';
    private const PREFIX = 'capability_';

    /** @var array<string, bool>|null */
    private ?array $cache = null;

    public function __construct(
        private readonly CapabilityRegistry $registry,
        private readonly SiteSettingService $settings,
        private readonly ModuleService $modules,
    ) {
    }

    /** Whether the capability screen has ever been saved. */
    public function isConfigured(): bool
    {
        return $this->settings->get(self::CONFIGURED_KEY) === '1';
    }

    /**
     * Effective capability state. An add-on reads as off while its parent is off,
     * whatever is stored for it — so turning a parent back on restores its add-ons
     * as the operator left them.
     *
     * @return array<string, bool>
     */
    public function all(): array
    {
        if ($this->cache !== null) {
            return $this->cache;
        }

        $configured = $this->isConfigured();
        $state = [];
        foreach ($this->registry->all() as $capability) {
            $state[$capability->key] = $configured
                ? $this->stored($capability)
                : $this->readBackFromModules($capability);
        }

        foreach ($this->registry->all() as $capability) {
            if ($capability->isAddon() && ($state[$capability->parent] ?? false) === false) {
                $state[$capability->key] = false;
            }
        }

        return $this->cache = $state;
    }

    public function isEnabled(string $key): bool
    {
        return $this->all()[$key] ?? false;
    }

    /**
     * What the enabled capabilities say each module's state should be. Every module
     * gets an answer: a module no enabled capability requires is implied *off*, not
     * merely unmentioned — otherwise turning the last capability that needed it off
     * would leave it on forever.
     *
     * @return array<string, bool>
     */
    public function impliedModules(): array
    {
        $implied = array_fill_keys(ModuleService::MODULES, false);

        foreach ($this->all() as $key => $enabled) {
            if (!$enabled) {
                continue;
            }
            foreach ($this->registry->get($key)?->requires ?? [] as $module) {
                if (array_key_exists($module, $implied)) {
                    $implied[$module] = true;
                }
            }
        }

        return $implied;
    }

    /**
     * Modules whose actual state differs from what the capabilities imply.
     *
     * @return array<string, array{implied: bool, actual: bool}>
     */
    public function deviations(): array
    {
        $actual = $this->modules->all();
        $deviations = [];

        foreach ($this->impliedModules() as $module => $implied) {
            $isOn = $actual[$module] ?? true;
            if ($isOn !== $implied) {
                $deviations[$module] = ['implied' => $implied, 'actual' => $isOn];
            }
        }

        return $deviations;
    }

    /**
     * Persist the submitted capability set and apply the change to modules.
     *
     * @param string[] $checked capability keys the operator ticked
     *
     * @return array<string, bool> the modules actually rewritten, for the flash message
     */
    public function save(array $checked): array
    {
        $before = $this->impliedModules();

        foreach ($this->registry->all() as $capability) {
            $this->settings->set(self::PREFIX . $capability->key, in_array($capability->key, $checked, true) ? '1' : '0');
        }
        $this->settings->set(self::CONFIGURED_KEY, '1');
        $this->cache = null;

        $after = $this->impliedModules();

        $changed = [];
        foreach ($after as $module => $enabled) {
            // Only where the *implication* moved. Leaving the rest alone is what
            // lets an Advanced-panel override survive an unrelated capability edit.
            if (($before[$module] ?? null) !== $enabled) {
                $this->modules->setEnabled($module, $enabled);
                $changed[$module] = $enabled;
            }
        }

        return $changed;
    }

    /** Discard every override: make the modules match the capabilities exactly. */
    public function resetModulesToImplied(): void
    {
        foreach ($this->impliedModules() as $module => $enabled) {
            $this->modules->setEnabled($module, $enabled);
        }
    }

    private function stored(Capability $capability): bool
    {
        $raw = $this->settings->get(self::PREFIX . $capability->key);

        // A capability added by a later release has no row yet, so it falls back
        // to its own default rather than to "off" — a new feature should arrive
        // in the state its author intended, not silently disabled.
        return $raw === null ? $capability->defaultEnabled : $raw === '1';
    }

    /**
     * Before the screen is ever saved, a capability reads as on when everything it
     * needs is already on. That makes the first visit a description of the install
     * rather than a proposal to change it.
     */
    private function readBackFromModules(Capability $capability): bool
    {
        if ($capability->requires === []) {
            return $capability->defaultEnabled;
        }

        foreach ($capability->requires as $module) {
            if (!$this->modules->isEnabled($module)) {
                return false;
            }
        }

        return true;
    }
}
