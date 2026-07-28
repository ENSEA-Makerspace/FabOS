<?php

namespace App\Capability;

/**
 * One thing a deployment can *do*, described the way an operator would describe
 * it — never the way the code is organised.
 *
 * A capability switches modules on; modules stay internal. The distinction is the
 * whole point of the layer: "Réserver de l'équipement" is a question a newcomer
 * can answer, `machines` + `maintenance` + `materials` is not.
 */
final class Capability
{
    /**
     * @param string   $key         stable identifier, persisted in SITE_SETTING
     * @param string   $label       what the operator reads
     * @param string   $description plain language: what turning this on gives you
     * @param string[] $requires    the modules this capability switches on
     * @param string   $group       presentation grouping: 'resource' | 'activity'
     * @param ?string  $parent      set to make this an *add-on* of another capability:
     *                              it is shown nested, and contributes nothing while
     *                              its parent is off
     * @param bool     $defaultEnabled
     *                              what a fresh install gets. Add-ons that are extra
     *                              work for the operator ship off; the plain reading
     *                              of a capability ships on
     */
    public function __construct(
        public readonly string $key,
        public readonly string $label,
        public readonly string $description,
        public readonly array $requires,
        public readonly string $group,
        public readonly ?string $parent = null,
        public readonly bool $defaultEnabled = true,
    ) {
    }

    public function isAddon(): bool
    {
        return $this->parent !== null;
    }
}
