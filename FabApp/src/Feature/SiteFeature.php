<?php

namespace App\Feature;

use App\Reservation\ReservableType;

/**
 * One thing this deployment can do.
 *
 * There used to be two layers here — "capabilities" the admin chose from, and
 * "modules" they switched on — with a registry, a derivation, a deviation model
 * and an Advanced panel joining them. The catalogue turned out to be one-to-one,
 * so all that machinery bought a second vocabulary for the same set of choices.
 * It is one concept now: a site feature has an operator-facing name *and* a key
 * that gates routes.
 *
 * The key is the old `SITE_MODULE` key, unchanged — renaming those rows would
 * need a migration for no functional gain, and every route gate already speaks
 * them.
 */
final class SiteFeature
{
    /**
     * @param string          $key           stable identifier; the `SITE_MODULE` row
     * @param string          $label         what the operator reads
     * @param string          $description   plain language: what turning this on gives you
     * @param string          $group         'resource' | 'activity' | 'directory'
     * @param ?string         $parent        set to make this an *add-on*: shown nested, and
     *                                       forced off while its parent is off
     * @param bool            $calendarLayer whether it draws a column on the shared calendar
     * @param ?ReservableType $reservable    the bookable kind it owns, if any
     */
    public function __construct(
        public readonly string $key,
        public readonly string $label,
        public readonly string $description,
        public readonly string $group,
        public readonly ?string $parent = null,
        public readonly bool $calendarLayer = false,
        public readonly ?ReservableType $reservable = null,
    ) {
    }

    public function isAddon(): bool
    {
        return $this->parent !== null;
    }
}
