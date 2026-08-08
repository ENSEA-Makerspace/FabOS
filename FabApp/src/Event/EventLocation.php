<?php

namespace App\Event;

use App\Entity\Event;
use App\Service\SiteSettingService;

/**
 * Answers "where is this, and how do I get there?" for one event.
 *
 * The two location modes differ in where the address comes from, not in what
 * the visitor sees, so resolving that difference belongs in one place rather
 * than in every template that wants to show a map link.
 *
 * The directions URL is **derived, never stored**. A map link saved next to an
 * address is one more field to keep in step and goes stale the moment someone
 * corrects a typo in the street name. OpenStreetMap is used rather than a
 * proprietary map because the link is rendered for every visitor including ones
 * who never agreed to a third party's tracking — it is a plain search URL with
 * no key, no embed and no script.
 */
final readonly class EventLocation
{
    public function __construct(
        public bool $onsite,
        /** The venue's human name, e.g. "Grande salle" — may be empty. */
        public string $label,
        /** The postal address that applies, resolved from the mode. May be empty. */
        public string $address,
        /** Map search link, or null when there is no address to point at. */
        public ?string $directionsUrl,
    ) {
    }

    public function hasAnything(): bool
    {
        return $this->label !== '' || $this->address !== '';
    }
}
