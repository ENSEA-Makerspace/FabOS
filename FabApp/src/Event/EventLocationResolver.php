<?php

namespace App\Event;

use App\Entity\Event;
use App\Service\SiteSettingService;

/**
 * Builds an EventLocation, pulling the lab's own address from site settings for
 * on-site events so it never has to be retyped per event.
 */
final class EventLocationResolver
{
    public function __construct(private readonly SiteSettingService $settings)
    {
    }

    public function resolve(Event $event): EventLocation
    {
        $onsite = $event->isOnsite();
        $address = trim($onsite ? $this->settings->getLabAddress() : (string) $event->getAddress());
        $label = trim((string) $event->getLieu());

        return new EventLocation(
            $onsite,
            $label,
            $address,
            $address !== '' ? $this->directionsUrl($address) : null,
        );
    }

    private function directionsUrl(string $address): string
    {
        return 'https://www.openstreetmap.org/search?query=' . rawurlencode($address);
    }
}
