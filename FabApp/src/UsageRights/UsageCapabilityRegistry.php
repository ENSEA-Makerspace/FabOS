<?php

namespace App\UsageRights;

/**
 * The honest package catalogue: only capabilities whose writes are enforced.
 * Site features describe what FabOS can show; this narrower registry describes
 * what a package can actually grant to one member.
 */
final class UsageCapabilityRegistry
{
    public const GROUP_BOOKING = 'booking';
    public const GROUP_ACTIVITY = 'activity';

    /** @return array<string, UsageCapability> */
    public function all(): array
    {
        return [
            'machines' => new UsageCapability('machines', 'machines', 'usage_rights.capabilities.machines.label', 'usage_rights.capabilities.machines.description', 'machines', self::GROUP_BOOKING),
            'places' => new UsageCapability('places', 'places', 'usage_rights.capabilities.places.label', 'usage_rights.capabilities.places.description', 'places', self::GROUP_BOOKING),
            'person_booking' => new UsageCapability('person_booking', 'person_booking', 'usage_rights.capabilities.person_booking.label', 'usage_rights.capabilities.person_booking.description', 'users', self::GROUP_BOOKING),
            'events' => new UsageCapability('events', 'events', 'usage_rights.capabilities.events.label', 'usage_rights.capabilities.events.description', 'events', self::GROUP_ACTIVITY),
        ];
    }

    public function get(string $key): ?UsageCapability
    {
        return $this->all()[$key] ?? null;
    }

    /** @return list<string> */
    public function keys(): array
    {
        return array_keys($this->all());
    }
}
