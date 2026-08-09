<?php

namespace App\UsageRights;

/** One member-facing capability that has a real enforcement chokepoint. */
final readonly class UsageCapability
{
    public function __construct(
        public string $key,
        public string $featureKey,
        public string $labelKey,
        public string $descriptionKey,
        public string $icon,
        public string $group,
    ) {
    }
}
