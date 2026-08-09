<?php

namespace App\Tests\UsageRights;

use App\UsageRights\UsageCapabilityRegistry;
use PHPUnit\Framework\TestCase;

final class UsageCapabilityRegistryTest extends TestCase
{
    public function testOnlyCapabilitiesWithImplementedChokepointsAreOffered(): void
    {
        self::assertSame(['machines', 'places', 'person_booking', 'events'], (new UsageCapabilityRegistry())->keys());
    }
}
