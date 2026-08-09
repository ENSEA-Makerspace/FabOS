<?php

namespace App\Tests\Reporting;

use App\Reporting\AnalyticsCapabilityRegistry;
use App\Reporting\ReportData;
use App\Reporting\ReportingAdapter;
use App\Reporting\ReportingRegistry;
use App\Reporting\ReportScope;
use PHPUnit\Framework\TestCase;

final class ReportingRegistryTest extends TestCase
{
    public function testCapabilitiesAreStableAndAdapterIsSelectedByWorkspace(): void
    {
        self::assertSame(['analytics.view', 'analytics.export'], (new AnalyticsCapabilityRegistry())->all());
        $adapter = new class implements ReportingAdapter {
            public function supports(string $workspace): bool { return $workspace === 'equipment'; }
            public function report(ReportScope $scope): ReportData { return new ReportData([], [], []); }
            public function export(ReportScope $scope): iterable { return []; }
        };

        self::assertSame($adapter, (new ReportingRegistry([$adapter]))->forWorkspace('equipment'));
    }
}
