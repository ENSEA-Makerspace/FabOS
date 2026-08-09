<?php

declare(strict_types=1);

namespace App\Tests\UsageRights;

use App\UsageRights\ShadowUsageGrant;
use App\UsageRights\UsageGrantAction;
use PHPUnit\Framework\TestCase;

final class UsageGrantActionTest extends TestCase
{
    public function testManageIncludesReportingButNeverUse(): void
    {
        self::assertTrue(UsageGrantAction::Manage->includesReporting());
        self::assertFalse(UsageGrantAction::Use->includesReporting());
        self::assertFalse(UsageGrantAction::Manage->covers(UsageGrantAction::Use));
    }

    public function testShadowGrantRequiresEveryDeclaredScope(): void
    {
        $grant = new ShadowUsageGrant('equipment', 'maintenance', UsageGrantAction::Manage, ['venue' => ['default'], 'category' => ['laser']]);

        self::assertTrue($grant->covers('equipment', 'maintenance', UsageGrantAction::Manage, ['venue' => 'default', 'category' => 'laser']));
        self::assertFalse($grant->covers('equipment', 'maintenance', UsageGrantAction::Manage, ['venue' => 'other', 'category' => 'laser']));
        self::assertFalse($grant->covers('equipment', 'maintenance', UsageGrantAction::Use, ['venue' => 'default']));
    }
}
