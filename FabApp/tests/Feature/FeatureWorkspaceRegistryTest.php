<?php

namespace App\Tests\Feature;

use App\Feature\FeatureWorkspaceRegistry;
use PHPUnit\Framework\TestCase;

final class FeatureWorkspaceRegistryTest extends TestCase
{
    public function testWorkspaceKeysAreUniqueAndOnlyUseAndManageAreExposed(): void
    {
        $workspaces = (new FeatureWorkspaceRegistry())->all();
        $keys = array_column($workspaces, 'key');

        self::assertSame($keys, array_values(array_unique($keys)));
        self::assertCount(13, $workspaces);
        foreach ($workspaces as $workspace) {
            self::assertSame(['Use', 'Manage'], $workspace['actions']);
            self::assertNotEmpty($workspace['sections']);
            self::assertNotEmpty($workspace['scopes']);
            foreach ($workspace['routes'] as $route) {
                self::assertNotSame('', $route['capability']);
                self::assertContains($route['right'], ['Use', 'Manage']);
            }
        }
    }

    public function testThemeContractUsesStableRegistryReferences(): void
    {
        $contract = (new FeatureWorkspaceRegistry())->themeContract();

        self::assertSame('Configuration → Thèmes', $contract['workspace']);
        self::assertStringContainsString('clés du registre', $contract['stableReferences']);
        self::assertContains('Publier', $contract['workflow']);
    }
}
