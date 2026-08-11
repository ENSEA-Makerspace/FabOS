<?php

namespace App\Tests\Feature;

use App\Feature\FeatureWorkspaceRegistry;
use PHPUnit\Framework\TestCase;

/**
 * ⚠️ **Two cases were removed here in S130b**, both asserting the workspace-tab
 * map: that every list route resolved to exactly one active tab, and that Quotas
 * and Reporting appeared only where implemented. The tabs were a second
 * sub-navigation drawn beside the sidebar's own, and they are gone — so the
 * assertions described machinery rather than behaviour. What they were protecting
 * moved with the navigation: see `App\Tests\Nav\AdminNavCatalogueTest`.
 *
 * What remains is what the registry still is: descriptive metadata for the
 * development-mode workspace vision screen.
 */
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
