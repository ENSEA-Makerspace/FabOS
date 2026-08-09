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

    public function testEveryPhaseCListRouteResolvesToOneActiveWorkspaceTab(): void
    {
        $registry = new FeatureWorkspaceRegistry();

        foreach (['app_admin_machines', 'app_admin_machine_categories', 'app_admin_machine_models', 'app_admin_materials', 'app_admin_maintenance', 'app_admin_events', 'app_admin_loanable_items', 'app_admin_loans', 'app_admin_places', 'app_admin_formations', 'app_admin_badges', 'app_admin_creations', 'app_admin_lab_pages', 'app_admin_users', 'app_admin_opening_hours', 'app_admin_usage_rights', 'app_admin_institutions', 'app_admin_features', 'app_admin_homepage', 'app_admin_settings', 'app_admin_emails'] as $route) {
            $workspace = $registry->forRoute($route);
            self::assertNotNull($workspace, $route);
            self::assertCount(1, array_filter($workspace['tabs'], static fn (array $tab): bool => $tab['active']), $route);
        }
    }

    public function testFutureQuotaAndReportingTabsStayHidden(): void
    {
        $registry = new FeatureWorkspaceRegistry();
        foreach (['app_admin_machines', 'app_admin_events', 'app_admin_places', 'app_admin_formations'] as $route) {
            $labels = array_column($registry->forRoute($route)['tabs'], 'label');
            self::assertNotContains('Quotas', $labels);
            self::assertNotContains('Reporting', $labels);
        }
    }
}
