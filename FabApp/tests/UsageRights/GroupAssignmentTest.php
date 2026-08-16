<?php

namespace App\Tests\UsageRights;

use PHPUnit\Framework\TestCase;

/**
 * A package can be given to a group, and taken back.
 *
 * 🔴 **`USAGE_RIGHT_ASSIGNMENT.groupId` was read and never written.** S133b added
 * the column, `Version20260816140000` backfilled the role-based assignments into
 * it, and `UsageGrantRepository::paths()` has consulted it on every live
 * chokepoint since S134 — while no screen in the product could create a row. The
 * failure mode is the one S134b already found once with the venue parameter: a
 * dimension that is stored, migrated, read, and unreachable reads as a feature
 * and behaves as an absence.
 *
 * ⚠️ **Source-text tests, for the same reason as `VenueScopedGrantTest`.** What
 * broke was not SQL semantics but wiring — which table the write lands in, and
 * whether the read can see it. That is what source text shows, and an integration
 * test would need a database, a group, a package and a member to assert something
 * a grep proves.
 */
final class GroupAssignmentTest extends TestCase
{
    private const REPOSITORY = __DIR__ . '/../../src/UsageRights/UsagePackageRepository.php';
    private const CONTROLLER = __DIR__ . '/../../src/Controller/UsageRightsAdminController.php';
    private const TEMPLATE = __DIR__ . '/../../templates/site/admin-usage-package-form.html.twig';
    private const SERVICE = __DIR__ . '/../../src/UsageRights/UsageRightsService.php';

    public function testTheGroupWriteLandsInTheTableTheLiveReaderConsults(): void
    {
        $source = file_get_contents(self::REPOSITORY);

        self::assertStringContainsString(
            'public function assignGroup(int $packageId, string $groupKey',
            $source,
            'A group assignment is addressed by group key; the previous signature took a roleId and wrote the duplicate table.',
        );
        self::assertStringContainsString(
            "'groupId' => (int) \$groupId",
            $source,
            'The row must carry groupId on USAGE_RIGHT_ASSIGNMENT — that is the column paths() reads.',
        );
    }

    /**
     * 🔴 The duplicate group table. `USAGE_PACKAGE_GROUP_ASSIGNMENT` joined
     * `UTILISATEUR_ROLE`, so it could not see a group created since S133b and
     * would have disagreed with the live reader about every one of them. S134b
     * converged the grant tables; this is the group half of the same convergence.
     */
    public function testNothingInTheApplicationReadsTheDuplicateGroupTable(): void
    {
        $hits = [];
        $files = new \RecursiveIteratorIterator(new \RecursiveDirectoryIterator(__DIR__ . '/../../src'));
        foreach ($files as $file) {
            if ($file->isFile() && $file->getExtension() === 'php') {
                $body = preg_replace('/^\s*(\*|\/\/).*$/m', '', (string) file_get_contents($file->getPathname())) ?? '';
                if (str_contains($body, 'USAGE_PACKAGE_GROUP_ASSIGNMENT')) {
                    $hits[] = $file->getPathname();
                }
            }
        }

        self::assertSame([], $hits, 'USAGE_PACKAGE_GROUP_ASSIGNMENT is the duplicate; it awaits a contract migration and must have no readers.');
    }

    /**
     * ⚠️ The anonymous audience has no account, and `verdict()` reads grants only
     * for a signed-in member. Offering it in the picker would be a control that
     * cannot do anything — the rule the working brief states as "never show a
     * control that does nothing".
     */
    public function testTheAnonymousAudienceIsRefusedRatherThanOffered(): void
    {
        self::assertStringContainsString(
            'AudienceResolver::GUEST',
            file_get_contents(self::REPOSITORY),
            'The repository must refuse a guest assignment rather than store one that can never grant anything.',
        );
        self::assertStringContainsString(
            "\$group['key'] !== AudienceResolver::GUEST",
            file_get_contents(self::CONTROLLER),
            'And the picker must not offer it, or the only possible outcome of choosing it is an error message.',
        );
    }

    /**
     * A group assignment that cannot be seen cannot be revoked, and the list was
     * an `INNER JOIN UTILISATEUR` — which is exactly zero group rows.
     */
    public function testGroupAssignmentsAreVisibleAndRevocable(): void
    {
        $repository = file_get_contents(self::REPOSITORY);
        self::assertStringContainsString('LEFT JOIN UTILISATEUR u ON u.id = a.userId', $repository);
        self::assertStringContainsString('LEFT JOIN USER_GROUP g ON g.id = a.groupId', $repository);

        $template = file_get_contents(self::TEMPLATE);
        self::assertStringContainsString("assignment.kind == 'group'", $template);
        self::assertStringContainsString('usage_package_assign_group_', $template, 'The group form needs its own CSRF token.');
    }

    /**
     * 🔴 Two implementations of "does this person hold this grant" is how the two
     * answers drift. `v2ShadowVerdict()` had no caller and was the last reader of
     * the duplicate table.
     */
    public function testThereIsOneGrantReaderLeft(): void
    {
        self::assertStringNotContainsString(
            'function v2ShadowVerdict',
            file_get_contents(self::SERVICE),
            'The dead shadow verdict must stay deleted: paths() is the reader the chokepoints use.',
        );
    }
}
