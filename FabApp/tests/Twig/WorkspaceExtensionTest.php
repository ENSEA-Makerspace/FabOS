<?php

namespace App\Tests\Twig;

use App\Feature\FeatureWorkspaceRegistry;
use App\Twig\WorkspaceExtension;
use PHPUnit\Framework\TestCase;

final class WorkspaceExtensionTest extends TestCase
{
    public function testFilterChipsAreScalarAllowListedBoundedAndRemovePagination(): void
    {
        $extension = new WorkspaceExtension(new FeatureWorkspaceRegistry());
        $chips = $extension->filterChips([
            'q' => str_repeat('a', 100),
            'statut' => 'active',
            'niveau' => ['crafted-array'],
            'unknown' => 'ignored',
            'location' => 'atelier-nord',
            'page' => '4',
        ]);

        self::assertCount(2, $chips);
        self::assertSame(80, mb_strlen($chips[0]['value']));
        self::assertSame('atelier-nord', $chips[0]['remove_query']['location']);
        self::assertArrayNotHasKey('page', $chips[0]['remove_query']);
        self::assertArrayNotHasKey('unknown', $chips[0]['remove_query']);
    }

    public function testNoMoreThanSixChipsAreRendered(): void
    {
        $extension = new WorkspaceExtension(new FeatureWorkspaceRegistry());
        self::assertCount(6, $extension->filterChips([
            'q' => 'x', 'statut' => 'x', 'niveau' => 'x', 'badge' => 'x',
            'category' => 'x', 'role' => 'x', 'dateFrom' => 'x', 'dateTo' => 'x',
        ]));
    }
}
