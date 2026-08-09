<?php

namespace App\Tests\Feature;

use App\Entity\Badge;
use PHPUnit\Framework\TestCase;

final class BadgeLifecycleTest extends TestCase
{
    public function testArchiveAndRestoreNeverReplaceTheDefinition(): void
    {
        $badge = (new Badge())->setNom('Sécurité laser');
        self::assertFalse($badge->isArchived());

        $badge->archive();
        $archivedAt = $badge->getArchivedAt();
        self::assertNotNull($archivedAt);
        self::assertSame($archivedAt, $badge->archive()->getArchivedAt());

        self::assertSame($badge, $badge->restore());
        self::assertFalse($badge->isArchived());
    }
}
