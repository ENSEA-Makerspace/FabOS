<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Undo one column that should never have been added (S134b).
 *
 * 🔴 **`USAGE_PACKAGE.archivedAt` is unused, and adding it was my mistake.** The
 * audit that found machines and trainings could not be retired grouped admin routes
 * by NAME, and packages have no `_archive` route — so they looked like the same gap.
 * They are not: a package retires through the `active` checkbox on its edit form,
 * shown as a state chip in the list and written by `UsagePackageRepository`. An audit
 * by route name cannot see a verb that is a FIELD.
 *
 * ⚠️ Safe to drop precisely because nothing ever read or wrote it: it was added by
 * `Version20260821120000` and no code shipped against it.
 */
final class Version20260821130000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S134b: drop the unused USAGE_PACKAGE.archivedAt — packages already retire via `active`.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE USAGE_PACKAGE DROP COLUMN IF EXISTS archivedAt');
    }

    public function down(Schema $schema): void
    {
        $this->addSql("ALTER TABLE USAGE_PACKAGE ADD archivedAt DATETIME DEFAULT NULL COMMENT '(DC2Type:datetime_immutable)'");
    }
}
