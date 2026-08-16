<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S133: machine categories stop being a facet and become manageable.
 *
 * ⚠️ **Expand only — nothing is contracted and nothing is rewritten.**
 * `MACHINE.categoryLabel` stays exactly as it is and stays the join key. This
 * table carries only what a *derived* facet cannot have: a category that exists
 * before any machine uses it, an icon chosen once instead of per machine, and an
 * archived state. Old code that never heard of this table keeps working, which is
 * the whole point of the expand step.
 *
 * ⚠️ **The label is the key on purpose, not a foreign key.** Introducing
 * `MACHINE.categoryId` would be a contract step: every template, form, filter and
 * export that reads `categoryLabel` would have to move first, and a half-migrated
 * install would have machines in two category systems at once. Renaming is
 * therefore a two-statement operation the application performs together, and the
 * `UPDATE MACHINE` half is what makes it a rename rather than an orphaning.
 *
 * ⚠️ The backfill is deliberately included: without it the screen would open on
 * an empty list beside a dozen categories already in use, which reads as data
 * loss. It inserts one row per distinct label in use and touches no machine.
 */
final class Version20260816120000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S133: MACHINE_CATEGORY, and LOANABLE_ITEM.archivedAt. Both additive.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS MACHINE_CATEGORY (
                id INT AUTO_INCREMENT NOT NULL,
                label VARCHAR(100) NOT NULL,
                iconSlug VARCHAR(50) DEFAULT NULL,
                archivedAt DATETIME DEFAULT NULL COMMENT '(DC2Type:datetime_immutable)',
                createdAt DATETIME NOT NULL COMMENT '(DC2Type:datetime_immutable)',
                UNIQUE INDEX UNIQ_MACHINE_CATEGORY_LABEL (label),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);

        // One row per label already in use, with the icon the machines of that
        // category most recently carried. `IGNORE` rather than a subquery guard:
        // the unique index is the authority, and a second run must be harmless.
        $this->addSql(<<<'SQL'
            INSERT IGNORE INTO MACHINE_CATEGORY (label, iconSlug, archivedAt, createdAt)
            SELECT TRIM(m.categoryLabel), MAX(m.iconSlug), NULL, NOW()
            FROM MACHINE m
            WHERE m.categoryLabel IS NOT NULL AND TRIM(m.categoryLabel) <> ''
            GROUP BY TRIM(m.categoryLabel)
        SQL);
    }

        // ⚠️ **The second half: a loan object can be retired without erasing who
        // borrowed it.** Deleting one removed its loans with it — "Objet supprimé
        // (et ses prêts)" — so retiring a battery pack destroyed the record of
        // every borrowing of it. Nullable and defaulted to NULL, so every existing
        // row is "not archived" and no backfill is needed.
        $this->addSql("ALTER TABLE LOANABLE_ITEM ADD archivedAt DATETIME DEFAULT NULL COMMENT '(DC2Type:datetime_immutable)'");
    }

    public function down(Schema $schema): void
    {
        // Safe to reverse: no machine or loan row was touched on the way up, so
        // this returns the categories to being the derived facet they were and the
        // loan objects to being deletable-only.
        $this->addSql('DROP TABLE IF EXISTS MACHINE_CATEGORY');
        $this->addSql('ALTER TABLE LOANABLE_ITEM DROP COLUMN archivedAt');
    }
}
