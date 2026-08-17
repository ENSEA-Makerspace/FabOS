<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S144b: a grant learns to say **on what** and **when**.
 *
 * The operator's sentence was "one package must let a member 3D print on Monday
 * afternoons". The model could say which feature, which action, which location
 * and which section — and nothing at all about the machine or the clock, so a lab
 * selling off-peak or per-machine access had to sell full access instead.
 *
 * Two additions, both **expand-only**:
 *
 *  1. **Three columns on `USAGE_PACKAGE_GRANT`.** `reservableType`,
 *     `reservableId`, `categoryLabel`. All NULL, and NULL keeps its established
 *     meaning in this model: *this grant is not restricted on that axis*. Every
 *     grant that exists today therefore behaves exactly as it did.
 *     ⚠️ `categoryLabel` is a LABEL and not a foreign key, deliberately —
 *     `MACHINE.categoryLabel` is the join key S133 kept when it built the category
 *     CRUD, so a grant scoped to "Impression 3D" follows a renamed category the
 *     same way a machine does, and a new printer joins the package by being put in
 *     the category rather than by editing what was sold.
 *
 *  2. **`USAGE_GRANT_WINDOW`.** Weekly opening slices of one grant: ISO day
 *     (1 = Monday), and minutes from local midnight. Wall-clock, not instants —
 *     "Monday afternoon" is a fact about the lab's clock and must not move by an
 *     hour in summer, exactly like the opening hours.
 *     ⚠️ **No window rows means no time restriction.** That is what makes this
 *     migration safe to run on a live install: a grant nobody has given a window
 *     to is unchanged. The reader enforces coverage of the WHOLE booking by the
 *     union of a grant's windows, not an overlap — a Monday-afternoon package
 *     must refuse a Monday-evening booking.
 *
 * ⚠️ **Deploy order does not matter here, and that is by design.** Grants v2 is
 * live on all four chokepoints and `UsageGrantRepository::paths()` returns an
 * empty list on any query error — which is a refusal for everybody. So the reader
 * probes for these columns and this table before naming them, and falls back to
 * the pre-S144b query when they are absent. Code may precede the migration
 * without taking booking away from the lab; running the migration is what turns
 * the new dimensions on.
 *
 * ⚠️ Nothing is dropped and nothing is backfilled. There is no data to move: the
 * dimensions did not exist before.
 */
final class Version20260817100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S144b: grant scope (resource, category) + USAGE_GRANT_WINDOW. Expand-only, no backfill.';
    }

    public function up(Schema $schema): void
    {
        // ⚠️ Guarded one by one rather than in a single ALTER: a re-run after a
        // partial failure must skip what is already there, and Doctrine does not
        // record a version whose migration threw part-way.
        foreach ([
            'reservableType' => 'VARCHAR(20) DEFAULT NULL',
            'reservableId' => 'INT DEFAULT NULL',
            'categoryLabel' => 'VARCHAR(100) DEFAULT NULL',
        ] as $column => $definition) {
            $this->addSql(sprintf(
                "SET @s := IF(
                    (SELECT COUNT(*) FROM INFORMATION_SCHEMA.COLUMNS
                     WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = 'USAGE_PACKAGE_GRANT' AND COLUMN_NAME = '%s') > 0,
                    'SELECT 1',
                    'ALTER TABLE USAGE_PACKAGE_GRANT ADD %s %s'
                )",
                $column,
                $column,
                $definition,
            ));
            $this->addSql('PREPARE stmt FROM @s');
            $this->addSql('EXECUTE stmt');
            $this->addSql('DEALLOCATE PREPARE stmt');
        }

        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS USAGE_GRANT_WINDOW (
                id INT AUTO_INCREMENT NOT NULL,
                grantId INT NOT NULL,
                dayOfWeek TINYINT NOT NULL,
                startMinute SMALLINT NOT NULL,
                endMinute SMALLINT NOT NULL,
                createdAt DATETIME NOT NULL,
                INDEX IDX_USAGE_GRANT_WINDOW_GRANT (grantId),
                UNIQUE INDEX UNIQ_USAGE_GRANT_WINDOW (grantId, dayOfWeek, startMinute, endMinute),
                PRIMARY KEY(id),
                CONSTRAINT FK_USAGE_GRANT_WINDOW_GRANT FOREIGN KEY (grantId)
                    REFERENCES USAGE_PACKAGE_GRANT (id) ON DELETE CASCADE
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);
    }

    /**
     * ⚠️ Down drops the windows and the three columns. It loses the restrictions
     * an operator wrote, which is the honest consequence of removing the only
     * place they are stored — there is nowhere else to put them. Nothing else in
     * the rights model depends on either, so the model goes back to being
     * unrestricted on those axes rather than broken.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS USAGE_GRANT_WINDOW');
        $this->addSql('ALTER TABLE USAGE_PACKAGE_GRANT DROP COLUMN reservableType');
        $this->addSql('ALTER TABLE USAGE_PACKAGE_GRANT DROP COLUMN reservableId');
        $this->addSql('ALTER TABLE USAGE_PACKAGE_GRANT DROP COLUMN categoryLabel');
    }
}
