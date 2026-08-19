<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S134d: a day can hold several ranges, and a date can override the week.
 *
 * Two limits the roadmap has recorded since S131, and both are the reason a lab
 * cannot sell machine time honestly:
 *
 *  1. **`UNIQ_OPENING_HOUR_VENUE_DAY` allowed exactly one range per weekday**, so
 *     a lunch break, an evening service or a split shift were inexpressible.
 *     ⚠️ Dropping a unique index is a **relaxation**, which is why this migration
 *     is safe in either order with the code: old code simply never writes a
 *     second row. The reverse is not true, so `down()` cannot always restore it —
 *     see below.
 *
 *  2. **No dated exceptions.** A public holiday had to be typed into the weekly
 *     grid and typed back out afterwards, and every lab that forgot left its
 *     calendar claiming to be open on Christmas Day.
 *
 * ⚠️ **Row semantics, unchanged for every row that exists today.** A closed day
 * is one row with `isClosed = 1` and null times; an open day is one *or more*
 * rows with both times set. Nothing needs backfilling: a week of seven single
 * rows keeps meaning exactly what it meant.
 *
 * ⚠️ **`SCHEDULE_EXCEPTION` REPLACES the weekday for that date rather than adding
 * to it.** One row with `isClosed = 1` closes the day with a reason the surfaces
 * can show; one or more rows with times are that date's opening, whatever the
 * weekday says. Merging the two instead would make "closed for stocktaking, 09:00
 * to 12:00 only" impossible to distinguish from a mistake.
 */
final class Version20260819100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S134d: several opening ranges per day + SCHEDULE_EXCEPTION for dated closures. Relaxation + additive.';
    }

    public function up(Schema $schema): void
    {
        // Guarded: an install that never received S106's index must not fail here.
        $this->addSql(<<<'SQL'
            SET @s := IF(
                (SELECT COUNT(*) FROM INFORMATION_SCHEMA.STATISTICS
                 WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = 'OPENING_HOUR'
                   AND INDEX_NAME = 'UNIQ_OPENING_HOUR_VENUE_DAY') > 0,
                'ALTER TABLE OPENING_HOUR DROP INDEX UNIQ_OPENING_HOUR_VENUE_DAY',
                'SELECT 1'
            )
        SQL);
        $this->addSql('PREPARE stmt FROM @s');
        $this->addSql('EXECUTE stmt');
        $this->addSql('DEALLOCATE PREPARE stmt');

        // A plain index replaces it: the reader still fetches a venue's week by
        // (venueId, dayOfWeek) and that lookup should not become a table scan
        // just because the pair stopped being unique.
        $this->addSql(<<<'SQL'
            SET @s := IF(
                (SELECT COUNT(*) FROM INFORMATION_SCHEMA.STATISTICS
                 WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = 'OPENING_HOUR'
                   AND INDEX_NAME = 'IDX_OPENING_HOUR_VENUE_DAY') > 0,
                'SELECT 1',
                'ALTER TABLE OPENING_HOUR ADD INDEX IDX_OPENING_HOUR_VENUE_DAY (venueId, dayOfWeek)'
            )
        SQL);
        $this->addSql('PREPARE stmt FROM @s');
        $this->addSql('EXECUTE stmt');
        $this->addSql('DEALLOCATE PREPARE stmt');

        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS SCHEDULE_EXCEPTION (
                id INT AUTO_INCREMENT NOT NULL,
                venueId INT NOT NULL,
                exceptionDate DATE NOT NULL,
                isClosed TINYINT(1) DEFAULT 1 NOT NULL,
                openTime TIME DEFAULT NULL,
                closeTime TIME DEFAULT NULL,
                reason VARCHAR(120) DEFAULT NULL,
                createdAt DATETIME NOT NULL,
                INDEX IDX_SCHEDULE_EXCEPTION_VENUE_DATE (venueId, exceptionDate),
                PRIMARY KEY(id),
                CONSTRAINT FK_SCHEDULE_EXCEPTION_VENUE FOREIGN KEY (venueId)
                    REFERENCES VENUE (id) ON DELETE CASCADE
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);
    }

    /**
     * ⚠️ **`down()` cannot restore the unique index if a lab has used the
     * feature**, and saying so is more honest than a statement that fails at 2am.
     * Re-adding it over two ranges on one day is a duplicate-key error. The
     * rollback therefore drops the exceptions table and leaves the index off; an
     * operator who genuinely wants the old constraint back has to decide which
     * range to keep, which is a data question and not a migration's to answer.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS SCHEDULE_EXCEPTION');
    }
}
