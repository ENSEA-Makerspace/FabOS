<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * A dated closure can last more than one day (S146g).
 *
 * 🔴 **The gap, in the operator's words: "i cant specify a range".** `SCHEDULE_EXCEPTION`
 * held a single `exceptionDate`, so "closed for the two weeks of Christmas" meant
 * filling the same form fourteen times — and then removing it meant deleting fourteen
 * rows one by one. A summer closure is one fact about the lab, not twenty-one.
 *
 * ⚠️ **A range on the row, NOT one row per day** — the opposite of the choice S146d
 * made for event series, and deliberately so. Those are *occurrences*: each session
 * moves, fills and is cancelled on its own, so they must be separate rows. A closure
 * is a *single statement* with a start and an end; splitting it would make "cancel the
 * closure" a bulk delete, which is exactly the complaint this fixes.
 *
 * ⚠️ **Nullable, and null means "one day".** Every existing row keeps meaning exactly
 * what it meant, and every reader treats a missing end as `endDate = exceptionDate`
 * via `COALESCE`. Nothing is backfilled, so this is expand-only and the running code
 * is unaffected until the code that reads it arrives.
 *
 * ⚠️ **Mapped ORM column ⇒ this migration ships BEFORE that code.** Every Doctrine
 * query on `ScheduleException` selects every mapped column, so no repository
 * try/catch can degrade it — the `OpeningHour.scopeType` 500 of S134d, again.
 */
final class Version20260821100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S146g: SCHEDULE_EXCEPTION.endDate — a dated closure can span several days. Expand-only.';
    }

    public function up(Schema $schema): void
    {
        $exists = (int) $this->connection->fetchOne(
            'SELECT COUNT(*) FROM information_schema.COLUMNS
             WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = ? AND COLUMN_NAME = ?',
            ['SCHEDULE_EXCEPTION', 'endDate'],
        );

        if ($exists === 0) {
            $this->addSql("ALTER TABLE SCHEDULE_EXCEPTION ADD endDate DATE DEFAULT NULL COMMENT '(DC2Type:date_immutable)'");
        }

        // ⚠️ The lookups ask "does this date fall inside the row", so the index that
        // helps is the pair, not the new column alone.
        $index = (int) $this->connection->fetchOne(
            'SELECT COUNT(*) FROM information_schema.STATISTICS
             WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = ? AND INDEX_NAME = ?',
            ['SCHEDULE_EXCEPTION', 'IDX_SCHEDULE_EXCEPTION_SPAN'],
        );

        if ($index === 0) {
            $this->addSql('CREATE INDEX IDX_SCHEDULE_EXCEPTION_SPAN ON SCHEDULE_EXCEPTION (venueId, exceptionDate, endDate)');
        }
    }

    /**
     * ⚠️ Dropping the column collapses every multi-day closure back to its first day.
     * Nothing else is lost — the start date is the original column — but a two-week
     * shutdown would silently become a one-day one, so this is a rollback to run
     * knowingly rather than casually.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('DROP INDEX IF EXISTS IDX_SCHEDULE_EXCEPTION_SPAN ON SCHEDULE_EXCEPTION');
        $this->addSql('ALTER TABLE SCHEDULE_EXCEPTION DROP COLUMN IF EXISTS endDate');
    }
}
