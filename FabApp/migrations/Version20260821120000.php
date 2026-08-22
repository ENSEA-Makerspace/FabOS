<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * A machine, a training and a package can be retired (S134b).
 *
 * 🔴 **Phase G's exit criterion, measured and failing.** The roadmap asks that
 * "chaque objet annoncé est créable, éditable, archivable depuis son workspace".
 * An audit of all 111 admin routes found three objects that can be created and
 * edited and then **never removed**: `MACHINE`, `FORMATION` and `USAGE_PACKAGE` have
 * no `archivedAt`, no delete route, and no in-page action either. A laser cutter
 * sold last year is still in every catalogue, every booking picker and every
 * calendar, for ever, and the only way out is the database.
 *
 * 🔴 **Archived, never deleted — and here that is not a preference.** `RESERVATION`,
 * `LOG_UTILISATION`, `ACCESS_RFID_LOG` and `PROGRESSION` all point at these rows.
 * Deleting a machine would take its usage history with it, or orphan it; the
 * roadmap's own rule ("un badge n'est jamais effacé", `AccountAnonymiser`) is the
 * same rule. Archiving hides the row from the pickers and the catalogues while every
 * past booking keeps rendering exactly as it did.
 *
 * ⚠️ **Same column, same name, same shape as `LOANABLE_ITEM`, `BADGE`,
 * `MACHINE_CATEGORY` and `EVENT_CATEGORY`** — this is the established pattern, not a
 * new one. Nullable, no default, no backfill: every existing row is live, which is
 * what it already meant.
 *
 * ⚠️ **Mapped ORM columns ⇒ this migration ships BEFORE the code.** Every Doctrine
 * query on those three entities selects every mapped column — the
 * `OpeningHour.scopeType` 500 of S134d, for the third time.
 */
final class Version20260821120000 extends AbstractMigration
{
    private const TABLES = ['MACHINE', 'FORMATION', 'USAGE_PACKAGE'];

    public function getDescription(): string
    {
        return 'S134b: archivedAt on MACHINE, FORMATION and USAGE_PACKAGE — the three objects that could never be retired. Expand-only.';
    }

    public function up(Schema $schema): void
    {
        foreach (self::TABLES as $table) {
            $exists = (int) $this->connection->fetchOne(
                'SELECT COUNT(*) FROM information_schema.COLUMNS
                 WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = ? AND COLUMN_NAME = ?',
                [$table, 'archivedAt'],
            );

            if ($exists === 0) {
                $this->addSql(sprintf(
                    "ALTER TABLE %s ADD archivedAt DATETIME DEFAULT NULL COMMENT '(DC2Type:datetime_immutable)'",
                    $table,
                ));
            }
        }
    }

    /**
     * ⚠️ Dropping these un-archives everything: a machine retired last month comes
     * back into every catalogue and picker. Nothing else is lost — no other column
     * was rewritten — but it is a rollback to run knowingly.
     */
    public function down(Schema $schema): void
    {
        foreach (self::TABLES as $table) {
            $this->addSql(sprintf('ALTER TABLE %s DROP COLUMN IF EXISTS archivedAt', $table));
        }
    }
}
