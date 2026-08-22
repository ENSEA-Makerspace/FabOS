<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S147, J-2 and J-21: the columns that let eight objects stop being destroyed,
 * and a grant stop depending on a name.
 *
 * 🔴 **This migration adds columns and reads nothing.** Nothing in the deployed
 * code names them yet, on purpose — see the deploy-order note at the bottom. It
 * is safe to run at any moment, and safe NOT to run: the site behaves exactly as
 * it does today until the code that uses these columns ships.
 *
 * ---
 *
 * **1. `archivedAt` on eight tables (J-2, which is S134f finally starting).**
 *
 * The review checked the controllers rather than the route names, and found
 * `->remove()` in eight admin actions: a space, an event, a material, an
 * institution, a lab page, a maintenance task, an RFID reader and a project are
 * all destroyed outright. `Machine`, `Formation`, `Badge`, `EventCategory`,
 * `MachineCategory` and `LoanableItem` already carry `archivedAt` and archive
 * instead; these eight are the remainder.
 *
 * ⚠️ **The route name lies in the other direction too**, and the next session
 * should not be fooled by it: `/admin/loanable-items/{id}/delete` ARCHIVES —
 * `AdminController::archiveLoanableItem()`. It is a misnamed route, not a ninth
 * hard delete, and renaming it is the whole of that fix.
 *
 * 🔴 **The hard part is not the column, it is the promise.** The roadmap's S134f
 * says archiving a bookable resource must explicitly cancel its future bookings.
 * `PLACE` is bookable; `EVENEMENT` has registrations and `SessionEnrolment` rows
 * hanging off it since S146e. Adding the column does not do that, and the code
 * that flips it must — otherwise a member keeps a confirmed booking on a space
 * that no longer exists to the lab.
 *
 * ⚠️ **`EVENEMENT`, not `EVENT`.** The table has been called that since the
 * beginning and it has caught people out before.
 *
 * ---
 *
 * **2. `categoryId` on `USAGE_PACKAGE_GRANT` (J-21).**
 *
 * S144b scoped a grant to a machine category with `categoryLabel`, a LABEL,
 * compared by exact string equality in `UsageGrantRepository`:
 *
 *     AND (g.categoryLabel IS NULL OR :categoryLabel IS NULL OR g.categoryLabel = :categoryLabel)
 *
 * Its comment argues the label follows a rename because `MACHINE.categoryLabel`
 * is the join key. That is true of the machines; it is **not** true of the grant.
 * Rename "Impression 3D" and every machine moves with it, while the grant keeps
 * naming a string nothing answers to any more — so the package silently stops
 * covering the printers, and the symptom is a refused booking with no visible
 * cause. Elsewhere this codebase already wrote the rule: *the slug is the key,
 * never the label.*
 *
 * ⚠️ **Both columns coexist on purpose.** `categoryLabel` stays, the reader keeps
 * honouring it, and `categoryId` is filled in when the editor learns to write it.
 * A grant with neither is unrestricted, exactly as today. Dropping the label is a
 * CONTRACT step and belongs in its own migration, after a backfill that can be
 * checked.
 *
 * ---
 *
 * ⚠️ **Deploy order, and it is the opposite of S144b's.** That one could ship its
 * code first because the reader probes for its columns. This one has no probe:
 * the moment an entity declares `archivedAt`, Doctrine puts it in every SELECT
 * for that table, and a missing column is a 500 on every page that lists those
 * objects. `LOANABLE_ITEM.archivedAt` shipped that way once and took two pages
 * down until the migration was run. **So: this migration first, then the code.**
 */
final class Version20260822120000 extends AbstractMigration
{
    /** Table => the column comment, so the schema explains itself to the next reader. */
    private const ARCHIVABLE = [
        'PLACE' => 'S147 J-2: archived instead of deleted. Cancelling future bookings is the caller\'s job.',
        'EVENEMENT' => 'S147 J-2: archived instead of deleted. Registrations and enrolments outlive the row.',
        'MATERIAL' => 'S147 J-2: archived instead of deleted.',
        'INSTITUTION' => 'S147 J-2: archived instead of deleted.',
        'LAB_PAGE' => 'S147 J-2: archived instead of deleted. An unpublished page is not the same thing.',
        'MAINTENANCE_TASK' => 'S147 J-2: archived instead of deleted. A done task is history, not rubbish.',
        'RFID_READER' => 'S147 J-2: archived instead of deleted. Its logs point at it.',
        'CREATION' => 'S147 J-2: archived instead of deleted.',
    ];

    public function getDescription(): string
    {
        return 'S147 J-2/J-21: archivedAt on eight tables + USAGE_PACKAGE_GRANT.categoryId. Expand-only, no backfill, nothing reads them yet.';
    }

    public function up(Schema $schema): void
    {
        foreach (self::ARCHIVABLE as $table => $comment) {
            // ⚠️ `IF NOT EXISTS` because two of these tables have been touched by
            // hand on at least one install, and a migration that cannot be re-run
            // is a migration somebody will be afraid to run.
            $this->addSql(sprintf(
                "ALTER TABLE %s ADD COLUMN IF NOT EXISTS archivedAt DATETIME DEFAULT NULL COMMENT '%s'",
                $table,
                str_replace("'", "''", $comment),
            ));
        }

        $this->addSql(
            "ALTER TABLE USAGE_PACKAGE_GRANT
             ADD COLUMN IF NOT EXISTS categoryId INT DEFAULT NULL
             COMMENT 'S147 J-21: the category by identity. categoryLabel stays until a checked backfill retires it.'",
        );

        // No foreign key: a grant that names a category which is later deleted
        // should keep saying so and simply stop matching, exactly like the label
        // does today. A cascade here would silently widen the package instead.
        $this->addSql(
            'CREATE INDEX IF NOT EXISTS IDX_USAGE_PACKAGE_GRANT_CATEGORY ON USAGE_PACKAGE_GRANT (categoryId)',
        );
    }

    public function down(Schema $schema): void
    {
        foreach (array_keys(self::ARCHIVABLE) as $table) {
            $this->addSql(sprintf('ALTER TABLE %s DROP COLUMN IF EXISTS archivedAt', $table));
        }

        $this->addSql('DROP INDEX IF EXISTS IDX_USAGE_PACKAGE_GRANT_CATEGORY ON USAGE_PACKAGE_GRANT');
        $this->addSql('ALTER TABLE USAGE_PACKAGE_GRANT DROP COLUMN IF EXISTS categoryId');
    }
}
