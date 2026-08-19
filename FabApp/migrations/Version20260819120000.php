<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S134d (suite): a schedule can be attached below the location.
 *
 * The roadmap's target was "un horaire **rattachable** — lieu par défaut,
 * surchargeable par workspace puis par ressource — avec héritage et une seule
 * réponse effective par instant", and its warning was "**ne pas dupliquer la
 * table par type de ressource** : c'est la même question à des portées
 * différentes". So: two nullable columns on the table that already answers the
 * question, and three levels expressed in them.
 *
 *  - `scopeType IS NULL`                → the LOCATION's week. What exists today.
 *  - `scopeType = 'machine', scopeId IS NULL` → every machine at that location.
 *  - `scopeType = 'machine', scopeId = 12`    → that one machine.
 *
 * ⚠️ **The middle level is the resource KIND, not a "workspace".** The roadmap
 * says workspace, but the thing a schedule actually varies by is what is being
 * booked — machines, spaces, people — and `ReservableType` is the name this
 * codebase already gives that. Inventing a parallel workspace key would have
 * been a second vocabulary for one idea.
 *
 * ⚠️ **Levels INTERSECT, they do not replace** (operator's decision,
 * 2026-08-19). A resource schedule can only narrow its location's; nobody uses
 * the laser cutter while the building is locked, and it means no level can ever
 * fail open. The cost is that hours written wider than the location do nothing,
 * so the editor shows the effective result and says so.
 *
 * ⚠️ **`SCHEDULE_EXCEPTION` stays location-wide, deliberately.** A public holiday
 * is a fact about the building. "This machine is down on the 26th" is
 * maintenance, which is a different feature with its own screen, and folding it
 * in here would make the two impossible to tell apart in a report.
 *
 * Expand-only: every existing row has `scopeType IS NULL` and keeps meaning
 * exactly what it meant.
 */
final class Version20260819120000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S134d: OPENING_HOUR.scopeType/scopeId — schedules attachable to a resource kind or one resource. Expand-only.';
    }

    public function up(Schema $schema): void
    {
        foreach ([
            'scopeType' => 'VARCHAR(20) DEFAULT NULL',
            'scopeId' => 'INT DEFAULT NULL',
        ] as $column => $definition) {
            $this->addSql(sprintf(
                "SET @s := IF(
                    (SELECT COUNT(*) FROM INFORMATION_SCHEMA.COLUMNS
                     WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = 'OPENING_HOUR' AND COLUMN_NAME = '%s') > 0,
                    'SELECT 1',
                    'ALTER TABLE OPENING_HOUR ADD %s %s'
                )",
                $column,
                $column,
                $definition,
            ));
            $this->addSql('PREPARE stmt FROM @s');
            $this->addSql('EXECUTE stmt');
            $this->addSql('DEALLOCATE PREPARE stmt');
        }

        // The resolver asks for one scope at a time, so the lookup is
        // (venue, scope, day) and should stay an index seek.
        $this->addSql(<<<'SQL'
            SET @s := IF(
                (SELECT COUNT(*) FROM INFORMATION_SCHEMA.STATISTICS
                 WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = 'OPENING_HOUR'
                   AND INDEX_NAME = 'IDX_OPENING_HOUR_SCOPE') > 0,
                'SELECT 1',
                'ALTER TABLE OPENING_HOUR ADD INDEX IDX_OPENING_HOUR_SCOPE (venueId, scopeType, scopeId, dayOfWeek)'
            )
        SQL);
        $this->addSql('PREPARE stmt FROM @s');
        $this->addSql('EXECUTE stmt');
        $this->addSql('DEALLOCATE PREPARE stmt');
    }

    /**
     * ⚠️ Dropping the columns deletes every schedule an operator attached below
     * the location. The location's own week — every row with `scopeType IS NULL`
     * — is untouched, so the lab keeps working; it simply loses the narrower
     * hours it had written.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('DELETE FROM OPENING_HOUR WHERE scopeType IS NOT NULL');
        $this->addSql('ALTER TABLE OPENING_HOUR DROP INDEX IDX_OPENING_HOUR_SCOPE');
        $this->addSql('ALTER TABLE OPENING_HOUR DROP COLUMN scopeType');
        $this->addSql('ALTER TABLE OPENING_HOUR DROP COLUMN scopeId');
    }
}
