<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S146f + S146d: an event says what KIND it is, and whose SÉANCE it is.
 *
 * Two different questions, deliberately answered by two different columns — the
 * operator's decision, 2026-08-20, and the distinction is the whole point:
 *
 *  - `categoryId` is a **label**: atelier, séance de formation, portes ouvertes.
 *    It is the lab's own vocabulary, it is descriptive, and **no code may ever
 *    branch on a particular category**. The moment something reads
 *    `category.slug === 'formation'` to decide behaviour, an operator renaming a
 *    row breaks the product, and the label has quietly become an enum.
 *  - `formationId` is a **link**: this event IS a session of that training. It is
 *    what lets a training page list its real next sessions — the hole S134c2 left
 *    when it had to delete a "three next sessions" block that invented them — and
 *    what lets registering for a session mean something (S146e).
 *
 * 🔴 **A category cannot do the link's job.** "Séance de formation" does not say
 * WHICH training, so a category alone leaves both of those broken. That is why
 * this migration carries both and not one.
 *
 * ⚠️ **The table is `EVENEMENT`, not `EVENT`.** The entity is `App\Entity\Event`
 * and reads `#[ORM\Table(name: 'EVENEMENT')]`; writing the class name into the SQL
 * would fail on a live database and pass every static check first.
 *
 * ⚠️ **Both columns are MAPPED ORM columns, so this migration ships BEFORE the
 * code that reads them.** Every Doctrine query on `Event` selects every mapped
 * column, so no repository try/catch can degrade: deploying the entity first is a
 * 500 on every page that lists an event. That is exactly what `OpeningHour.scopeType`
 * did in S134d. The fail-safe-DBAL-behind-a-probe branch does NOT apply here.
 *
 * ⚠️ **Both are nullable, and both are `ON DELETE SET NULL`.** An event with no
 * category is an event, not an error — and archiving a category or a training must
 * never delete somebody's event. Expand only: nothing is dropped or backfilled, so
 * the running code is unaffected until the code that reads them arrives.
 *
 * ⚠️ **No categories are seeded.** They are content, in the lab's own words and
 * language, and a migration that invents "Atelier" hands every install a French row
 * it did not ask for. The admin screen creates them; an install with none simply
 * shows no category chip.
 */
final class Version20260820100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S146f/S146d: EVENT_CATEGORY, plus EVENEMENT.categoryId and EVENEMENT.formationId (expand only).';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS EVENT_CATEGORY (
                id INT AUTO_INCREMENT NOT NULL,
                label VARCHAR(100) NOT NULL,
                slug VARCHAR(120) NOT NULL,
                iconSlug VARCHAR(50) DEFAULT NULL,
                position INT DEFAULT 0 NOT NULL,
                archivedAt DATETIME DEFAULT NULL COMMENT '(DC2Type:datetime_immutable)',
                createdAt DATETIME NOT NULL COMMENT '(DC2Type:datetime_immutable)',
                UNIQUE INDEX UNIQ_EVENT_CATEGORY_SLUG (slug),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);

        // ⚠️ Guarded one statement at a time rather than in one ALTER: a re-run
        // after a partial failure must be able to add only what is missing.
        $this->addColumnIfMissing('EVENEMENT', 'categoryId', 'INT DEFAULT NULL');
        $this->addColumnIfMissing('EVENEMENT', 'formationId', 'INT DEFAULT NULL');

        $this->addIndexIfMissing('EVENEMENT', 'IDX_EVENEMENT_CATEGORY', 'categoryId');
        $this->addIndexIfMissing('EVENEMENT', 'IDX_EVENEMENT_FORMATION', 'formationId');

        $this->addForeignKeyIfMissing(
            'EVENEMENT',
            'FK_EVENEMENT_CATEGORY',
            'categoryId',
            'EVENT_CATEGORY',
        );
        $this->addForeignKeyIfMissing(
            'EVENEMENT',
            'FK_EVENEMENT_FORMATION',
            'formationId',
            'FORMATION',
        );
    }

    /**
     * ⚠️ Down drops the two columns and the table. It is safe here only because
     * this is an expand-only step: nothing has been backfilled out of another
     * column, so nothing is lost that was not entered against these columns.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE EVENEMENT DROP FOREIGN KEY IF EXISTS FK_EVENEMENT_CATEGORY');
        $this->addSql('ALTER TABLE EVENEMENT DROP FOREIGN KEY IF EXISTS FK_EVENEMENT_FORMATION');
        $this->addSql('ALTER TABLE EVENEMENT DROP COLUMN IF EXISTS categoryId');
        $this->addSql('ALTER TABLE EVENEMENT DROP COLUMN IF EXISTS formationId');
        $this->addSql('DROP TABLE IF EXISTS EVENT_CATEGORY');
    }

    private function addColumnIfMissing(string $table, string $column, string $definition): void
    {
        $exists = (int) $this->connection->fetchOne(
            'SELECT COUNT(*) FROM information_schema.COLUMNS
             WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = ? AND COLUMN_NAME = ?',
            [$table, $column],
        );

        if ($exists === 0) {
            $this->addSql(sprintf('ALTER TABLE %s ADD %s %s', $table, $column, $definition));
        }
    }

    private function addIndexIfMissing(string $table, string $index, string $column): void
    {
        $exists = (int) $this->connection->fetchOne(
            'SELECT COUNT(*) FROM information_schema.STATISTICS
             WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = ? AND INDEX_NAME = ?',
            [$table, $index],
        );

        if ($exists === 0) {
            $this->addSql(sprintf('CREATE INDEX %s ON %s (%s)', $index, $table, $column));
        }
    }

    private function addForeignKeyIfMissing(string $table, string $name, string $column, string $target): void
    {
        $exists = (int) $this->connection->fetchOne(
            'SELECT COUNT(*) FROM information_schema.TABLE_CONSTRAINTS
             WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = ? AND CONSTRAINT_NAME = ?',
            [$table, $name],
        );

        if ($exists === 0) {
            $this->addSql(sprintf(
                'ALTER TABLE %s ADD CONSTRAINT %s FOREIGN KEY (%s) REFERENCES %s (id) ON DELETE SET NULL',
                $table,
                $name,
                $column,
                $target,
            ));
        }
    }
}
