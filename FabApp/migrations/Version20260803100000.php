<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Poster artwork and a real location for events.
 *
 * `locationMode` exists because "where is it?" has two genuinely different
 * answers that need different treatment. **On site** the address is the lab's
 * own and should not be retyped per event — it lives once in a site setting, so
 * moving premises means editing one field rather than every event ever created.
 * **Off site** the address belongs to the event, and is the only case where a
 * per-event address is worth storing.
 *
 * Both modes want directions, which are derived from whichever address applies
 * rather than stored: a map URL saved alongside an address is one more thing to
 * keep in step, and it goes stale the moment someone edits the address.
 *
 * `lieu` is kept as the human label ("Grande salle", "Chez Machin") and is now
 * complemented by, not replaced by, the address.
 */
final class Version20260803100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add EVENEMENT.posterFilename, locationMode and address.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql("ALTER TABLE EVENEMENT
            ADD posterFilename VARCHAR(255) DEFAULT NULL,
            ADD locationMode VARCHAR(20) DEFAULT 'onsite' NOT NULL,
            ADD address VARCHAR(500) DEFAULT NULL");
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE EVENEMENT DROP posterFilename, DROP locationMode, DROP address');
    }
}
