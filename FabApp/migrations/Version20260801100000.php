<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Event registration, with a waitlist and open to people without an account.
 *
 * Two decisions are worth reading before changing this table.
 *
 * **contactEmail exists to make dedupe possible at all.** Registration is open
 * to guests, who have no account to key on, so nothing would stop the same
 * person signing up five times. Every row therefore carries a normalised
 * address — copied from the account for members, typed in by guests — with a
 * unique index per event. That is the only thing standing between an open
 * signup form and a nonsense attendee list.
 *
 * **Capacity is counted, not stored as "seats left".** A remaining-seats column
 * would have to be kept in step with every registration, cancellation and
 * promotion, and would drift the first time one of those paths threw halfway.
 * The count is derived from the rows instead, inside the same transaction that
 * inserts, so two people racing for the final seat cannot both win it.
 */
final class Version20260801100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add EVENT_REGISTRATION (guests welcome, waitlisted overflow) and EVENEMENT.capacite.';
    }

    public function up(Schema $schema): void
    {
        // Null capacity = unlimited, consistent with how quotas and passes read.
        $this->addSql('ALTER TABLE EVENEMENT ADD capacite INT DEFAULT NULL');

        // Whether people without an account may take a place. Defaults to 1
        // because open registration is the model this table was built for; an
        // organiser running a members-only event unticks it deliberately. Note
        // this restricts *new* signups only — existing guest rows on an event
        // later closed to guests keep their places rather than being silently
        // dropped from an attendee list the organiser may already have printed.
        $this->addSql('ALTER TABLE EVENEMENT ADD guestsAllowed TINYINT(1) DEFAULT 1 NOT NULL');

        // Calling an event off is a state, not a delete: the registrations stay,
        // so the organiser keeps the list of who had to be told, and anyone
        // following an old link gets an explanation instead of a 404. The reason
        // is stored rather than only mailed, so the page can show it too.
        $this->addSql('ALTER TABLE EVENEMENT ADD cancelledAt DATETIME DEFAULT NULL, ADD cancellationReason VARCHAR(500) DEFAULT NULL');

        $this->addSql('CREATE TABLE IF NOT EXISTS EVENT_REGISTRATION (
            id INT AUTO_INCREMENT NOT NULL,
            eventId INT NOT NULL,
            userId INT DEFAULT NULL,
            guestName VARCHAR(180) DEFAULT NULL,
            contactEmail VARCHAR(180) NOT NULL,
            status VARCHAR(20) NOT NULL,
            createdAt DATETIME NOT NULL,
            cancelledAt DATETIME DEFAULT NULL,
            promotedAt DATETIME DEFAULT NULL,
            INDEX IDX_EVENT_REG_EVENT (eventId),
            INDEX IDX_EVENT_REG_USER (userId),
            INDEX IDX_EVENT_REG_STATUS (eventId, status),
            PRIMARY KEY(id)
        ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');

        // One signup per address per event. Cancelled rows keep their slot in the
        // index on purpose: re-registering reuses the existing row (see the
        // service), which preserves the original audit trail instead of piling
        // up a row per change of mind.
        $this->addSql('CREATE UNIQUE INDEX UNIQ_EVENT_REG_CONTACT ON EVENT_REGISTRATION (eventId, contactEmail)');

        $this->addSql('ALTER TABLE EVENT_REGISTRATION ADD CONSTRAINT FK_EVENT_REG_EVENT FOREIGN KEY (eventId) REFERENCES EVENEMENT (id) ON DELETE CASCADE');
        // A deleted account leaves its registration behind as a guest-shaped row:
        // the organiser still needs an accurate head count for an event that may
        // already have happened.
        $this->addSql('ALTER TABLE EVENT_REGISTRATION ADD CONSTRAINT FK_EVENT_REG_USER FOREIGN KEY (userId) REFERENCES UTILISATEUR (id) ON DELETE SET NULL');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE EVENT_REGISTRATION DROP FOREIGN KEY FK_EVENT_REG_EVENT');
        $this->addSql('ALTER TABLE EVENT_REGISTRATION DROP FOREIGN KEY FK_EVENT_REG_USER');
        $this->addSql('DROP TABLE IF EXISTS EVENT_REGISTRATION');
        $this->addSql('ALTER TABLE EVENEMENT DROP cancelledAt, DROP cancellationReason');
        $this->addSql('ALTER TABLE EVENEMENT DROP guestsAllowed');
        $this->addSql('ALTER TABLE EVENEMENT DROP capacite');
    }
}
