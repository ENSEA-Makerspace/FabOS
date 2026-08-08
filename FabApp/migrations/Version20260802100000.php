<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Door check-in: who actually turned up, as opposed to who said they would.
 *
 * Kept as its own timestamp rather than another `status` value, because
 * attendance is **orthogonal** to registration. A status column can only hold
 * one truth at a time, and "registered" and "arrived" are two different facts
 * about the same person — collapsing them would make it impossible to tell a
 * no-show (registered, never checked in) from someone who cancelled, which is
 * precisely the distinction an organiser wants afterwards.
 *
 * The timestamp doubles as the flag: null means not yet, set means arrived, and
 * undoing a mistaken tap just nulls it again. checkedInById records which staff
 * member was on the door, so a disputed entry has an answer.
 */
final class Version20260802100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add EVENT_REGISTRATION.checkedInAt/checkedInById for door check-in.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE EVENT_REGISTRATION ADD checkedInAt DATETIME DEFAULT NULL, ADD checkedInById INT DEFAULT NULL');
        $this->addSql('CREATE INDEX IDX_EVENT_REG_CHECKIN ON EVENT_REGISTRATION (eventId, checkedInAt)');

        // The staff member is a reference, not an owner: losing their account
        // must not erase the record that someone was admitted.
        $this->addSql('ALTER TABLE EVENT_REGISTRATION ADD CONSTRAINT FK_EVENT_REG_CHECKIN_BY FOREIGN KEY (checkedInById) REFERENCES UTILISATEUR (id) ON DELETE SET NULL');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE EVENT_REGISTRATION DROP FOREIGN KEY FK_EVENT_REG_CHECKIN_BY');
        $this->addSql('DROP INDEX IDX_EVENT_REG_CHECKIN ON EVENT_REGISTRATION');
        $this->addSql('ALTER TABLE EVENT_REGISTRATION DROP checkedInAt, DROP checkedInById');
    }
}
