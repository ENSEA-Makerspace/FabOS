<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Scheduled reminders: the record of which reminder has already gone out.
 *
 * The scanner runs on a timer and re-examines the same rows every hour, so
 * "have I already told them about this?" is the only thing standing between a
 * helpful reminder and a mailbox full of duplicates. The unique key on
 * reminderKey is what enforces that — the claim is an INSERT that either wins
 * or collides, which stays correct even with two workers running at once.
 */
final class Version20260728100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add MAIL_REMINDER, the already-sent ledger for scheduled reminder mail.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE IF NOT EXISTS MAIL_REMINDER (id INT AUTO_INCREMENT NOT NULL, reminderKey VARCHAR(190) NOT NULL, kind VARCHAR(50) NOT NULL, sentAt DATETIME NOT NULL, UNIQUE INDEX UNIQ_MAIL_REMINDER_KEY (reminderKey), INDEX IDX_MAIL_REMINDER_KIND (kind), INDEX IDX_MAIL_REMINDER_SENT (sentAt), PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS MAIL_REMINDER');
    }
}
