<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Per-category notification preferences, and the link back from a logged mail
 * to the person it was for.
 *
 * The opt-out table stores *absences of consent* rather than consent: a row
 * means "don't send me this kind", and having no row means the default applies.
 * Most people never change their settings, so the table stays small, and a new
 * category becomes available to everyone without backfilling a row per user —
 * which a table of positive preferences would have required.
 *
 * EMAIL_LOG.userId is what lets the renderer put a working unsubscribe link in
 * the footer: the worker renders from the log row alone, and until now that row
 * knew an address but not an account.
 */
final class Version20260729100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add USER_NOTIFICATION_OPTOUT and EMAIL_LOG.userId for per-category mail preferences.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE IF NOT EXISTS USER_NOTIFICATION_OPTOUT (userId INT NOT NULL, category VARCHAR(50) NOT NULL, optedOutAt DATETIME NOT NULL, INDEX IDX_USER_NOTIF_OPTOUT_USER (userId), PRIMARY KEY(userId, category)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');

        // A deleted account should take its preferences with it.
        $this->addSql('ALTER TABLE USER_NOTIFICATION_OPTOUT ADD CONSTRAINT FK_USER_NOTIF_OPTOUT_USER FOREIGN KEY (userId) REFERENCES UTILISATEUR (id) ON DELETE CASCADE');

        // Nullable on purpose: mail to a bare address (a walk-in borrower with no
        // account) has no user to point at, and must still be logged.
        $this->addSql('ALTER TABLE EMAIL_LOG ADD userId INT DEFAULT NULL');
        $this->addSql('CREATE INDEX IDX_EMAIL_LOG_USER ON EMAIL_LOG (userId)');

        // UTILISATEUR.rappelReservation has been settable from the profile and the
        // admin since long before anything read it — no code has ever consulted it,
        // so people who switched booking reminders off were quietly reminded anyway
        // once S15 shipped. The reminder category is the switch that now works;
        // carry the old intent across rather than resetting those people to "yes".
        $this->addSql('INSERT IGNORE INTO USER_NOTIFICATION_OPTOUT (userId, category, optedOutAt) SELECT id, \'reminder\', NOW() FROM UTILISATEUR WHERE rappelReservation = 0');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP INDEX IDX_EMAIL_LOG_USER ON EMAIL_LOG');
        $this->addSql('ALTER TABLE EMAIL_LOG DROP userId');
        $this->addSql('ALTER TABLE USER_NOTIFICATION_OPTOUT DROP FOREIGN KEY FK_USER_NOTIF_OPTOUT_USER');
        $this->addSql('DROP TABLE IF EXISTS USER_NOTIFICATION_OPTOUT');
    }
}
