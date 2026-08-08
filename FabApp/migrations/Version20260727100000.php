<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Mail backbone: the audit log every outgoing mail is written to, and the
 * Messenger queue table its async sends go through (the transport is configured
 * with auto_setup=0, so the table has to exist up front).
 *
 * Also widens SITE_SETTING.settingValue, which was VARCHAR(50) — too narrow for
 * a transport DSN, and already too narrow for the lab-rules HTML it holds today.
 */
final class Version20260727100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add EMAIL_LOG and messenger_messages, and widen SITE_SETTING.settingValue to TEXT.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE IF NOT EXISTS EMAIL_LOG (id INT AUTO_INCREMENT NOT NULL, portalId INT DEFAULT 0 NOT NULL, category VARCHAR(50) NOT NULL, recipient VARCHAR(180) NOT NULL, recipientName VARCHAR(180) DEFAULT NULL, locale VARCHAR(5) NOT NULL, template VARCHAR(100) NOT NULL, contextJson LONGTEXT NOT NULL, subject VARCHAR(255) DEFAULT NULL, status VARCHAR(20) NOT NULL, error VARCHAR(500) DEFAULT NULL, queuedAt DATETIME NOT NULL, sentAt DATETIME DEFAULT NULL, INDEX IDX_EMAIL_LOG_QUEUED (queuedAt), INDEX IDX_EMAIL_LOG_STATUS (status), INDEX IDX_EMAIL_LOG_RECIPIENT (recipient), PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');

        // Standard Doctrine-transport table; MESSENGER_TRANSPORT_DSN uses auto_setup=0.
        $this->addSql('CREATE TABLE IF NOT EXISTS messenger_messages (id BIGINT AUTO_INCREMENT NOT NULL, body LONGTEXT NOT NULL, headers LONGTEXT NOT NULL, queue_name VARCHAR(190) NOT NULL, created_at DATETIME NOT NULL, available_at DATETIME NOT NULL, delivered_at DATETIME DEFAULT NULL, INDEX IDX_75EA56E0FB7336F0 (queue_name), INDEX IDX_75EA56E0E3BD61CE (available_at), INDEX IDX_75EA56E016BA31DB (delivered_at), PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');

        $this->addSql('ALTER TABLE SITE_SETTING MODIFY settingValue TEXT NOT NULL');
    }

    public function down(Schema $schema): void
    {
        // Values longer than the old limit would be truncated on the way back down.
        $this->addSql('UPDATE SITE_SETTING SET settingValue = LEFT(settingValue, 50)');
        $this->addSql('ALTER TABLE SITE_SETTING MODIFY settingValue VARCHAR(50) NOT NULL');
        $this->addSql('DROP TABLE IF EXISTS messenger_messages');
        $this->addSql('DROP TABLE IF EXISTS EMAIL_LOG');
    }
}
