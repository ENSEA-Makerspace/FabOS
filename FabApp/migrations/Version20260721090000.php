<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Generic site settings key/value store (currently: default_locale).
 */
final class Version20260721090000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Create SITE_SETTING table for site-wide settings (default locale).';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE IF NOT EXISTS SITE_SETTING (settingKey VARCHAR(50) NOT NULL, settingValue VARCHAR(50) NOT NULL, PRIMARY KEY (settingKey)) DEFAULT CHARACTER SET utf8mb4');
        $this->addSql("INSERT IGNORE INTO SITE_SETTING (settingKey, settingValue) VALUES ('default_locale', 'fr')");
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS SITE_SETTING');
    }
}
