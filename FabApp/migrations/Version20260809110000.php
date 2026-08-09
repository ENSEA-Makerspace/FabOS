<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

final class Version20260809110000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Create the default physical venue and scope opening hours to it.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql("CREATE TABLE VENUE (id INT AUTO_INCREMENT NOT NULL, slug VARCHAR(80) NOT NULL, name VARCHAR(140) NOT NULL, address VARCHAR(255) DEFAULT NULL, timezone VARCHAR(64) NOT NULL, active TINYINT(1) NOT NULL DEFAULT 1, UNIQUE INDEX UNIQ_VENUE_SLUG (slug), PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB");
        $this->addSql("INSERT INTO VENUE (slug, name, address, timezone, active) SELECT 'default', COALESCE(NULLIF(MAX(CASE WHEN settingKey = 'venue_label' THEN settingValue END), ''), 'Lieu principal'), NULLIF(MAX(CASE WHEN settingKey = 'lab_address' THEN settingValue END), ''), COALESCE(NULLIF(MAX(CASE WHEN settingKey = 'timezone' THEN settingValue END), ''), 'Europe/Paris'), 1 FROM SITE_SETTING WHERE portalId = 0");
        $this->addSql('ALTER TABLE OPENING_HOUR ADD venueId INT DEFAULT NULL');
        $this->addSql("UPDATE OPENING_HOUR SET venueId = (SELECT id FROM VENUE WHERE slug = 'default') WHERE venueId IS NULL");
        $this->addSql('ALTER TABLE OPENING_HOUR MODIFY venueId INT NOT NULL');
        $this->addSql('ALTER TABLE OPENING_HOUR DROP INDEX uniq_opening_hour_day_of_week');
        $this->addSql('ALTER TABLE OPENING_HOUR ADD INDEX IDX_OPENING_HOUR_VENUE (venueId), ADD UNIQUE INDEX UNIQ_OPENING_HOUR_VENUE_DAY (venueId, dayOfWeek), ADD CONSTRAINT FK_OPENING_HOUR_VENUE FOREIGN KEY (venueId) REFERENCES VENUE (id)');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE OPENING_HOUR DROP FOREIGN KEY FK_OPENING_HOUR_VENUE');
        $this->addSql('ALTER TABLE OPENING_HOUR DROP INDEX UNIQ_OPENING_HOUR_VENUE_DAY, DROP INDEX IDX_OPENING_HOUR_VENUE, DROP COLUMN venueId');
        $this->addSql('ALTER TABLE OPENING_HOUR ADD UNIQUE INDEX uniq_opening_hour_day_of_week (dayOfWeek)');
        $this->addSql('DROP TABLE VENUE');
    }
}
