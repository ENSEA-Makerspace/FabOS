<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Replace the single Badge.whereRecognized string with a proper INSTITUTION
 * entity (name + link), many-to-many with BADGE, so a badge can be recognized
 * by several institutions instead of one free-text line.
 */
final class Version20260722140000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add INSTITUTION and BADGE_INSTITUTION tables, drop BADGE.whereRecognized.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE INSTITUTION (id INT AUTO_INCREMENT NOT NULL, nom VARCHAR(255) NOT NULL, url VARCHAR(500) DEFAULT NULL, createdAt DATETIME NOT NULL, PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('CREATE TABLE BADGE_INSTITUTION (badge_id INT NOT NULL, institution_id INT NOT NULL, INDEX IDX_BADGE_INSTITUTION_BADGE (badge_id), INDEX IDX_BADGE_INSTITUTION_INSTITUTION (institution_id), PRIMARY KEY(badge_id, institution_id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('ALTER TABLE BADGE_INSTITUTION ADD CONSTRAINT FK_BADGE_INSTITUTION_BADGE FOREIGN KEY (badge_id) REFERENCES BADGE (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE BADGE_INSTITUTION ADD CONSTRAINT FK_BADGE_INSTITUTION_INSTITUTION FOREIGN KEY (institution_id) REFERENCES INSTITUTION (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE BADGE DROP whereRecognized');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE BADGE ADD whereRecognized VARCHAR(255) DEFAULT NULL');
        $this->addSql('DROP TABLE BADGE_INSTITUTION');
        $this->addSql('DROP TABLE INSTITUTION');
    }
}
