<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Add whereRecognized and a self-referencing prerequisiteBadgeId to BADGE,
 * backing the new badge detail page (recognition scope + lvl1/lvl2-style unlock chain).
 */
final class Version20260722130000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add whereRecognized and prerequisiteBadgeId columns to BADGE.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE BADGE ADD whereRecognized VARCHAR(255) DEFAULT NULL, ADD prerequisiteBadgeId INT DEFAULT NULL');
        $this->addSql('ALTER TABLE BADGE ADD CONSTRAINT FK_BADGE_PREREQUISITE FOREIGN KEY (prerequisiteBadgeId) REFERENCES BADGE (id) ON DELETE SET NULL');
        $this->addSql('CREATE INDEX IDX_BADGE_PREREQUISITE ON BADGE (prerequisiteBadgeId)');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE BADGE DROP FOREIGN KEY FK_BADGE_PREREQUISITE');
        $this->addSql('DROP INDEX IDX_BADGE_PREREQUISITE ON BADGE');
        $this->addSql('ALTER TABLE BADGE DROP whereRecognized, DROP prerequisiteBadgeId');
    }
}
