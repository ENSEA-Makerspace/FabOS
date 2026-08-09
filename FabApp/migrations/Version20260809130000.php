<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

final class Version20260809130000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Store the member preferred venue as a display preference.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE UTILISATEUR ADD preferredVenueId INT DEFAULT NULL');
        $this->addSql('ALTER TABLE UTILISATEUR ADD INDEX IDX_UTILISATEUR_PREFERRED_VENUE (preferredVenueId), ADD CONSTRAINT FK_UTILISATEUR_PREFERRED_VENUE FOREIGN KEY (preferredVenueId) REFERENCES VENUE (id) ON DELETE SET NULL');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE UTILISATEUR DROP FOREIGN KEY FK_UTILISATEUR_PREFERRED_VENUE');
        $this->addSql('ALTER TABLE UTILISATEUR DROP INDEX IDX_UTILISATEUR_PREFERRED_VENUE, DROP COLUMN preferredVenueId');
    }
}
