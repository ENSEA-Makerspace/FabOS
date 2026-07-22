<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Add the EVENEMENT table backing the Event module (admin-managed, display-only
 * events shown on the home page, calendar and a kiosk screen). Named EVENEMENT
 * because EVENT is a reserved word in MySQL.
 */
final class Version20260722170000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add EVENEMENT table for the Event module.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE EVENEMENT (id INT AUTO_INCREMENT NOT NULL, titre VARCHAR(180) NOT NULL, description LONGTEXT DEFAULT NULL, dateDebut DATETIME NOT NULL, dateFin DATETIME DEFAULT NULL, lieu VARCHAR(180) DEFAULT NULL, createdAt DATETIME NOT NULL, PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('CREATE INDEX IDX_EVENEMENT_DATEDEBUT ON EVENEMENT (dateDebut)');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE EVENEMENT');
    }
}
