<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Add PLACE table and generalize RESERVATION to point at either a
 * Machine or a Place: RESERVATION.machineId becomes nullable and a new
 * nullable placeId FK is added, so machine and place bookings share the
 * same reservation flow, conflict-checking, and "my reservations" list.
 */
final class Version20260722160000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add PLACE table; make RESERVATION.machineId nullable and add RESERVATION.placeId.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE PLACE (id INT AUTO_INCREMENT NOT NULL, nom VARCHAR(150) NOT NULL, description LONGTEXT DEFAULT NULL, localisation VARCHAR(150) DEFAULT NULL, capacite INT DEFAULT NULL, createdAt DATETIME NOT NULL, PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('ALTER TABLE RESERVATION MODIFY machineId INT DEFAULT NULL');
        $this->addSql('ALTER TABLE RESERVATION ADD placeId INT DEFAULT NULL');
        $this->addSql('ALTER TABLE RESERVATION ADD CONSTRAINT FK_RESERVATION_PLACE FOREIGN KEY (placeId) REFERENCES PLACE (id) ON DELETE CASCADE');
        $this->addSql('CREATE INDEX IDX_RESERVATION_PLACE ON RESERVATION (placeId)');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE RESERVATION DROP FOREIGN KEY FK_RESERVATION_PLACE');
        $this->addSql('DROP INDEX IDX_RESERVATION_PLACE ON RESERVATION');
        $this->addSql('ALTER TABLE RESERVATION DROP placeId');
        $this->addSql('ALTER TABLE RESERVATION MODIFY machineId INT NOT NULL');
        $this->addSql('DROP TABLE PLACE');
    }
}
