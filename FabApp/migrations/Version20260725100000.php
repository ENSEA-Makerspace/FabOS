<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Bookable people: weekly availability windows plus the per-user booking
 * settings the person's booking page reads. Additive only — nobody is bookable
 * until an admin flips UTILISATEUR.bookable, so this deploys ahead of the code.
 */
final class Version20260725100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add USER_AVAILABILITY and the UTILISATEUR booking settings for bookable people.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE USER_AVAILABILITY (id INT AUTO_INCREMENT NOT NULL, userId INT NOT NULL, dayOfWeek SMALLINT NOT NULL, startTime TIME NOT NULL, endTime TIME NOT NULL, INDEX IDX_USER_AVAILABILITY_USER (userId, dayOfWeek), PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('ALTER TABLE USER_AVAILABILITY ADD CONSTRAINT FK_USER_AVAILABILITY_USER FOREIGN KEY (userId) REFERENCES UTILISATEUR (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE UTILISATEUR ADD bookable TINYINT(1) DEFAULT 0 NOT NULL, ADD bookingDurations VARCHAR(60) DEFAULT NULL, ADD bookingNote VARCHAR(500) DEFAULT NULL');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE UTILISATEUR DROP bookable, DROP bookingDurations, DROP bookingNote');
        $this->addSql('DROP TABLE USER_AVAILABILITY');
    }
}
