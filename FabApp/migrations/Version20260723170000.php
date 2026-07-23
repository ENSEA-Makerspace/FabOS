<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Add the MAINTENANCE_TASK table backing the Maintenance module — a per-machine
 * due/overdue task backlog (non-blocking v1). Additive only.
 */
final class Version20260723170000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add MAINTENANCE_TASK table for the Maintenance module.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE MAINTENANCE_TASK (id INT AUTO_INCREMENT NOT NULL, machineId INT NOT NULL, title VARCHAR(180) NOT NULL, type VARCHAR(20) DEFAULT \'preventive\' NOT NULL, link VARCHAR(500) DEFAULT NULL, dueDate DATE DEFAULT NULL, status VARCHAR(20) DEFAULT \'pending\' NOT NULL, doneDate DATE DEFAULT NULL, doneById INT DEFAULT NULL, notes LONGTEXT DEFAULT NULL, recurrenceDays INT DEFAULT NULL, createdAt DATETIME NOT NULL, INDEX IDX_MAINTENANCE_MACHINE (machineId), INDEX IDX_MAINTENANCE_STATUS (status), PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('ALTER TABLE MAINTENANCE_TASK ADD CONSTRAINT FK_MAINTENANCE_MACHINE FOREIGN KEY (machineId) REFERENCES MACHINE (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE MAINTENANCE_TASK ADD CONSTRAINT FK_MAINTENANCE_DONEBY FOREIGN KEY (doneById) REFERENCES UTILISATEUR (id) ON DELETE SET NULL');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE MAINTENANCE_TASK');
    }
}
