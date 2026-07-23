<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Add the MACHINE_MATERIAL join table linking materials to the machines that
 * accept them (Material ↔ Machine compatibility). Runs after the MATERIAL
 * table migration (Version20260723140000).
 */
final class Version20260723150000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add MACHINE_MATERIAL join table (materials a machine accepts).';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE MACHINE_MATERIAL (materialId INT NOT NULL, machineId INT NOT NULL, INDEX IDX_MACHINE_MATERIAL_MATERIAL (materialId), INDEX IDX_MACHINE_MATERIAL_MACHINE (machineId), PRIMARY KEY(materialId, machineId)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('ALTER TABLE MACHINE_MATERIAL ADD CONSTRAINT FK_MACHINE_MATERIAL_MATERIAL FOREIGN KEY (materialId) REFERENCES MATERIAL (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE MACHINE_MATERIAL ADD CONSTRAINT FK_MACHINE_MATERIAL_MACHINE FOREIGN KEY (machineId) REFERENCES MACHINE (id) ON DELETE CASCADE');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE MACHINE_MATERIAL');
    }
}
