<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Add the MATERIAL table backing the Materials catalogue module — a shared
 * catalogue of significant lab materials (filament, sheet stock, resin…).
 * Additive only; the machine/training M2M links come in a later slice.
 */
final class Version20260723140000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add MATERIAL table for the Materials catalogue module.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE MATERIAL (id INT AUTO_INCREMENT NOT NULL, name VARCHAR(150) NOT NULL, category VARCHAR(80) DEFAULT NULL, description LONGTEXT DEFAULT NULL, imageUrl VARCHAR(500) DEFAULT NULL, icon VARCHAR(16) DEFAULT NULL, specs LONGTEXT DEFAULT NULL, storageLocation VARCHAR(180) DEFAULT NULL, purchaseUrl VARCHAR(500) DEFAULT NULL, color VARCHAR(60) DEFAULT NULL, createdAt DATETIME NOT NULL, PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('CREATE INDEX IDX_MATERIAL_CATEGORY ON MATERIAL (category)');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE MATERIAL');
    }
}
