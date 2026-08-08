<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Add LAB_PAGE and LAB_PAGE_IMAGE tables for the new admin-toggleable
 * Lab pages module: a fixed 2-level page hierarchy with a simple photo
 * gallery per page.
 */
final class Version20260722150000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add LAB_PAGE and LAB_PAGE_IMAGE tables.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE LAB_PAGE (id INT AUTO_INCREMENT NOT NULL, parentPageId INT DEFAULT NULL, titre VARCHAR(255) NOT NULL, contenu LONGTEXT DEFAULT NULL, position INT NOT NULL, createdAt DATETIME NOT NULL, updatedAt DATETIME DEFAULT NULL, INDEX IDX_LAB_PAGE_PARENT (parentPageId), PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('ALTER TABLE LAB_PAGE ADD CONSTRAINT FK_LAB_PAGE_PARENT FOREIGN KEY (parentPageId) REFERENCES LAB_PAGE (id) ON DELETE CASCADE');
        $this->addSql('CREATE TABLE LAB_PAGE_IMAGE (id INT AUTO_INCREMENT NOT NULL, labPageId INT NOT NULL, imageFilename VARCHAR(255) NOT NULL, createdAt DATETIME NOT NULL, INDEX IDX_LAB_PAGE_IMAGE_PAGE (labPageId), PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('ALTER TABLE LAB_PAGE_IMAGE ADD CONSTRAINT FK_LAB_PAGE_IMAGE_PAGE FOREIGN KEY (labPageId) REFERENCES LAB_PAGE (id) ON DELETE CASCADE');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE LAB_PAGE_IMAGE DROP FOREIGN KEY FK_LAB_PAGE_IMAGE_PAGE');
        $this->addSql('DROP TABLE LAB_PAGE_IMAGE');
        $this->addSql('ALTER TABLE LAB_PAGE DROP FOREIGN KEY FK_LAB_PAGE_PARENT');
        $this->addSql('DROP TABLE LAB_PAGE');
    }
}
