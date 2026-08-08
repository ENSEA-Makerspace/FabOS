<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Add a tags column to CREATION (comma-separated list).
 */
final class Version20260720130000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add tags column to CREATION.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE CREATION ADD tags VARCHAR(255) DEFAULT NULL');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE CREATION DROP tags');
    }
}
