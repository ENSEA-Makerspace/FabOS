<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Add an isPinned column to CREATION so admins can pin creations to the top of the gallery.
 */
final class Version20260722110000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add isPinned column to CREATION.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE CREATION ADD isPinned TINYINT(1) NOT NULL DEFAULT 0');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE CREATION DROP isPinned');
    }
}
