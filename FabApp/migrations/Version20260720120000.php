<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Optional site modules (badges / formations / leaderboard / projects) on-off flags.
 */
final class Version20260720120000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Create SITE_MODULE table for toggling optional site modules.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE IF NOT EXISTS SITE_MODULE (moduleKey VARCHAR(50) NOT NULL, enabled TINYINT(1) NOT NULL DEFAULT 1, PRIMARY KEY (moduleKey)) DEFAULT CHARACTER SET utf8mb4');
        foreach (['leaderboard', 'projects', 'badges', 'formations'] as $key) {
            $this->addSql("INSERT IGNORE INTO SITE_MODULE (moduleKey, enabled) VALUES ('$key', 1)");
        }
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS SITE_MODULE');
    }
}
