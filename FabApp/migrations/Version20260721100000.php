<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Hide the "Machines à découvrir" homepage section for everyone (admin decision:
 * the machines listing is being removed from the home page).
 */
final class Version20260721100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Hide the featured_machines homepage section by default.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql("UPDATE HOMEPAGE_SECTION_VISIBILITY SET visibleAnonymous = 0, visibleUser = 0, visibleStaff = 0, visibleAdmin = 0 WHERE sectionKey = 'featured_machines'");
    }

    public function down(Schema $schema): void
    {
        $this->addSql("UPDATE HOMEPAGE_SECTION_VISIBILITY SET visibleAnonymous = 1, visibleUser = 1, visibleStaff = 1, visibleAdmin = 1 WHERE sectionKey = 'featured_machines'");
    }
}
