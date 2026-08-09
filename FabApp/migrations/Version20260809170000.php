<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/** S115: nullable workspace facets make this a safe expand migration. */
final class Version20260809170000 extends AbstractMigration
{
    public function isTransactional(): bool { return false; }
    public function getDescription(): string { return 'Add optional category, manager and department facets to spaces.'; }
    public function up(Schema $schema): void { $this->addSql('ALTER TABLE PLACE ADD category VARCHAR(120) DEFAULT NULL, ADD manager VARCHAR(150) DEFAULT NULL, ADD department VARCHAR(150) DEFAULT NULL'); }
    public function down(Schema $schema): void { $this->addSql('ALTER TABLE PLACE DROP category, DROP manager, DROP department'); }
}
