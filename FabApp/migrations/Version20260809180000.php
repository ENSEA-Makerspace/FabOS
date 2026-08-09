<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

final class Version20260809180000 extends AbstractMigration
{
    public function isTransactional(): bool { return false; }
    public function getDescription(): string { return 'Add optional manufacturer and model metadata to equipment.'; }
    public function up(Schema $schema): void { $this->addSql('ALTER TABLE MACHINE ADD manufacturer VARCHAR(150) DEFAULT NULL, ADD model VARCHAR(150) DEFAULT NULL'); }
    public function down(Schema $schema): void { $this->addSql('ALTER TABLE MACHINE DROP manufacturer, DROP model'); }
}
