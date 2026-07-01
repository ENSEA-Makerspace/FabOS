<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

final class Version20260701120732 extends AbstractMigration
{
    public function getDescription(): string { return 'No-op: ancienne migration remplacée par Version20260701123000.'; }
    public function up(Schema $schema): void {}
    public function down(Schema $schema): void {}
}
