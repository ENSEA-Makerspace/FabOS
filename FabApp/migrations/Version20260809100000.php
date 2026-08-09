<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

final class Version20260809100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Make full-access usage packages durable as capabilities evolve.';
    }

    public function isTransactional(): bool
    {
        // MariaDB commits ALTER TABLE implicitly; declaring that explicitly
        // avoids Doctrine attempting to commit an already-closed transaction.
        return false;
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE USAGE_PACKAGE ADD fullAccess TINYINT(1) DEFAULT 0 NOT NULL AFTER active');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE USAGE_PACKAGE DROP fullAccess');
    }
}
