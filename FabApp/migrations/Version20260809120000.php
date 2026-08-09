<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

final class Version20260809120000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Scope physical resources to the default venue; leave off-site events unscoped.';
    }

    public function up(Schema $schema): void
    {
        foreach (['MACHINE', 'PLACE', 'LOANABLE_ITEM'] as $table) {
            $this->addSql(sprintf('ALTER TABLE %s ADD venueId INT DEFAULT NULL', $table));
            $this->addSql(sprintf("UPDATE %s SET venueId = (SELECT id FROM VENUE WHERE slug = 'default') WHERE venueId IS NULL", $table));
            $this->addSql(sprintf('ALTER TABLE %s MODIFY venueId INT NOT NULL', $table));
            $this->addSql(sprintf('ALTER TABLE %s ADD INDEX IDX_%s_VENUE (venueId), ADD CONSTRAINT FK_%s_VENUE FOREIGN KEY (venueId) REFERENCES VENUE (id)', $table, $table, $table));
        }

        $this->addSql('ALTER TABLE EVENEMENT ADD venueId INT DEFAULT NULL');
        $this->addSql("UPDATE EVENEMENT SET venueId = (SELECT id FROM VENUE WHERE slug = 'default') WHERE locationMode = 'onsite'");
        $this->addSql('ALTER TABLE EVENEMENT ADD INDEX IDX_EVENEMENT_VENUE (venueId), ADD CONSTRAINT FK_EVENEMENT_VENUE FOREIGN KEY (venueId) REFERENCES VENUE (id) ON DELETE SET NULL');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE EVENEMENT DROP FOREIGN KEY FK_EVENEMENT_VENUE');
        $this->addSql('ALTER TABLE EVENEMENT DROP INDEX IDX_EVENEMENT_VENUE, DROP COLUMN venueId');

        foreach (['MACHINE', 'PLACE', 'LOANABLE_ITEM'] as $table) {
            $this->addSql(sprintf('ALTER TABLE %s DROP FOREIGN KEY FK_%s_VENUE', $table, $table));
            $this->addSql(sprintf('ALTER TABLE %s DROP INDEX IDX_%s_VENUE, DROP COLUMN venueId', $table, $table));
        }
    }
}
