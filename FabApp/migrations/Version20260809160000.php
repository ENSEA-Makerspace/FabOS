<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/** S116: badge definitions are archived and their local award journal is never cascaded away. */
final class Version20260809160000 extends AbstractMigration
{
    public function isTransactional(): bool { return false; }
    public function getDescription(): string
    {
        return 'Archive badge definitions and make badge-award/equipment foreign keys non-destructive.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE BADGE ADD archivedAt DATETIME DEFAULT NULL');
        $this->addSql('ALTER TABLE UTILISATEUR_BADGE DROP FOREIGN KEY fk_utilisateur_badge_badge');
        $this->addSql('ALTER TABLE UTILISATEUR_BADGE ADD CONSTRAINT fk_utilisateur_badge_badge FOREIGN KEY (badgeId) REFERENCES BADGE (id) ON DELETE RESTRICT');
        $this->addSql('ALTER TABLE MACHINE_BADGE DROP FOREIGN KEY fk_machine_badge_badge');
        $this->addSql('ALTER TABLE MACHINE_BADGE ADD CONSTRAINT fk_machine_badge_badge FOREIGN KEY (badgeId) REFERENCES BADGE (id) ON DELETE RESTRICT');
        $this->addSql('ALTER TABLE BADGE_INSTITUTION DROP FOREIGN KEY FK_BADGE_INSTITUTION_BADGE');
        $this->addSql('ALTER TABLE BADGE_INSTITUTION ADD CONSTRAINT FK_BADGE_INSTITUTION_BADGE FOREIGN KEY (badge_id) REFERENCES BADGE (id) ON DELETE RESTRICT');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE UTILISATEUR_BADGE DROP FOREIGN KEY fk_utilisateur_badge_badge');
        $this->addSql('ALTER TABLE UTILISATEUR_BADGE ADD CONSTRAINT fk_utilisateur_badge_badge FOREIGN KEY (badgeId) REFERENCES BADGE (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE MACHINE_BADGE DROP FOREIGN KEY fk_machine_badge_badge');
        $this->addSql('ALTER TABLE MACHINE_BADGE ADD CONSTRAINT fk_machine_badge_badge FOREIGN KEY (badgeId) REFERENCES BADGE (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE BADGE_INSTITUTION DROP FOREIGN KEY FK_BADGE_INSTITUTION_BADGE');
        $this->addSql('ALTER TABLE BADGE_INSTITUTION ADD CONSTRAINT FK_BADGE_INSTITUTION_BADGE FOREIGN KEY (badge_id) REFERENCES BADGE (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE BADGE DROP archivedAt');
    }
}
