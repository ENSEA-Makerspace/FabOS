<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

final class Version20260706180000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Crée la table des favoris machines par utilisateur.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE MACHINE_FAVORITE (id INT AUTO_INCREMENT NOT NULL, utilisateurId INT NOT NULL, machineId INT NOT NULL, createdAt DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP, INDEX IDX_MACHINE_FAVORITE_UTILISATEUR (utilisateurId), INDEX IDX_MACHINE_FAVORITE_MACHINE (machineId), UNIQUE INDEX unique_machine_favorite_user_machine (utilisateurId, machineId), PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('ALTER TABLE MACHINE_FAVORITE ADD CONSTRAINT FK_MACHINE_FAVORITE_UTILISATEUR FOREIGN KEY (utilisateurId) REFERENCES UTILISATEUR (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE MACHINE_FAVORITE ADD CONSTRAINT FK_MACHINE_FAVORITE_MACHINE FOREIGN KEY (machineId) REFERENCES MACHINE (id) ON DELETE CASCADE');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS MACHINE_FAVORITE');
    }
}
