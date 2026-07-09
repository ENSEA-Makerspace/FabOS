<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Auto-generated Migration: Please modify to your needs!
 */
final class Version20260709142042 extends AbstractMigration
{
    public function getDescription(): string
    {
        return '';
    }

    public function up(Schema $schema): void
    {
        // this up() migration is auto-generated, please modify it to your needs
        $this->addSql('CREATE TABLE CREATION (id INT AUTO_INCREMENT NOT NULL, title VARCHAR(150) NOT NULL, description LONGTEXT DEFAULT NULL, imageFilename VARCHAR(255) DEFAULT NULL, fileFilename VARCHAR(255) DEFAULT NULL, externalUrl VARCHAR(500) DEFAULT NULL, printDurationMinutes INT DEFAULT NULL, authorName VARCHAR(150) DEFAULT NULL, isPublished TINYINT DEFAULT 1 NOT NULL, createdAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL, updatedAt DATETIME DEFAULT NULL, authorId INT DEFAULT NULL, INDEX IDX_91DD1938A196F9FD (authorId), PRIMARY KEY (id)) DEFAULT CHARACTER SET utf8mb4');
        $this->addSql('CREATE TABLE CREATION_VOTE (id INT AUTO_INCREMENT NOT NULL, createdAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL, creationId INT NOT NULL, userId INT NOT NULL, INDEX IDX_60EBEEA1FC3CDF41 (creationId), INDEX IDX_60EBEEA164B64DCC (userId), UNIQUE INDEX uniq_creation_vote_user (creationId, userId), PRIMARY KEY (id)) DEFAULT CHARACTER SET utf8mb4');
        $this->addSql('ALTER TABLE CREATION ADD CONSTRAINT FK_91DD1938A196F9FD FOREIGN KEY (authorId) REFERENCES UTILISATEUR (id) ON DELETE SET NULL');
        $this->addSql('ALTER TABLE CREATION_VOTE ADD CONSTRAINT FK_60EBEEA1FC3CDF41 FOREIGN KEY (creationId) REFERENCES CREATION (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE CREATION_VOTE ADD CONSTRAINT FK_60EBEEA164B64DCC FOREIGN KEY (userId) REFERENCES UTILISATEUR (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE utilisateur ADD avatarFilename VARCHAR(255) DEFAULT NULL');
    }

    public function down(Schema $schema): void
    {
        // this down() migration is auto-generated, please modify it to your needs
        $this->addSql('ALTER TABLE CREATION DROP FOREIGN KEY FK_91DD1938A196F9FD');
        $this->addSql('ALTER TABLE CREATION_VOTE DROP FOREIGN KEY FK_60EBEEA1FC3CDF41');
        $this->addSql('ALTER TABLE CREATION_VOTE DROP FOREIGN KEY FK_60EBEEA164B64DCC');
        $this->addSql('DROP TABLE CREATION');
        $this->addSql('DROP TABLE CREATION_VOTE');
        $this->addSql('ALTER TABLE UTILISATEUR DROP avatarFilename');
    }
}
