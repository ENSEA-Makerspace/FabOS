<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Add the LOANABLE_ITEM and LOAN tables backing the Loans module (lending
 * catalogue + checkout records). Additive only.
 */
final class Version20260723160000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add LOANABLE_ITEM and LOAN tables for the Loans module.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE LOANABLE_ITEM (id INT AUTO_INCREMENT NOT NULL, name VARCHAR(150) NOT NULL, category VARCHAR(80) DEFAULT NULL, description LONGTEXT DEFAULT NULL, imageUrl VARCHAR(500) DEFAULT NULL, icon VARCHAR(16) DEFAULT NULL, quantity INT DEFAULT 1 NOT NULL, storageLocation VARCHAR(180) DEFAULT NULL, createdAt DATETIME NOT NULL, PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('CREATE INDEX IDX_LOANABLE_ITEM_CATEGORY ON LOANABLE_ITEM (category)');

        $this->addSql('CREATE TABLE LOAN (id INT AUTO_INCREMENT NOT NULL, itemId INT NOT NULL, borrowerId INT DEFAULT NULL, borrowerName VARCHAR(180) DEFAULT NULL, borrowerEmail VARCHAR(180) DEFAULT NULL, borrowerPhone VARCHAR(60) DEFAULT NULL, dateTaken DATETIME NOT NULL, expectedReturnDate DATE DEFAULT NULL, actualReturnDate DATE DEFAULT NULL, status VARCHAR(20) DEFAULT \'out\' NOT NULL, conditionOut LONGTEXT DEFAULT NULL, conditionReturn LONGTEXT DEFAULT NULL, notes LONGTEXT DEFAULT NULL, lentById INT DEFAULT NULL, createdAt DATETIME NOT NULL, INDEX IDX_LOAN_ITEM (itemId), INDEX IDX_LOAN_BORROWER (borrowerId), INDEX IDX_LOAN_STATUS (status), PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('ALTER TABLE LOAN ADD CONSTRAINT FK_LOAN_ITEM FOREIGN KEY (itemId) REFERENCES LOANABLE_ITEM (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE LOAN ADD CONSTRAINT FK_LOAN_BORROWER FOREIGN KEY (borrowerId) REFERENCES UTILISATEUR (id) ON DELETE SET NULL');
        $this->addSql('ALTER TABLE LOAN ADD CONSTRAINT FK_LOAN_LENTBY FOREIGN KEY (lentById) REFERENCES UTILISATEUR (id) ON DELETE SET NULL');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE LOAN');
        $this->addSql('DROP TABLE LOANABLE_ITEM');
    }
}
