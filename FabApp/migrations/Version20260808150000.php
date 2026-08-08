<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

final class Version20260808150000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add reusable usage packages, their feature grants and member assignments.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE USAGE_PACKAGE (
            id INT AUTO_INCREMENT NOT NULL,
            portalId INT NOT NULL DEFAULT 0,
            name VARCHAR(120) NOT NULL,
            description TEXT DEFAULT NULL,
            active TINYINT(1) NOT NULL DEFAULT 1,
            createdAt DATETIME NOT NULL,
            updatedAt DATETIME NOT NULL,
            INDEX IDX_USAGE_PACKAGE_PORTAL (portalId),
            PRIMARY KEY(id)
        ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('CREATE TABLE USAGE_PACKAGE_FEATURE (
            packageId INT NOT NULL,
            featureKey VARCHAR(80) NOT NULL,
            INDEX IDX_USAGE_PACKAGE_FEATURE_PACKAGE (packageId),
            PRIMARY KEY(packageId, featureKey),
            CONSTRAINT FK_USAGE_PACKAGE_FEATURE_PACKAGE FOREIGN KEY (packageId) REFERENCES USAGE_PACKAGE (id) ON DELETE CASCADE
        ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('CREATE TABLE USAGE_RIGHT_ASSIGNMENT (
            id INT AUTO_INCREMENT NOT NULL,
            packageId INT NOT NULL,
            userId INT NOT NULL,
            validFrom DATETIME DEFAULT NULL,
            validUntil DATETIME DEFAULT NULL,
            issuedById INT DEFAULT NULL,
            createdAt DATETIME NOT NULL,
            revokedAt DATETIME DEFAULT NULL,
            revokedById INT DEFAULT NULL,
            INDEX IDX_USAGE_RIGHT_ASSIGNMENT_PACKAGE (packageId),
            INDEX IDX_USAGE_RIGHT_ASSIGNMENT_USER (userId),
            INDEX IDX_USAGE_RIGHT_ASSIGNMENT_ACTIVE (userId, revokedAt, validFrom, validUntil),
            PRIMARY KEY(id),
            CONSTRAINT FK_USAGE_RIGHT_ASSIGNMENT_PACKAGE FOREIGN KEY (packageId) REFERENCES USAGE_PACKAGE (id) ON DELETE CASCADE,
            CONSTRAINT FK_USAGE_RIGHT_ASSIGNMENT_USER FOREIGN KEY (userId) REFERENCES UTILISATEUR (id) ON DELETE CASCADE,
            CONSTRAINT FK_USAGE_RIGHT_ASSIGNMENT_ISSUER FOREIGN KEY (issuedById) REFERENCES UTILISATEUR (id) ON DELETE SET NULL,
            CONSTRAINT FK_USAGE_RIGHT_ASSIGNMENT_REVOKER FOREIGN KEY (revokedById) REFERENCES UTILISATEUR (id) ON DELETE SET NULL
        ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE USAGE_RIGHT_ASSIGNMENT');
        $this->addSql('DROP TABLE USAGE_PACKAGE_FEATURE');
        $this->addSql('DROP TABLE USAGE_PACKAGE');
    }
}
