<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/** S111: persistent v2 package grants, deliberately not enforced in this phase. */
final class Version20260809150000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add global package grants and group assignments; copy legacy feature grants as use grants.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE USAGE_PACKAGE_GRANT (id INT AUTO_INCREMENT NOT NULL, packageId INT NOT NULL, featureKey VARCHAR(80) NOT NULL, sectionKey VARCHAR(80) DEFAULT NULL, action VARCHAR(10) NOT NULL, venueId INT DEFAULT NULL, createdAt DATETIME NOT NULL, INDEX IDX_USAGE_PACKAGE_GRANT_PACKAGE (packageId), INDEX IDX_USAGE_PACKAGE_GRANT_VENUE (venueId), INDEX IDX_USAGE_PACKAGE_GRANT_LOOKUP (featureKey, sectionKey, action), PRIMARY KEY(id), CONSTRAINT FK_USAGE_PACKAGE_GRANT_PACKAGE FOREIGN KEY (packageId) REFERENCES USAGE_PACKAGE (id) ON DELETE CASCADE, CONSTRAINT FK_USAGE_PACKAGE_GRANT_VENUE FOREIGN KEY (venueId) REFERENCES VENUE (id) ON DELETE RESTRICT) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql('CREATE TABLE USAGE_PACKAGE_GROUP_ASSIGNMENT (id INT AUTO_INCREMENT NOT NULL, packageId INT NOT NULL, roleId INT NOT NULL, validFrom DATETIME DEFAULT NULL, validUntil DATETIME DEFAULT NULL, issuedById INT DEFAULT NULL, createdAt DATETIME NOT NULL, revokedAt DATETIME DEFAULT NULL, revokedById INT DEFAULT NULL, INDEX IDX_USAGE_PACKAGE_GROUP_ASSIGNMENT_PACKAGE (packageId), INDEX IDX_USAGE_PACKAGE_GROUP_ASSIGNMENT_ROLE_ACTIVE (roleId, revokedAt, validFrom, validUntil), PRIMARY KEY(id), CONSTRAINT FK_USAGE_PACKAGE_GROUP_ASSIGNMENT_PACKAGE FOREIGN KEY (packageId) REFERENCES USAGE_PACKAGE (id) ON DELETE CASCADE, CONSTRAINT FK_USAGE_PACKAGE_GROUP_ASSIGNMENT_ROLE FOREIGN KEY (roleId) REFERENCES ROLE (id) ON DELETE CASCADE, CONSTRAINT FK_USAGE_PACKAGE_GROUP_ASSIGNMENT_ISSUER FOREIGN KEY (issuedById) REFERENCES UTILISATEUR (id) ON DELETE SET NULL, CONSTRAINT FK_USAGE_PACKAGE_GROUP_ASSIGNMENT_REVOKER FOREIGN KEY (revokedById) REFERENCES UTILISATEUR (id) ON DELETE SET NULL) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql("INSERT INTO USAGE_PACKAGE_GRANT (packageId, featureKey, sectionKey, action, venueId, createdAt) SELECT packageId, featureKey, NULL, 'use', NULL, NOW() FROM USAGE_PACKAGE_FEATURE");
        $this->addSql('UPDATE USAGE_PACKAGE SET portalId = 0');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE USAGE_PACKAGE_GROUP_ASSIGNMENT');
        $this->addSql('DROP TABLE USAGE_PACKAGE_GRANT');
        $this->addSql('UPDATE USAGE_PACKAGE SET portalId = 0');
    }
}
