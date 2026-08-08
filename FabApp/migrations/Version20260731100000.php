<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Access passes: a staff-issued, expiring exemption from booking quotas.
 *
 * ⚠️ Read this before adding a column. A pass lifts **quotas** and nothing
 * else. There is deliberately no certification-bypass flag here, and one must
 * not be added to this table: "how much may you use it" and "are you trained to
 * use it safely" are different questions, and a pass is a convenience object
 * that gets handed around, extended and eventually made bearer-shared. Any
 * safety bypass needs its own explicitly-issued, supervision-scoped record.
 *
 * Equally absent is a fee-waiver flag, which the spec calls for: there is no
 * billing system yet to read it, and a stored grant that changes nothing is
 * worse than an absent one.
 *
 * Scope columns are nullable and null means "anything": a pass with both null
 * covers every resource, reservableType alone covers every machine, and both
 * set covers one machine. Validity is likewise all-optional — a pass with no
 * window and no cap is open-ended, which is a real thing staff want and should
 * have to choose deliberately rather than work around.
 */
final class Version20260731100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add ACCESS_PASS and ACCESS_PASS_USE: staff-issued, expiring quota exemptions.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE IF NOT EXISTS ACCESS_PASS (
            id INT AUTO_INCREMENT NOT NULL,
            userId INT NOT NULL,
            reservableType VARCHAR(20) DEFAULT NULL,
            reservableId INT DEFAULT NULL,
            validFrom DATETIME DEFAULT NULL,
            validUntil DATETIME DEFAULT NULL,
            maxUses INT DEFAULT NULL,
            usesCount INT DEFAULT 0 NOT NULL,
            reason VARCHAR(255) DEFAULT NULL,
            issuedById INT DEFAULT NULL,
            createdAt DATETIME NOT NULL,
            revokedAt DATETIME DEFAULT NULL,
            revokedById INT DEFAULT NULL,
            INDEX IDX_ACCESS_PASS_USER (userId),
            INDEX IDX_ACCESS_PASS_SCOPE (reservableType, reservableId),
            PRIMARY KEY(id)
        ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');

        // A revoked account should take its passes with it; the issuer is only a
        // reference, so losing that account must not delete the audit trail.
        $this->addSql('ALTER TABLE ACCESS_PASS ADD CONSTRAINT FK_ACCESS_PASS_USER FOREIGN KEY (userId) REFERENCES UTILISATEUR (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE ACCESS_PASS ADD CONSTRAINT FK_ACCESS_PASS_ISSUER FOREIGN KEY (issuedById) REFERENCES UTILISATEUR (id) ON DELETE SET NULL');

        // Separate from usesCount so "how many are left" and "what was it spent
        // on" are both answerable — a counter alone cannot be audited.
        $this->addSql('CREATE TABLE IF NOT EXISTS ACCESS_PASS_USE (
            id INT AUTO_INCREMENT NOT NULL,
            passId INT NOT NULL,
            reservationId INT DEFAULT NULL,
            userId INT DEFAULT NULL,
            usedAt DATETIME NOT NULL,
            INDEX IDX_ACCESS_PASS_USE_PASS (passId),
            PRIMARY KEY(id)
        ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');

        $this->addSql('ALTER TABLE ACCESS_PASS_USE ADD CONSTRAINT FK_ACCESS_PASS_USE_PASS FOREIGN KEY (passId) REFERENCES ACCESS_PASS (id) ON DELETE CASCADE');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE ACCESS_PASS_USE DROP FOREIGN KEY FK_ACCESS_PASS_USE_PASS');
        $this->addSql('DROP TABLE IF EXISTS ACCESS_PASS_USE');
        $this->addSql('ALTER TABLE ACCESS_PASS DROP FOREIGN KEY FK_ACCESS_PASS_USER');
        $this->addSql('ALTER TABLE ACCESS_PASS DROP FOREIGN KEY FK_ACCESS_PASS_ISSUER');
        $this->addSql('DROP TABLE IF EXISTS ACCESS_PASS');
    }
}
