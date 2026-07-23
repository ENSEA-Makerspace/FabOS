<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Portal foundation: the PORTAL table plus a portal scope on the two config
 * stores. Additive — existing SITE_SETTING / SITE_MODULE rows become portalId 0
 * ("global"), which is exactly what every lookup falls back to, and the seeded
 * default portal carries no hostname so nothing resolves to it. Behaviour is
 * unchanged until someone points a hostname at a second portal.
 *
 * Expand step: run this BEFORE deploying the code that reads portalId.
 */
final class Version20260726100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add PORTAL and scope SITE_SETTING / SITE_MODULE by portalId (0 = global).';
    }

    public function up(Schema $schema): void
    {
        // Written to be re-runnable: DDL isn't transactional, so a statement failing
        // half-way through must not leave the migration impossible to retry.
        $this->addSql('CREATE TABLE IF NOT EXISTS PORTAL (id INT AUTO_INCREMENT NOT NULL, slug VARCHAR(50) NOT NULL, name VARCHAR(100) NOT NULL, hostname VARCHAR(255) DEFAULT NULL, isDefault TINYINT(1) DEFAULT 0 NOT NULL, UNIQUE INDEX UNIQ_PORTAL_SLUG (slug), UNIQUE INDEX UNIQ_PORTAL_HOSTNAME (hostname), PRIMARY KEY(id)) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
        $this->addSql("INSERT IGNORE INTO PORTAL (slug, name, hostname, isDefault) VALUES ('default', 'FabOS', NULL, 1)");

        // portalId 0 is the global scope, not a PORTAL row — so it can sit in the
        // primary key (no nullable PK columns) and keep the existing upserts working.
        $this->addSql('ALTER TABLE SITE_SETTING ADD COLUMN IF NOT EXISTS portalId INT DEFAULT 0 NOT NULL');
        $this->addSql('ALTER TABLE SITE_SETTING DROP PRIMARY KEY, ADD PRIMARY KEY (settingKey, portalId)');
        $this->addSql('ALTER TABLE SITE_MODULE ADD COLUMN IF NOT EXISTS portalId INT DEFAULT 0 NOT NULL');
        $this->addSql('ALTER TABLE SITE_MODULE DROP PRIMARY KEY, ADD PRIMARY KEY (moduleKey, portalId)');
    }

    public function down(Schema $schema): void
    {
        // Per-portal overrides have no home once the scope is gone; drop them so the
        // narrowed primary keys can't collide on duplicate keys.
        $this->addSql('DELETE FROM SITE_SETTING WHERE portalId <> 0');
        $this->addSql('DELETE FROM SITE_MODULE WHERE portalId <> 0');
        $this->addSql('ALTER TABLE SITE_MODULE DROP PRIMARY KEY, ADD PRIMARY KEY (moduleKey)');
        $this->addSql('ALTER TABLE SITE_MODULE DROP COLUMN IF EXISTS portalId');
        $this->addSql('ALTER TABLE SITE_SETTING DROP PRIMARY KEY, ADD PRIMARY KEY (settingKey)');
        $this->addSql('ALTER TABLE SITE_SETTING DROP COLUMN IF EXISTS portalId');
        $this->addSql('DROP TABLE IF EXISTS PORTAL');
    }
}
