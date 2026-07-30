<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * A record of the pages people ask for and do not get.
 *
 * Two shape decisions carry the whole design:
 *
 * **One row per distinct path, with a counter** — not one row per hit. The write
 * happens on the 404 path, where a bot sweep or a hotlinked dead image can
 * arrive thousands of times, and a row-per-hit table would turn a 404 storm into
 * a database storm and then grow without bound. The unique key on
 * `(portalId, path)` is what makes the `ON DUPLICATE KEY UPDATE` upsert work, and
 * it is also why the table stays small enough to need no scheduled prune: its
 * size is bounded by the number of *distinct* wrong URLs, not by traffic.
 *
 * **`reason` is stored, not derived** — `feature` (someone reached for a feature
 * that is switched off), `internal` (a broken link in the site's own content),
 * `external` (a wrong URL out in the world). The first is not a bug at all; it is
 * demand, and it is answered on the feature screen. Telling those apart is the
 * entire point of the table, and it cannot be reconstructed after the fact.
 *
 * `portalId INT NOT NULL DEFAULT 0` with 0 = global, matching SITE_SETTING and
 * SITE_MODULE — a nullable column could not sit in the unique key.
 *
 * The reader is raw DBAL and fail-safe, so the code may ship before this runs:
 * with no table, nothing is recorded and the screen shows an empty list.
 */
final class Version20260804100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add MISSING_PAGE: aggregated log of requested URLs that 404.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE IF NOT EXISTS MISSING_PAGE (
            id INT AUTO_INCREMENT NOT NULL,
            portalId INT DEFAULT 0 NOT NULL,
            path VARCHAR(190) NOT NULL,
            reason VARCHAR(20) NOT NULL,
            hits INT DEFAULT 1 NOT NULL,
            firstSeen DATETIME NOT NULL,
            lastSeen DATETIME NOT NULL,
            lastReferrer VARCHAR(255) DEFAULT NULL,
            UNIQUE INDEX UNIQ_MISSING_PAGE_PATH (portalId, path),
            INDEX IDX_MISSING_PAGE_HITS (hits),
            PRIMARY KEY(id)
        ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS MISSING_PAGE');
    }
}
