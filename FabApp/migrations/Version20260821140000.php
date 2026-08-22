<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * The contract step: two duplicate tables go (S134b).
 *
 * 🔴 **`USAGE_GRANT` should never have existed.** S111 shipped `USAGE_PACKAGE_GRANT`
 * as THE grant table; S133b created a second one with the same five columns because
 * a comment on `ShadowUsageGrant` ("S111 persists it") was read as a plan rather than
 * a record. S134b converged the two; this removes the loser.
 * 🔴 **`USAGE_PACKAGE_GROUP_ASSIGNMENT` lost its last reader in S144a**, when
 * `v2ShadowVerdict()` was deleted — a second, unexercised implementation of "does
 * this person hold this grant" that still joined `UTILISATEUR_ROLE` and would have
 * disagreed with `paths()` about every group created since S133b.
 *
 * ✅ **Checked before dropping, on the live database, rather than trusted:**
 *   - no PHP reads either name — the only hits in `src/` are the two comments that
 *     tell this story;
 *   - `USAGE_PACKAGE_GROUP_ASSIGNMENT` holds **0 rows**;
 *   - all **21** rows of `USAGE_GRANT` have an exact twin in `USAGE_PACKAGE_GRANT`
 *     on (packageId, featureKey, section↔sectionKey, action, venueId) — a LEFT JOIN
 *     for the missing ones returned zero. ⚠️ A matching row COUNT would not have
 *     proved this; the join is what proves it.
 *
 * ⚠️ **Contract, so it is the step that cannot be taken back by re-running.** `down()`
 * rebuilds the shapes but not the contents — which is survivable precisely because
 * the contents were a duplicate: `USAGE_GRANT` can be re-derived from
 * `USAGE_PACKAGE_GRANT`, and the group table was empty.
 *
 * ⚠️ **Nothing reads these, so the code needs no deploy before or after.** This is the
 * one migration in the sequence with no ordering hazard attached.
 */
final class Version20260821140000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S134b (contract): drop USAGE_GRANT and USAGE_PACKAGE_GROUP_ASSIGNMENT — converged in S134b, unread since.';
    }

    public function up(Schema $schema): void
    {
        // ⚠️ A last guard in SQL, not only in the docblock above: if anything has
        // written to the group table since this was written, stop rather than drop.
        $stranded = (int) $this->connection->fetchOne('SELECT COUNT(*) FROM USAGE_PACKAGE_GROUP_ASSIGNMENT');
        $this->abortIf(
            $stranded > 0,
            sprintf('USAGE_PACKAGE_GROUP_ASSIGNMENT holds %d row(s): something started using it. Investigate before dropping.', $stranded),
        );

        $orphans = (int) $this->connection->fetchOne(
            'SELECT COUNT(*) FROM USAGE_GRANT g
             LEFT JOIN USAGE_PACKAGE_GRANT p
               ON p.packageId = g.packageId AND p.featureKey = g.featureKey
              AND p.sectionKey <=> g.section AND p.action = g.action AND p.venueId <=> g.venueId
             WHERE p.id IS NULL',
        );
        $this->abortIf(
            $orphans > 0,
            sprintf('%d USAGE_GRANT row(s) have no twin in USAGE_PACKAGE_GRANT: converge them before dropping.', $orphans),
        );

        $this->addSql('DROP TABLE IF EXISTS USAGE_PACKAGE_GROUP_ASSIGNMENT');
        $this->addSql('DROP TABLE IF EXISTS USAGE_GRANT');
    }

    public function down(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS USAGE_GRANT (
                id INT AUTO_INCREMENT NOT NULL,
                packageId INT NOT NULL,
                featureKey VARCHAR(64) NOT NULL,
                section VARCHAR(64) DEFAULT NULL,
                action VARCHAR(32) NOT NULL,
                venueId INT DEFAULT NULL,
                createdAt DATETIME NOT NULL COMMENT '(DC2Type:datetime_immutable)',
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);

        // ⚠️ Re-derived, not restored: this table was always a copy of the live one.
        $this->addSql(
            'INSERT INTO USAGE_GRANT (packageId, featureKey, section, action, venueId, createdAt)
             SELECT packageId, featureKey, sectionKey, action, venueId, createdAt FROM USAGE_PACKAGE_GRANT',
        );

        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS USAGE_PACKAGE_GROUP_ASSIGNMENT (
                id INT AUTO_INCREMENT NOT NULL,
                packageId INT NOT NULL,
                groupId INT NOT NULL,
                createdAt DATETIME NOT NULL COMMENT '(DC2Type:datetime_immutable)',
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);
    }
}
