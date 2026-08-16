<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S133b: groups, audiences and grants v2 — the SHADOW half.
 *
 * ⚠️ **Nothing here changes a single live authorisation.** Every table is new or
 * a nullable column, the live path (`USAGE_PACKAGE_FEATURE` →
 * `UsagePackageRepository::grantingPackages()`) is untouched and still the only
 * thing `UsageRightsService::verdict()` reads. Grants v2 is computed beside it
 * and shown; S134 is where it starts deciding, one audited chokepoint at a time.
 * A migration that silently began enforcing a new model is the single worst
 * outcome available here, so the enforcement switch is deliberately not in it.
 *
 * Three additions:
 *
 *  1. **`USER_GROUP` / `USER_GROUP_MEMBER`.** The seven built-in entries are
 *     seeded with `builtin = 1` and stable keys; labels are editable, the rows
 *     are not deletable. ⚠️ `user` and `guest` are seeded too but are **system
 *     audiences**: they never get a `USER_GROUP_MEMBER` row, because "every
 *     active account" and "nobody signed in" are resolved, not stored — a
 *     membership row for them is one more thing provisioning can forget.
 *
 *  2. **`USAGE_GRANT`.** What a package actually grants: feature, optional
 *     section, `use` or `manage`, and an optional venue. Backfilled one-to-one
 *     from `USAGE_PACKAGE_FEATURE` as `use` with no section and no venue, which
 *     is exactly what v1 means — so the shadow starts life agreeing with the live
 *     model, and every later disagreement is a deliberate edit rather than a
 *     migration artefact.
 *
 *  3. **`USAGE_RIGHT_ASSIGNMENT.groupId`.** A package given to a group instead of
 *     a person. Nullable, so every existing row stays a personal assignment.
 */
final class Version20260816130000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S133b: USER_GROUP, USER_GROUP_MEMBER, USAGE_GRANT, assignment.groupId. Additive, shadow only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS USER_GROUP (
                id INT AUTO_INCREMENT NOT NULL,
                groupKey VARCHAR(60) NOT NULL,
                label VARCHAR(120) NOT NULL,
                description VARCHAR(255) DEFAULT NULL,
                builtin TINYINT(1) NOT NULL DEFAULT 0,
                virtual TINYINT(1) NOT NULL DEFAULT 0,
                createdAt DATETIME NOT NULL COMMENT '(DC2Type:datetime_immutable)',
                UNIQUE INDEX UNIQ_USER_GROUP_KEY (groupKey),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);

        // ⚠️ `virtual = 1` on user and guest: resolved from the account, never
        // stored as membership. Admin/Manager/Staff/Super user/Formateurs have
        // real members and can receive packages.
        foreach ([
            ['admin', 'Administrateur global', 'Recovery système. Protégé : le dernier administrateur ne peut pas être retiré.', 0],
            ['manager', 'Manager', 'Encadrement d’un ou plusieurs workspaces.', 0],
            ['staff', 'Staff', 'Équipe du lieu : comptoir, accueil, passages.', 0],
            ['superuser', 'Super user', 'Membre expérimenté, autonome sur davantage d’équipement.', 0],
            ['trainers', 'Formateurs', 'Rattaché au workspace Formations.', 0],
            ['user', 'Utilisateur authentifié', 'Audience système : tout compte local actif. Sans ligne d’appartenance.', 1],
            ['guest', 'Invité anonyme', 'Audience système virtuelle, pour les actions réellement publiques.', 1],
        ] as [$key, $label, $description, $virtual]) {
            $this->addSql(
                'INSERT IGNORE INTO USER_GROUP (groupKey, label, description, builtin, virtual, createdAt) VALUES (:k, :l, :d, 1, :v, NOW())',
                ['k' => $key, 'l' => $label, 'd' => $description, 'v' => $virtual],
            );
        }

        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS USER_GROUP_MEMBER (
                groupId INT NOT NULL,
                userId INT NOT NULL,
                addedAt DATETIME NOT NULL COMMENT '(DC2Type:datetime_immutable)',
                INDEX IDX_USER_GROUP_MEMBER_USER (userId),
                PRIMARY KEY(groupId, userId),
                CONSTRAINT FK_USER_GROUP_MEMBER_GROUP FOREIGN KEY (groupId) REFERENCES USER_GROUP (id) ON DELETE CASCADE,
                CONSTRAINT FK_USER_GROUP_MEMBER_USER FOREIGN KEY (userId) REFERENCES UTILISATEUR (id) ON DELETE CASCADE
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);

        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS USAGE_GRANT (
                id INT AUTO_INCREMENT NOT NULL,
                packageId INT NOT NULL,
                featureKey VARCHAR(60) NOT NULL,
                section VARCHAR(60) DEFAULT NULL,
                action VARCHAR(10) NOT NULL,
                venueId INT DEFAULT NULL,
                createdAt DATETIME NOT NULL COMMENT '(DC2Type:datetime_immutable)',
                INDEX IDX_USAGE_GRANT_PACKAGE (packageId),
                INDEX IDX_USAGE_GRANT_FEATURE (featureKey),
                PRIMARY KEY(id),
                CONSTRAINT FK_USAGE_GRANT_PACKAGE FOREIGN KEY (packageId) REFERENCES USAGE_PACKAGE (id) ON DELETE CASCADE
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);

        // The shadow starts life AGREEING with the live model. Anything that
        // disagrees later is an edit somebody made, not something a migration did.
        $this->addSql(<<<'SQL'
            INSERT INTO USAGE_GRANT (packageId, featureKey, section, action, venueId, createdAt)
            SELECT f.packageId, f.featureKey, NULL, 'use', NULL, NOW()
            FROM USAGE_PACKAGE_FEATURE f
            WHERE NOT EXISTS (
                SELECT 1 FROM USAGE_GRANT g
                WHERE g.packageId = f.packageId AND g.featureKey = f.featureKey
                  AND g.action = 'use' AND g.section IS NULL AND g.venueId IS NULL
            )
        SQL);

        $this->addSql('ALTER TABLE USAGE_RIGHT_ASSIGNMENT ADD groupId INT DEFAULT NULL');
        $this->addSql('ALTER TABLE USAGE_RIGHT_ASSIGNMENT ADD CONSTRAINT FK_USAGE_ASSIGNMENT_GROUP FOREIGN KEY (groupId) REFERENCES USER_GROUP (id) ON DELETE CASCADE');
        // ⚠️ **Widening, and it is done NOW on purpose.** A group assignment has no
        // `userId`, so the column has to accept NULL before one can exist. Doing it
        // here rather than during S134's activation keeps the schema change and the
        // enforcement change in separate deploys — which is the whole reason
        // expand/contract exists. Widening NOT NULL to NULL loses nothing and old
        // code, which always writes a userId, is unaffected; the existing queries
        // `INNER JOIN UTILISATEUR`, so a group row is simply invisible to them.
        $this->addSql('ALTER TABLE USAGE_RIGHT_ASSIGNMENT MODIFY userId INT DEFAULT NULL');
    }

    public function down(Schema $schema): void
    {
        // ⚠️ Narrowing back would fail on any group assignment. Deliberately not
        // attempted: `down()` removes what this migration added and leaves the
        // column nullable, which the pre-migration code tolerates.
        $this->addSql('ALTER TABLE USAGE_RIGHT_ASSIGNMENT DROP FOREIGN KEY FK_USAGE_ASSIGNMENT_GROUP');
        $this->addSql('ALTER TABLE USAGE_RIGHT_ASSIGNMENT DROP COLUMN groupId');
        $this->addSql('DROP TABLE IF EXISTS USAGE_GRANT');
        $this->addSql('DROP TABLE IF EXISTS USER_GROUP_MEMBER');
        $this->addSql('DROP TABLE IF EXISTS USER_GROUP');
    }
}
