<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S134b: undo a duplicate. Grants converge on `USAGE_PACKAGE_GRANT`; package
 * assignment to a group converges on `USER_GROUP`.
 *
 * 🔴 **What went wrong.** S133b created `USAGE_GRANT` — packageId, featureKey,
 * section, action, venueId — without noticing that `Version20260809150000` (S111)
 * had created `USAGE_PACKAGE_GRANT` with the same five columns, a foreign key to
 * `VENUE`, three indexes and a working reader in `UsagePackageRepository`. The
 * comment on `ShadowUsageGrant` says "S111 persists it" and was read as a plan
 * rather than as a record. Two tables for one concept is the exact fault this
 * codebase keeps writing down about itself.
 *
 * ⚠️ **The order here is the whole safety story.** Grants v2 is live on all four
 * chokepoints and enforcement is ON, so the table the reader consults is deciding
 * real access right now. This migration only ever ADDS rows to the table that is
 * about to be read; the reader is switched in the same deploy, and **neither old
 * table is dropped**. A contract migration retires them once the converged model
 * has been watched. Dropping here would leave no way back that does not involve
 * restoring a backup.
 *
 * Two convergences:
 *
 *  1. **Grants.** Every `USAGE_GRANT` row is copied into `USAGE_PACKAGE_GRANT`
 *     unless an equivalent one is already there. ⚠️ The column is `section` in
 *     one and `sectionKey` in the other — the kind of difference that makes a
 *     copy silently drop a dimension, so it is named explicitly rather than
 *     matched by position.
 *
 *  2. **Groups.** S111 assigned packages to **roles**
 *     (`USAGE_PACKAGE_GROUP_ASSIGNMENT.roleId`). The vision document specifies
 *     seven protected audiences, two of which — Guest and User — cannot be roles
 *     at all because they are resolved rather than stored. Each role-based
 *     assignment becomes a `USAGE_RIGHT_ASSIGNMENT` carrying `groupId`, mapping
 *     the role onto its built-in group where one matches and creating a plain
 *     (non-builtin, deletable) group where none does.
 */
final class Version20260816140000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S134b: converge grants on USAGE_PACKAGE_GRANT and group assignment on USER_GROUP. Additive; drops nothing.';
    }

    public function up(Schema $schema): void
    {
        // ---- 1. Grants ----------------------------------------------------
        // ⚠️ `IS NOT DISTINCT FROM` does not exist in MariaDB; `<=>` is its
        // NULL-safe equality and is what makes "same grant" mean the same thing
        // for an unscoped grant (venueId NULL) as for a scoped one.
        $this->addSql(<<<'SQL'
            INSERT INTO USAGE_PACKAGE_GRANT (packageId, featureKey, sectionKey, action, venueId, createdAt)
            SELECT g.packageId, g.featureKey, g.section, g.action, g.venueId, NOW()
            FROM USAGE_GRANT g
            WHERE NOT EXISTS (
                SELECT 1 FROM USAGE_PACKAGE_GRANT t
                WHERE t.packageId = g.packageId
                  AND t.featureKey = g.featureKey
                  AND t.action = g.action
                  AND t.sectionKey <=> g.section
                  AND t.venueId <=> g.venueId
            )
        SQL);

        // A package that reached neither table keeps its v1 features as an
        // unscoped `use` grant — the same one-for-one meaning S133b's backfill
        // had, applied to whatever the first backfill did not reach.
        $this->addSql(<<<'SQL'
            INSERT INTO USAGE_PACKAGE_GRANT (packageId, featureKey, sectionKey, action, venueId, createdAt)
            SELECT f.packageId, f.featureKey, NULL, 'use', NULL, NOW()
            FROM USAGE_PACKAGE_FEATURE f
            WHERE NOT EXISTS (
                SELECT 1 FROM USAGE_PACKAGE_GRANT t
                WHERE t.packageId = f.packageId AND t.featureKey = f.featureKey
                  AND t.action = 'use' AND t.sectionKey IS NULL AND t.venueId IS NULL
            )
        SQL);

        // ---- 2. Groups ----------------------------------------------------
        // A group for every role that holds a package assignment and has no
        // built-in counterpart. `builtin = 0`, so these stay deletable — they are
        // the lab's own groups, not the seven protected ones.
        $this->addSql(<<<'SQL'
            INSERT IGNORE INTO USER_GROUP (groupKey, label, description, builtin, virtual, createdAt)
            SELECT DISTINCT CONCAT('role_', LOWER(r.nom)), r.nom,
                   'Repris d''une attribution par rôle (S134b).', 0, 0, NOW()
            FROM USAGE_PACKAGE_GROUP_ASSIGNMENT a
            INNER JOIN ROLE r ON r.id = a.roleId
            WHERE LOWER(r.nom) NOT IN ('admin', 'manager', 'staff', 'superuser', 'super_user', 'trainer', 'trainers', 'formateur', 'formateurs', 'user')
        SQL);

        // Each role-based assignment becomes a group assignment. ⚠️ `userId` is
        // NULL on these rows — that is what S133b widened the column for, and it
        // is why the pre-existing queries, which all `INNER JOIN UTILISATEUR`,
        // cannot see them and cannot be confused by them.
        $this->addSql(<<<'SQL'
            INSERT INTO USAGE_RIGHT_ASSIGNMENT (packageId, userId, groupId, validFrom, validUntil, issuedById, createdAt, revokedAt)
            SELECT a.packageId, NULL, g.id, a.validFrom, a.validUntil, a.issuedById, NOW(), a.revokedAt
            FROM USAGE_PACKAGE_GROUP_ASSIGNMENT a
            INNER JOIN ROLE r ON r.id = a.roleId
            INNER JOIN USER_GROUP g ON g.groupKey = CASE LOWER(r.nom)
                WHEN 'admin' THEN 'admin'
                WHEN 'manager' THEN 'manager'
                WHEN 'staff' THEN 'staff'
                WHEN 'superuser' THEN 'superuser'
                WHEN 'super_user' THEN 'superuser'
                WHEN 'trainer' THEN 'trainers'
                WHEN 'trainers' THEN 'trainers'
                WHEN 'formateur' THEN 'trainers'
                WHEN 'formateurs' THEN 'trainers'
                WHEN 'user' THEN 'user'
                ELSE CONCAT('role_', LOWER(r.nom))
            END
            WHERE NOT EXISTS (
                SELECT 1 FROM USAGE_RIGHT_ASSIGNMENT t
                WHERE t.packageId = a.packageId AND t.groupId = g.id
            )
        SQL);
    }

    public function down(Schema $schema): void
    {
        // Only the rows this migration created. ⚠️ The grant rows are deliberately
        // NOT removed: by the time anybody rolls back, the converged table is what
        // the reader has been using, and deleting from it would revoke live access.
        $this->addSql('DELETE FROM USAGE_RIGHT_ASSIGNMENT WHERE groupId IS NOT NULL AND userId IS NULL');
        $this->addSql("DELETE FROM USER_GROUP WHERE builtin = 0 AND groupKey LIKE 'role\\_%'");
    }
}
