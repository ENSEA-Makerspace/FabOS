<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S144c: a package can carry a budget — "10 hours of machine time a week".
 *
 * The second half of the operator's sentence. S144b taught a grant *what* and
 * *when*; this is *how much*. Between them a fablab can express the shapes it
 * actually sells: unlimited access to one machine, off-peak access to a
 * category, a prepaid block of hours, a session allowance that resets on Monday.
 *
 * ⚠️ **No allowance row means no ceiling.** Every package that exists today
 * carries none, so this migration changes nothing until an operator adds one —
 * which is what makes it safe to run on a live lab in the middle of a week.
 *
 * ⚠️ **`period = 'total'` is the absence of a period**, not a long one: the
 * allowance is spent once and never resets. A prepaid block of hours cannot be
 * expressed as "per year" without lying every 1 January.
 *
 * ⚠️ Deliberately **no `categoryLabel`** here, although the grant table has one.
 * Nothing would enforce it yet, and a column that only a form writes is exactly
 * the fault S144a spent a session repairing on `USAGE_RIGHT_ASSIGNMENT.groupId`.
 * It goes in when the counting query can honour it.
 */
final class Version20260817110000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S144c: USAGE_PACKAGE_ALLOWANCE — how much a package allows, per day/week/month/total. Additive.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS USAGE_PACKAGE_ALLOWANCE (
                id INT AUTO_INCREMENT NOT NULL,
                packageId INT NOT NULL,
                featureKey VARCHAR(64) DEFAULT NULL,
                reservableType VARCHAR(20) DEFAULT NULL,
                unit VARCHAR(16) NOT NULL,
                amount INT NOT NULL,
                period VARCHAR(16) NOT NULL,
                createdAt DATETIME NOT NULL,
                INDEX IDX_USAGE_ALLOWANCE_PACKAGE (packageId),
                PRIMARY KEY(id),
                CONSTRAINT FK_USAGE_ALLOWANCE_PACKAGE FOREIGN KEY (packageId)
                    REFERENCES USAGE_PACKAGE (id) ON DELETE CASCADE
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);
    }

    /** Drops the budgets an operator wrote; there is nowhere else they are kept. */
    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS USAGE_PACKAGE_ALLOWANCE');
    }
}
