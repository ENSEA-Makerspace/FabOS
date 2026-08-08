<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Booking quotas: how much of a resource one tier of person may hold.
 *
 * Every limit column is nullable and null means "no limit", so the table
 * arriving empty is a complete, valid configuration — booking behaves exactly
 * as it did before this migration until an admin fills something in. That is
 * why **no rows are seeded**: a lab upgrading into this version must not
 * discover it by having its members' bookings start getting refused.
 *
 * Keyed by (reservableType, tier) rather than per machine or per space: the
 * unit people reason about is "how much may a member book a machine", and a
 * per-resource override can be layered on later without moving this table.
 */
final class Version20260730100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add BOOKING_POLICY: per resource-kind x tier booking limits, all optional.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('CREATE TABLE IF NOT EXISTS BOOKING_POLICY (
            id INT AUTO_INCREMENT NOT NULL,
            reservableType VARCHAR(20) NOT NULL,
            tier VARCHAR(20) NOT NULL,
            minNoticeMinutes INT DEFAULT NULL,
            maxHorizonDays INT DEFAULT NULL,
            slotIncrementMinutes INT DEFAULT NULL,
            minDurationMinutes INT DEFAULT NULL,
            maxDurationMinutes INT DEFAULT NULL,
            maxActiveReservations INT DEFAULT NULL,
            maxPerDay INT DEFAULT NULL,
            maxPerWeek INT DEFAULT NULL,
            bufferMinutes INT DEFAULT NULL,
            updatedAt DATETIME NOT NULL,
            UNIQUE INDEX UNIQ_BOOKING_POLICY_SCOPE (reservableType, tier),
            PRIMARY KEY(id)
        ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS BOOKING_POLICY');
    }
}
