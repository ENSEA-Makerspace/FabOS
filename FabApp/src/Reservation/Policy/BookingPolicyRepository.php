<?php

namespace App\Reservation\Policy;

use App\Reservation\ReservableType;
use Doctrine\DBAL\Connection;

/**
 * Storage for the quota table. Raw DBAL and fail-safe, like the other
 * config-adjacent stores.
 *
 * The failure direction here is the important one: **an unreadable table
 * resolves to "no limit", not "no booking"**. A quota engine that starts
 * refusing every reservation because its own config table is missing would take
 * the lab offline over a setting nobody had even filled in — the same reasoning
 * that makes an empty table a valid configuration.
 */
final class BookingPolicyRepository
{
    public function __construct(private readonly Connection $db)
    {
    }

    public function find(ReservableType $type, BookingTier $tier): ?BookingPolicy
    {
        try {
            $row = $this->db->fetchAssociative(
                'SELECT * FROM BOOKING_POLICY WHERE reservableType = :type AND tier = :tier',
                ['type' => $type->value, 'tier' => $tier->value],
            );
        } catch (\Throwable) {
            return null;
        }

        return is_array($row) ? BookingPolicy::fromRow($row) : null;
    }

    /**
     * Every configured policy, keyed "type:tier" — one query for the admin grid,
     * which would otherwise do a lookup per cell.
     *
     * @return array<string, BookingPolicy>
     */
    public function allByScope(): array
    {
        try {
            $rows = $this->db->fetchAllAssociative('SELECT * FROM BOOKING_POLICY');
        } catch (\Throwable) {
            return [];
        }

        $policies = [];
        foreach ($rows as $row) {
            $policy = BookingPolicy::fromRow($row);
            if ($policy !== null) {
                $policies[$policy->type->value . ':' . $policy->tier->value] = $policy;
            }
        }

        return $policies;
    }

    /**
     * Writes one scope's limits.
     *
     * A policy with nothing set is deleted rather than stored as a row of nulls,
     * so "configured" and "constrains something" stay the same thing and the
     * admin grid can show empty cells honestly.
     */
    public function save(BookingPolicy $policy): void
    {
        if ($policy->isUnrestricted()) {
            $this->delete($policy->type, $policy->tier);

            return;
        }

        $values = $policy->toFormValues();
        $columns = implode(', ', BookingPolicy::FIELDS);
        $placeholders = implode(', ', array_map(static fn (string $f): string => ':' . $f, BookingPolicy::FIELDS));
        $updates = implode(', ', array_map(static fn (string $f): string => $f . ' = :' . $f, BookingPolicy::FIELDS));

        $this->db->executeStatement(
            "INSERT INTO BOOKING_POLICY (reservableType, tier, {$columns}, updatedAt)
             VALUES (:type, :tier, {$placeholders}, NOW())
             ON DUPLICATE KEY UPDATE {$updates}, updatedAt = NOW()",
            ['type' => $policy->type->value, 'tier' => $policy->tier->value] + $values,
        );
    }

    public function delete(ReservableType $type, BookingTier $tier): void
    {
        try {
            $this->db->executeStatement(
                'DELETE FROM BOOKING_POLICY WHERE reservableType = :type AND tier = :tier',
                ['type' => $type->value, 'tier' => $tier->value],
            );
        } catch (\Throwable) {
        }
    }
}
