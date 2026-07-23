<?php

namespace App\Reservation\Policy;

use App\Entity\Utilisateur;
use App\Reservation\ReservableType;
use Doctrine\DBAL\Connection;

/**
 * Storage for access passes.
 *
 * **Reads fail CLOSED here — the opposite of BookingPolicyRepository, and on
 * purpose.** A policy table that cannot be read resolves to "no limit", because
 * refusing every booking over a config problem would take the lab offline. A
 * pass table that cannot be read resolves to "no pass", because the failure
 * would otherwise *grant* an exemption nobody issued. In both cases the
 * unreadable state falls back to the ordinary rules; it is only that "ordinary"
 * sits on opposite sides of the two tables.
 */
final class AccessPassRepository
{
    public function __construct(private readonly Connection $db)
    {
    }

    /**
     * The pass this person should spend on this booking, if any.
     *
     * Picks the most constrained applicable pass — soonest to expire, then
     * fewest uses left — so a narrow "this machine, this week" pass is consumed
     * before an open-ended one, and the general pass survives for later.
     */
    public function findApplicable(Utilisateur $user, ReservableType $type, int $id, \DateTimeImmutable $now): ?AccessPass
    {
        $userId = $user->getId();
        if ($userId === null) {
            return null;
        }

        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT * FROM ACCESS_PASS
                 WHERE userId = :user AND revokedAt IS NULL
                 ORDER BY validUntil IS NULL, validUntil ASC, id ASC',
                ['user' => $userId],
            );
        } catch (\Throwable) {
            return null;
        }

        $best = null;
        foreach ($rows as $row) {
            $pass = AccessPass::fromRow($row);
            if ($pass === null || !$pass->appliesTo($type, $id, $now)) {
                continue;
            }

            if ($best === null || $this->isNarrower($pass, $best)) {
                $best = $pass;
            }
        }

        return $best;
    }

    /**
     * Spends one use, and records what it was spent on.
     *
     * The counter is bumped with a guarded UPDATE rather than a read-then-write,
     * so two bookings racing for the last use of a pass cannot both win.
     *
     * @return bool whether this call got the use
     */
    public function consume(AccessPass $pass, ?int $reservationId, ?int $userId): bool
    {
        try {
            $affected = $this->db->executeStatement(
                'UPDATE ACCESS_PASS
                 SET usesCount = usesCount + 1
                 WHERE id = :id AND revokedAt IS NULL AND (maxUses IS NULL OR usesCount < maxUses)',
                ['id' => $pass->id],
            );

            if ($affected === 0) {
                return false;
            }

            $this->db->executeStatement(
                'INSERT INTO ACCESS_PASS_USE (passId, reservationId, userId, usedAt) VALUES (:pass, :reservation, :user, NOW())',
                ['pass' => $pass->id, 'reservation' => $reservationId, 'user' => $userId],
            );
        } catch (\Throwable) {
            return false;
        }

        return true;
    }

    /** @return AccessPass[] */
    public function findAll(): array
    {
        try {
            $rows = $this->db->fetchAllAssociative('SELECT * FROM ACCESS_PASS ORDER BY id DESC LIMIT 200');
        } catch (\Throwable) {
            return [];
        }

        $passes = [];
        foreach ($rows as $row) {
            $pass = AccessPass::fromRow($row);
            if ($pass !== null) {
                $passes[] = $pass;
            }
        }

        return $passes;
    }

    public function issue(
        int $userId,
        ?ReservableType $type,
        ?int $reservableId,
        ?\DateTimeImmutable $validFrom,
        ?\DateTimeImmutable $validUntil,
        ?int $maxUses,
        ?string $reason,
        ?int $issuedById,
    ): void {
        $this->db->executeStatement(
            'INSERT INTO ACCESS_PASS (userId, reservableType, reservableId, validFrom, validUntil, maxUses, usesCount, reason, issuedById, createdAt)
             VALUES (:user, :type, :resource, :from, :until, :maxUses, 0, :reason, :issuer, NOW())',
            [
                'user' => $userId,
                'type' => $type?->value,
                'resource' => $reservableId,
                'from' => $validFrom?->format('Y-m-d H:i:s'),
                'until' => $validUntil?->format('Y-m-d H:i:s'),
                'maxUses' => $maxUses,
                'reason' => $reason !== null && $reason !== '' ? mb_substr($reason, 0, 255) : null,
                'issuer' => $issuedById,
            ],
        );
    }

    /** Revocation is a tombstone, never a delete: the audit trail has to survive. */
    public function revoke(int $passId, ?int $revokedById): void
    {
        try {
            $this->db->executeStatement(
                'UPDATE ACCESS_PASS SET revokedAt = NOW(), revokedById = :by WHERE id = :id AND revokedAt IS NULL',
                ['id' => $passId, 'by' => $revokedById],
            );
        } catch (\Throwable) {
        }
    }

    /** @return array<int, int> passId => recorded uses, for the admin list. */
    public function useCounts(): array
    {
        try {
            $rows = $this->db->fetchAllAssociative('SELECT passId, COUNT(*) AS total FROM ACCESS_PASS_USE GROUP BY passId');
        } catch (\Throwable) {
            return [];
        }

        $counts = [];
        foreach ($rows as $row) {
            $counts[(int) $row['passId']] = (int) $row['total'];
        }

        return $counts;
    }

    /** Narrower = expires sooner, or (all else equal) has fewer uses left. */
    private function isNarrower(AccessPass $candidate, AccessPass $current): bool
    {
        if ($candidate->validUntil !== $current->validUntil) {
            if ($candidate->validUntil === null) {
                return false;
            }

            if ($current->validUntil === null) {
                return true;
            }

            return $candidate->validUntil < $current->validUntil;
        }

        $candidateLeft = $candidate->remainingUses();
        $currentLeft = $current->remainingUses();

        if ($candidateLeft === null) {
            return false;
        }

        return $currentLeft === null || $candidateLeft < $currentLeft;
    }
}
