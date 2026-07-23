<?php

namespace App\Reservation\Policy;

use App\Reservation\ReservableType;

/**
 * One staff-issued exemption from the booking quotas.
 *
 * What it grants is fixed and singular — quota lift — so there is no "grants"
 * field to interpret. That is a deliberate limit, not an oversight: see the
 * migration for why certification bypass must never become a column here.
 *
 * Scope and validity are both "null means unbounded", which makes the common
 * cases short to express: a pass for one machine this week, or a blanket pass
 * for a new member with no expiry, are the same object with different nulls.
 */
final readonly class AccessPass
{
    public function __construct(
        public int $id,
        public int $userId,
        public ?ReservableType $reservableType,
        public ?int $reservableId,
        public ?\DateTimeImmutable $validFrom,
        public ?\DateTimeImmutable $validUntil,
        public ?int $maxUses,
        public int $usesCount,
        public ?string $reason,
        public ?int $issuedById,
        public ?\DateTimeImmutable $revokedAt,
    ) {
    }

    /** @param array<string, mixed> $row */
    public static function fromRow(array $row): ?self
    {
        if (!isset($row['id'], $row['userId'])) {
            return null;
        }

        $date = static function (string $key) use ($row): ?\DateTimeImmutable {
            $raw = $row[$key] ?? null;

            if ($raw === null || $raw === '') {
                return null;
            }

            try {
                return new \DateTimeImmutable((string) $raw);
            } catch (\Throwable) {
                return null;
            }
        };

        return new self(
            (int) $row['id'],
            (int) $row['userId'],
            ReservableType::tryParse(isset($row['reservableType']) ? (string) $row['reservableType'] : null),
            isset($row['reservableId']) && $row['reservableId'] !== null ? (int) $row['reservableId'] : null,
            $date('validFrom'),
            $date('validUntil'),
            isset($row['maxUses']) && $row['maxUses'] !== null ? (int) $row['maxUses'] : null,
            (int) ($row['usesCount'] ?? 0),
            isset($row['reason']) && $row['reason'] !== null ? (string) $row['reason'] : null,
            isset($row['issuedById']) && $row['issuedById'] !== null ? (int) $row['issuedById'] : null,
            $date('revokedAt'),
        );
    }

    /** Whether this pass may be spent right now on this particular resource. */
    public function appliesTo(ReservableType $type, int $id, \DateTimeImmutable $now): bool
    {
        if ($this->revokedAt !== null) {
            return false;
        }

        // Null scope means "anything", so only a set scope can disqualify.
        if ($this->reservableType !== null && $this->reservableType !== $type) {
            return false;
        }

        if ($this->reservableId !== null && $this->reservableId !== $id) {
            return false;
        }

        if ($this->validFrom !== null && $now < $this->validFrom) {
            return false;
        }

        if ($this->validUntil !== null && $now > $this->validUntil) {
            return false;
        }

        return !$this->isExhausted();
    }

    public function isExhausted(): bool
    {
        return $this->maxUses !== null && $this->usesCount >= $this->maxUses;
    }

    /** Live, as opposed to revoked, expired or spent — for the admin list. */
    public function isActive(\DateTimeImmutable $now): bool
    {
        return $this->revokedAt === null
            && !$this->isExhausted()
            && ($this->validUntil === null || $now <= $this->validUntil);
    }

    public function remainingUses(): ?int
    {
        return $this->maxUses === null ? null : max(0, $this->maxUses - $this->usesCount);
    }

    /** Short human description of what this pass covers, for the admin list. */
    public function scopeLabel(): string
    {
        if ($this->reservableType === null) {
            return 'Toutes les ressources';
        }

        $kind = match ($this->reservableType) {
            ReservableType::Machine => 'Machines',
            ReservableType::Place => 'Espaces',
            ReservableType::User => 'Personnes',
        };

        return $this->reservableId === null ? $kind : $kind . ' #' . $this->reservableId;
    }
}
