<?php

declare(strict_types=1);

namespace App\Venue;

use App\Entity\Venue;
use App\Repository\VenueRepository;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;

/**
 * The rules that decide whether a sub-venue may be archived, and what it costs.
 *
 * **Why a service and not a controller check (S129).** Three different screens
 * need the same verdict — the list (to disable the button), the confirmation (to
 * state the impact) and the POST (to actually refuse). A rule that only lives in
 * the template is an affordance, not an invariant, and the roadmap's Sol criterion
 * for this session is explicitly "aucun lockout".
 *
 * ⚠️ **Archive is not delete, and nothing here deletes.** `Machine.venueId` and
 * `LoanableItem.venueId` are `NOT NULL … ON DELETE RESTRICT`, so a venue holding
 * either cannot be removed by the database in any case. Archiving sets
 * `active = false`, which removes the venue from `VenueContext`'s pickers and from
 * the aggregate view — the rows keep pointing at it and come back on restore.
 */
final class VenueGuard
{
    /** Attachment counts, in the order the confirmation screen lists them. */
    private const ATTACHMENTS = [
        'machines' => ['MACHINE', 'venueId'],
        'places' => ['PLACE', 'venueId'],
        'events' => ['EVENEMENT', 'venueId'],
        'loanables' => ['LOANABLE_ITEM', 'venueId'],
        'hours' => ['OPENING_HOUR', 'venueId'],
        'members' => ['UTILISATEUR', 'preferredVenueId'],
    ];

    public function __construct(
        private readonly Connection $db,
        private readonly EntityManagerInterface $em,
        private readonly VenueRepository $venues,
    ) {
    }

    /**
     * How many rows point at this venue, per kind. Zero-valued kinds are kept so
     * the caller can render a complete, stable list rather than a shifting one.
     *
     * @return array<string, int>
     */
    public function attachments(Venue $venue): array
    {
        $id = $venue->getId();
        if ($id === null) {
            return array_fill_keys(array_keys(self::ATTACHMENTS), 0);
        }

        $counts = [];
        foreach (self::ATTACHMENTS as $kind => [$table, $column]) {
            $counts[$kind] = (int) $this->db->fetchOne(
                sprintf('SELECT COUNT(*) FROM %s WHERE %s = :venue', $table, $column),
                ['venue' => $id],
            );
        }

        return $counts;
    }

    /** Total rows that would follow the venue into the archive. */
    public function attachmentTotal(Venue $venue): int
    {
        return array_sum($this->attachments($venue));
    }

    /**
     * Why this venue cannot be archived, or null when it can.
     *
     * Returns a translation key rather than a sentence: the same verdict is shown
     * as a disabled button's tooltip, as a refusal flash and in five languages.
     */
    public function archiveRefusal(Venue $venue): ?string
    {
        if (!$venue->isActive()) {
            return 'venues.refusal.already_archived';
        }

        // The default venue is the fallback every un-scoped row was backfilled to
        // in S106/S107. Archiving it would empty the aggregate view for objects
        // that have never been given anything else.
        if ($venue->isDefault()) {
            return 'venues.refusal.default_protected';
        }

        // ⚠️ The real lockout. `VenueContext` resolves "all" to the *active* venues;
        // with none active, every venue-scoped list renders empty and the picker
        // offers nothing to switch back to — including this screen.
        if ($this->countActive() <= 1) {
            return 'venues.refusal.last_active';
        }

        return null;
    }

    public function canArchive(Venue $venue): bool
    {
        return $this->archiveRefusal($venue) === null;
    }

    /**
     * Archive after re-checking the verdict.
     *
     * ⚠️ The re-check is not redundant with the controller's: between rendering the
     * confirmation and posting it, another admin may have archived the other venue,
     * making this one the last active. The guard is the chokepoint, so it decides.
     *
     * @throws \DomainException carrying the refusal key
     */
    public function archive(Venue $venue): void
    {
        $refusal = $this->archiveRefusal($venue);
        if ($refusal !== null) {
            throw new \DomainException($refusal);
        }

        $venue->setActive(false);
        $this->em->flush();
    }

    /** Restoring is always safe: it only ever adds a venue back to the pickers. */
    public function restore(Venue $venue): void
    {
        $venue->setActive(true);
        $this->em->flush();
    }

    private function countActive(): int
    {
        return $this->venues->count(['active' => true]);
    }
}
