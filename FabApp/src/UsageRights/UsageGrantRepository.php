<?php

declare(strict_types=1);

namespace App\UsageRights;

use App\Entity\Utilisateur;
use Doctrine\DBAL\Connection;

/**
 * Grants v2 — the reader, and since S134 an authoritative one.
 *
 * ⚠️ **No longer shadow-only.** `UsageRightsService::verdict()` consults this for
 * any capability an operator has moved with `isUsageRightsV2Active()`; everything
 * else still reads `USAGE_PACKAGE_FEATURE`. Treat every change here as a change
 * to live authorisation.
 *
 * 🔴 **The table is `USAGE_PACKAGE_GRANT`, and S133b got that wrong.** It created
 * a second table, `USAGE_GRANT`, with the same five columns — because the note on
 * `ShadowUsageGrant` saying "S111 persists it" was read as a plan rather than as
 * a record of `Version20260809150000`, which had already created this one with a
 * foreign key to `VENUE`, three indexes and a reader. S134b converged them.
 * ⚠️ The column is `sectionKey` here and was `section` there; a positional copy
 * would have dropped the dimension in silence.
 *
 * **What v2 adds over v1**, and why each of them makes the answer different:
 *
 *  - an **action**: v1 grants a feature, full stop, so there was no way to say
 *    "may see the reports but may not book". Manage never confers Use.
 *  - a **venue**: a grant can be limited to one location, which a multi-location
 *    installation cannot express at all today.
 *  - a **section**: a sub-domain of a feature (Maintenance under Équipement).
 *  - a **source**: a package can reach somebody through a personal assignment
 *    **or** through a group they are in. v1 only had the personal one.
 *
 * ⚠️ **Grants combine with OR, dimensions within one grant with AND.** Two
 * packages granting different venues grant both venues; one grant naming a venue
 * and a section requires both. A `NULL` venue or section means "unrestricted",
 * which is what the v1 backfill produces — so the shadow starts life agreeing
 * with the live model, exactly.
 *
 * ⚠️ **No grant ever removes a right.** Nothing here subtracts; the absence of a
 * covering grant is what a refusal is made of.
 */
final class UsageGrantRepository
{
    public function __construct(
        private readonly Connection $db,
        private readonly AudienceResolver $audiences,
    ) {
    }

    /**
     * Every path by which this person holds this capability at this instant.
     *
     * Returns the *paths*, not a boolean, because the roadmap requires the shadow
     * to be **explainable**: "denied" and "denied because the only package that
     * would have covered it is scoped to the other location" are different
     * answers, and only one of them tells an operator what to do.
     *
     * @return list<array{package: string, source: string, sourceLabel: string, action: string, section: ?string, venue: ?string}>
     */
    public function paths(
        ?Utilisateur $user,
        string $featureKey,
        UsageGrantAction $action,
        ?int $venueId = null,
        ?\DateTimeImmutable $at = null,
    ): array {
        $keys = $this->audiences->keysFor($user);
        $moment = ($at ?? new \DateTimeImmutable())->format('Y-m-d H:i:s');

        try {
            $rows = $this->db->fetchAllAssociative(
                <<<'SQL'
                SELECT p.name AS package,
                       g.action, g.sectionKey AS section,
                       v.name AS venue,
                       CASE WHEN a.userId IS NOT NULL THEN 'direct' ELSE 'group' END AS source,
                       COALESCE(ug.label, '') AS sourceLabel
                FROM USAGE_RIGHT_ASSIGNMENT a
                INNER JOIN USAGE_PACKAGE p ON p.id = a.packageId AND p.active = 1
                INNER JOIN USAGE_PACKAGE_GRANT g ON g.packageId = p.id
                LEFT JOIN USER_GROUP ug ON ug.id = a.groupId
                LEFT JOIN VENUE v ON v.id = g.venueId
                WHERE a.revokedAt IS NULL
                  AND (a.validFrom IS NULL OR a.validFrom <= :moment)
                  AND (a.validUntil IS NULL OR a.validUntil > :moment)
                  AND g.featureKey = :feature
                  AND g.action = :action
                  -- ⚠️ **Two different meanings of NULL, and they are not the same
                  -- one.** `g.venueId IS NULL` is a grant that is not restricted to
                  -- a location, so it covers every location. `:venue IS NULL` is a
                  -- CALLER that is not asking about a location — the shadow screen
                  -- asking "does this person hold this anywhere" — and it must match
                  -- scoped grants too, or that page would report a venue-scoped
                  -- member as having nothing. A caller that passes a venue gets the
                  -- restriction enforced; that is what `verdict()` does since S134b.
                  AND (g.venueId IS NULL OR :venue IS NULL OR g.venueId = :venue)
                  AND (
                        (:userId IS NOT NULL AND a.userId = :userId)
                     OR (a.groupId IS NOT NULL AND ug.groupKey IN (:keys))
                  )
                SQL,
                [
                    'moment' => $moment,
                    'feature' => $featureKey,
                    'action' => $action->value,
                    'venue' => $venueId,
                    'userId' => $user?->getId(),
                    'keys' => $keys === [] ? [''] : $keys,
                ],
                ['keys' => \Doctrine\DBAL\ArrayParameterType::STRING],
            );
        } catch (\Throwable) {
            // ⚠️ **This catch is no longer harmless, and that is worth saying
            // out loud.** While grants v2 was shadow-only an empty path list cost
            // a comparison; since S134 a capability can be live on this reader, so
            // swallowing a query error here would refuse everybody rather than
            // fail loudly. It is kept only for the window between deploying this
            // code and running its migration, and `tableExists()` is what the
            // screens use to say so in words instead of showing zeros.
            return [];
        }

        return array_map(static fn (array $row): array => [
            'package' => (string) $row['package'],
            'source' => (string) $row['source'],
            'sourceLabel' => (string) $row['sourceLabel'],
            'action' => (string) $row['action'],
            'section' => $row['section'] !== null ? (string) $row['section'] : null,
            'venue' => $row['venue'] !== null ? (string) $row['venue'] : null,
        ], $rows);
    }

    public function tableExists(): bool
    {
        try {
            $this->db->fetchOne('SELECT COUNT(*) FROM USAGE_PACKAGE_GRANT');

            return true;
        } catch (\Throwable) {
            return false;
        }
    }
}
