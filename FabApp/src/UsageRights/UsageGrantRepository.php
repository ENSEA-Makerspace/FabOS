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
 *  - a **resource** (S144b): a kind, one machine, or a category. "Lets you 3D
 *    print" is a sentence about a family of machines, not about a feature.
 *  - a **time of week** (S144b): `USAGE_GRANT_WINDOW`, enforced by coverage of
 *    the whole booking rather than by overlap — see `GrantWindowSet`.
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
        private readonly UsageGrantSchema $schema,
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
        ?UsageScope $scope = null,
    ): array {
        $scope ??= UsageScope::any();
        $rows = $this->grantRows($user, $featureKey, $action, $scope);

        // ⚠️ **The window filter is in PHP, and on purpose.** "Every minute of
        // this booking falls inside the union of that grant's windows" is not a
        // predicate a row-at-a-time WHERE clause can express — the union is across
        // rows, and a booking can span days. Expressed in SQL it would have been
        // an overlap test, which is the exact mistake `GrantWindowSet` exists to
        // refuse: a Monday-afternoon package must not open a Monday evening.
        if ($scope->isTimed()) {
            $windows = $this->windowsForGrants(array_map(static fn (array $row): int => (int) $row['grantId'], $rows));
            if ($windows !== []) {
                $rows = array_values(array_filter($rows, static fn (array $row): bool => GrantWindowSet::covers(
                    $windows[(int) $row['grantId']] ?? [],
                    $scope->from,
                    $scope->until,
                )));
            }
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

    /**
     * The covering grant rows, before any window filtering.
     *
     * ⚠️ **One query, two readers.** `paths()` needs these rows to answer about an
     * instant and `windowsFor()` needs them to answer about a week. Written twice
     * they would drift, and this is the query that decides who may book: the venue
     * NULL-versus-NULL subtlety below took two sessions to get right the first
     * time and must not be re-derived in a second copy.
     *
     * @return list<array<string, mixed>>
     */
    private function grantRows(
        ?Utilisateur $user,
        string $featureKey,
        UsageGrantAction $action,
        UsageScope $scope,
    ): array {
        $keys = $this->audiences->keysFor($user);
        $moment = $scope->at()->format('Y-m-d H:i:s');

        // ⚠️ **Only asked of a schema that has them** (S144b). This reader is
        // authoritative on four live chokepoints, and its catch-all returns an
        // empty path list — which is a refusal. Naming a column that the operator
        // has not migrated yet would therefore take booking away from every
        // non-admin between the deploy and the migration, silently. The scope
        // clause is compiled in only once the columns answer.
        $scoped = $this->schema->hasScopeColumns();
        $scopeClause = $scoped
            ? <<<'SQL'
                  AND (g.reservableType IS NULL OR :reservableType IS NULL OR g.reservableType = :reservableType)
                  AND (g.reservableId IS NULL OR :reservableId IS NULL OR g.reservableId = :reservableId)
            SQL
            : '';

        // ⚠️ **S147, J-21 — quand un grant porte l'identifiant, l'IDENTIFIANT DÉCIDE
        // et le libellé se tait.**
        // 🔴 Première version fausse, et le témoin négatif l'a attrapée : les deux
        // clauses étaient des ET, si bien qu'un grant portant les deux exigeait les
        // deux — et après un renommage son libellé stocké est périmé par
        // construction. Le grant « avec identifiant » se décrochait donc exactement
        // comme celui d'avant. Le libellé n'est pas une seconde restriction : c'est
        // un cache de l'identifiant, et un cache périmé ne doit rien interdire.
        // ⚠️ Un grant SANS identifiant garde le comportement d'avant, au libellé —
        // c'est ce qui laisse les grants anciens fonctionner jusqu'au backfill.
        $categoryScoped = $scoped && $this->schema->hasCategoryIdColumn();
        $categoryClause = $scoped
            ? ($categoryScoped
                ? <<<'SQL'
                      AND (
                            (g.categoryId IS NOT NULL AND (:categoryId IS NULL OR g.categoryId = :categoryId))
                         OR (g.categoryId IS NULL AND (g.categoryLabel IS NULL OR :categoryLabel IS NULL OR g.categoryLabel = :categoryLabel))
                      )
                SQL
                : '  AND (g.categoryLabel IS NULL OR :categoryLabel IS NULL OR g.categoryLabel = :categoryLabel)')
            : '';

        try {
            $rows = $this->db->fetchAllAssociative(
                <<<SQL
                SELECT p.name AS package, g.id AS grantId,
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
                {$scopeClause}
                {$categoryClause}
                  AND (
                        (:userId IS NOT NULL AND a.userId = :userId)
                     OR (a.groupId IS NOT NULL AND ug.groupKey IN (:keys))
                  )
                SQL,
                array_merge([
                    'moment' => $moment,
                    'feature' => $featureKey,
                    'action' => $action->value,
                    'venue' => $scope->venueId,
                    'userId' => $user?->getId(),
                    'keys' => $keys === [] ? [''] : $keys,
                ], $scoped ? [
                    'reservableType' => $scope->reservableType,
                    'reservableId' => $scope->reservableId,
                    'categoryLabel' => $scope->categoryLabel,
                ] : [], $categoryScoped ? ['categoryId' => $scope->categoryId] : []),
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
        return $rows;
    }

    /**
     * The weekly opening windows a person's grants impose on this capability.
     *
     * 🔴 **Why this is not just `paths()` again.** `paths()` answers about ONE
     * instant: it takes the interval a booking already has and filters the grants
     * that cover it. A calendar has no single instant — it draws a whole week and
     * has to know, slot by slot, which of them a package would accept. So this
     * returns the windows themselves and lets the surface do the drawing.
     *
     * ⚠️ **An empty list means NO time restriction, never "nothing allowed".**
     * Grants combine with OR, so a single covering grant that carries no window
     * opens the whole week and the answer is `[]` — the same value as "this person
     * has no grants at all", which is safe because the caller has already asked
     * `verdict()` whether they may book here at all. Getting this backwards would
     * grey out every slot for everybody.
     *
     * @return list<GrantWindow> the union across every covering grant
     */
    public function windowsFor(
        ?Utilisateur $user,
        string $featureKey,
        UsageGrantAction $action,
        ?UsageScope $scope = null,
    ): array {
        $rows = $this->grantRows($user, $featureKey, $action, $scope ?? UsageScope::any());
        if ($rows === []) {
            return [];
        }

        $byGrant = $this->windowsForGrants(array_map(static fn (array $row): int => (int) $row['grantId'], $rows));

        $union = [];
        foreach ($rows as $row) {
            $windows = $byGrant[(int) $row['grantId']] ?? [];
            if ($windows === []) {
                // One unrestricted grant is enough: nothing below it can narrow.
                return [];
            }
            foreach ($windows as $window) {
                $union[] = $window;
            }
        }

        return $union;
    }

    /**
     * @param list<int> $grantIds
     * @return array<int, list<GrantWindow>> grant id => its windows, absent when it has none
     */
    public function windowsForGrants(array $grantIds): array
    {
        $grantIds = array_values(array_unique(array_filter($grantIds)));
        if ($grantIds === [] || !$this->schema->hasWindowTable()) {
            return [];
        }

        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT grantId, dayOfWeek, startMinute, endMinute FROM USAGE_GRANT_WINDOW
                 WHERE grantId IN (:ids) ORDER BY dayOfWeek, startMinute',
                ['ids' => $grantIds],
                ['ids' => \Doctrine\DBAL\ArrayParameterType::INTEGER],
            );
        } catch (\Throwable) {
            return [];
        }

        $windows = [];
        foreach ($rows as $row) {
            $windows[(int) $row['grantId']][] = GrantWindow::fromRow($row);
        }

        return $windows;
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
