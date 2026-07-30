<?php

namespace App\Http;

use App\Portal\PortalContext;
use Doctrine\DBAL\Connection;

/**
 * A record of the URLs people ask for and do not get.
 *
 * The point is not the 404s themselves — it is that two very different things
 * look identical in a web server log, and the operator needs them apart:
 *
 *  - a **broken link** in their own content, which is a mistake to fix;
 *  - **somebody reaching for a feature that is switched off**, which is not a
 *    bug at all — it is the gating model working — but it *is* demand, and the
 *    answer to it lives on the feature screen, not in the templates.
 *
 * Design constraints, all of them consequences of sitting on the 404 path:
 *
 *  - **Aggregated by path, never one row per hit.** A bot sweep or a hotlinked
 *    dead image would otherwise turn a 404 storm into a database storm, and the
 *    table would grow without bound. One `INSERT … ON DUPLICATE KEY UPDATE` per
 *    miss, and the counter does the rest. That is also why this needs no
 *    scheduled prune to stay small: the row count is bounded by the number of
 *    *distinct* wrong URLs, not by traffic.
 *  - **Raw DBAL and fail-safe, like the other config-adjacent stores.** Every
 *    method swallows its errors. A missing table must not turn a 404 into a 500,
 *    which would be a strictly worse page than the one we were about to render.
 *  - **Last observation wins** for the reason and the referrer. A path that used
 *    to be a disabled feature and is now simply wrong should read as the latter.
 */
final class MissingPageLog
{
    /** Somebody asked for a page belonging to a feature that is switched off. */
    public const REASON_FEATURE = 'feature';
    /** Reached from a link on this site — almost always the operator's own content. */
    public const REASON_INTERNAL = 'internal';
    /** Reached from a link somewhere else, so the wrong URL is out in the world. */
    public const REASON_EXTERNAL = 'external';

    public function __construct(
        private readonly Connection $db,
        private readonly PortalContext $portals,
    ) {
    }

    public function record(string $path, string $reason, ?string $referrer = null): void
    {
        try {
            $this->db->executeStatement(
                'INSERT INTO MISSING_PAGE (portalId, path, reason, hits, firstSeen, lastSeen, lastReferrer)
                 VALUES (:portal, :path, :reason, 1, NOW(), NOW(), :referrer)
                 ON DUPLICATE KEY UPDATE hits = hits + 1, lastSeen = NOW(), reason = :reason, lastReferrer = :referrer',
                [
                    'portal' => $this->portals->scopeId(),
                    // 190 keeps the (portalId, path) unique key inside InnoDB's
                    // index limit under utf8mb4. Anything longer is truncated
                    // rather than dropped: a 300-character probe still counts.
                    'path' => mb_substr($path, 0, 190),
                    'reason' => mb_substr($reason, 0, 20),
                    'referrer' => $referrer === null ? null : mb_substr($referrer, 0, 255),
                ],
            );
        } catch (\Throwable) {
            // Never let bookkeeping break the page it is describing.
        }
    }

    /**
     * Most-asked-for first, which is the order that matters: one URL asked for
     * two hundred times is a link the operator can find and fix.
     *
     * @return list<array<string, mixed>>
     */
    public function top(int $limit = 200): array
    {
        try {
            return $this->db->fetchAllAssociative(
                'SELECT path, reason, hits, firstSeen, lastSeen, lastReferrer
                 FROM MISSING_PAGE
                 WHERE portalId IN (:g, :p)
                 ORDER BY hits DESC, lastSeen DESC
                 LIMIT ' . max(1, min(1000, $limit)),
                ['g' => PortalContext::GLOBAL_SCOPE, 'p' => $this->portals->scopeId()],
            );
        } catch (\Throwable) {
            return [];
        }
    }

    /** @return array{paths: int, hits: int, feature: int, internal: int, external: int} */
    public function summary(): array
    {
        $summary = ['paths' => 0, 'hits' => 0, 'feature' => 0, 'internal' => 0, 'external' => 0];

        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT reason, COUNT(*) AS paths, SUM(hits) AS hits
                 FROM MISSING_PAGE WHERE portalId IN (:g, :p) GROUP BY reason',
                ['g' => PortalContext::GLOBAL_SCOPE, 'p' => $this->portals->scopeId()],
            );
        } catch (\Throwable) {
            return $summary;
        }

        foreach ($rows as $row) {
            $reason = (string) $row['reason'];
            $summary['paths'] += (int) $row['paths'];
            $summary['hits'] += (int) $row['hits'];
            if (array_key_exists($reason, $summary)) {
                $summary[$reason] = (int) $row['paths'];
            }
        }

        return $summary;
    }

    /**
     * Forgets paths nobody has asked for in a while. Aggregation already keeps
     * the table small, so this is about relevance rather than size: a URL that
     * stopped being requested six months ago is not a lead any more.
     */
    public function prune(int $keepDays = 90): int
    {
        try {
            return $this->db->executeStatement(
                'DELETE FROM MISSING_PAGE WHERE lastSeen < DATE_SUB(NOW(), INTERVAL :days DAY)',
                ['days' => max(1, $keepDays)],
            );
        } catch (\Throwable) {
            return 0;
        }
    }

    /** Starts the record over — after fixing the links, so the next miss stands out. */
    public function clear(): int
    {
        try {
            return $this->db->executeStatement(
                'DELETE FROM MISSING_PAGE WHERE portalId IN (:g, :p)',
                ['g' => PortalContext::GLOBAL_SCOPE, 'p' => $this->portals->scopeId()],
            );
        } catch (\Throwable) {
            return 0;
        }
    }
}
