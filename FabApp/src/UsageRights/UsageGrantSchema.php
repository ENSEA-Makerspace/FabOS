<?php

declare(strict_types=1);

namespace App\UsageRights;

use Doctrine\DBAL\Connection;

/**
 * Which parts of the grant schema this database actually has (S144b).
 *
 * 🔴 **Why this exists at all.** Grants v2 decides real access on four live
 * chokepoints, and `UsageGrantRepository::paths()` answers an empty path list —
 * a refusal for everybody — on any query error. Naming a column the operator has
 * not migrated yet would therefore take booking away from the whole lab between
 * a deploy and a migration, silently and site-wide. S133 already shipped the
 * cheaper version of that mistake: `LOANABLE_ITEM.archivedAt` went out before its
 * migration and two pages 500'd until it was run.
 *
 * ⚠️ **Probes fail towards the OLD behaviour**, never towards the new one. An
 * install with the code and not the migration keeps exactly the rights it had:
 * no resource scope, no windows, every grant unrestricted on the new axes. That
 * direction is the whole safety argument for deploying code first.
 *
 * ⚠️ Memoised per request, not cached across them: a probe that survived a
 * migration would keep answering "no" for as long as the process lived, which on
 * a long-running worker is until somebody restarts it.
 */
final class UsageGrantSchema
{
    private ?bool $scopeColumns = null;
    private ?bool $windowTable = null;

    public function __construct(private readonly Connection $db)
    {
    }

    public function hasScopeColumns(): bool
    {
        return $this->scopeColumns ??= $this->answers(
            'SELECT reservableType, reservableId, categoryLabel FROM USAGE_PACKAGE_GRANT LIMIT 1',
        );
    }

    public function hasWindowTable(): bool
    {
        return $this->windowTable ??= $this->answers('SELECT COUNT(*) FROM USAGE_GRANT_WINDOW');
    }

    /** ⚠️ An empty table is a yes: the question is whether the schema answers, not whether it has rows. */
    private function answers(string $sql): bool
    {
        try {
            $this->db->fetchOne($sql);

            return true;
        } catch (\Throwable) {
            return false;
        }
    }
}
