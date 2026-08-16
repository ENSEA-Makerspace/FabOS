<?php

declare(strict_types=1);

namespace App\UsageRights;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Service\SiteSettingService;

/**
 * The live verdict, the grants-v2 verdict, and **why they differ** (S133b).
 *
 * ⚠️ **This decides nothing.** It calls `UsageRightsService::verdict()` for the
 * answer that is actually in force and `UsageGrantRepository::paths()` for the
 * answer the new model would give, and reports both. No route, voter or template
 * may branch on the shadow half — the roadmap's phrase is "shadow visible et
 * explicable", and the visible part is the whole deliverable until S134.
 *
 * ⚠️ **A shadow that only says "differs" is not explainable.** The reason an
 * operator cannot act on a bare mismatch is that there are four quite different
 * causes, and they need four different responses:
 *
 *   `agree`             nothing to do.
 *   `enforcement_off`   the live answer is "yes" because enforcement is off, not
 *                       because anything granted it. This is the state the whole
 *                       installation is in today, so it is the *expected* row and
 *                       must not be shown as a fault.
 *   `admin_bypass`      admins are allowed by the live model regardless. Grants
 *                       v2 keeps recovery, so this is expected too.
 *   `shadow_would_deny` the one that matters: with enforcement on, this person
 *                       would lose access they have today. Every one of these is
 *                       a package or a group that needs writing before S134.
 *   `shadow_would_allow` v2 is broader than v1 — an over-wide grant.
 */
final class UsageRightsShadow
{
    public function __construct(
        private readonly UsageRightsService $rights,
        private readonly UsageGrantRepository $grants,
        private readonly UsageCapabilityRegistry $capabilities,
        private readonly SiteFeatureService $features,
        private readonly SiteSettingService $settings,
        private readonly AudienceResolver $audiences,
    ) {
    }

    /**
     * One row per capability this installation actually runs.
     *
     * @return list<array{
     *     capability: UsageCapability, live: bool, liveReason: string,
     *     shadow: bool, paths: list<array<string, mixed>>, status: string
     * }>
     */
    public function forUser(?Utilisateur $user, ?int $venueId = null): array
    {
        $isAdmin = $user instanceof Utilisateur && in_array('ROLE_ADMIN', $user->getRoles(), true);
        $enforced = $this->settings->isUsageRightsEnforced();

        $rows = [];
        foreach ($this->capabilities->all() as $capability) {
            if (!$this->features->isEnabled($capability->featureKey)) {
                continue;
            }

            $live = $this->rights->verdict($user, $capability->key);
            // ⚠️ `Use`, not `Manage`. The live model has exactly one level, and
            // comparing it against the union of both would report every
            // manage-only package as a widening that is not there.
            $paths = $this->grants->paths($user, $capability->featureKey, UsageGrantAction::Use, $venueId);
            $shadow = $isAdmin || $paths !== [];

            $rows[] = [
                'capability' => $capability,
                'live' => $live->allowed,
                'liveReason' => $live->reason,
                'shadow' => $shadow,
                'paths' => $paths,
                'status' => match (true) {
                    $live->allowed === $shadow => 'agree',
                    $isAdmin => 'admin_bypass',
                    !$enforced && $live->allowed && !$shadow => 'enforcement_off',
                    $live->allowed && !$shadow => 'shadow_would_deny',
                    default => 'shadow_would_allow',
                },
            ];
        }

        return $rows;
    }

    /**
     * How many people each outcome applies to — the number that says whether
     * S134's activation is safe to attempt at all.
     *
     * ⚠️ Counted over the members handed in rather than over every account: the
     * caller pages, and a summary computed from a page while presented as a total
     * is exactly the kind of number that got a whole session built on it once.
     *
     * @param iterable<Utilisateur> $users
     *
     * @return array<string, int>
     */
    public function summary(iterable $users, ?int $venueId = null): array
    {
        $totals = ['agree' => 0, 'enforcement_off' => 0, 'admin_bypass' => 0, 'shadow_would_deny' => 0, 'shadow_would_allow' => 0];

        foreach ($users as $user) {
            foreach ($this->forUser($user, $venueId) as $row) {
                $totals[$row['status']] = ($totals[$row['status']] ?? 0) + 1;
            }
        }

        return $totals;
    }

    /** @return list<string> */
    public function audiencesOf(?Utilisateur $user): array
    {
        return $this->audiences->keysFor($user);
    }

    public function isReady(): bool
    {
        return $this->grants->tableExists();
    }
}
