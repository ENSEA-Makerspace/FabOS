<?php

namespace App\UsageRights;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Feature\SiteFeatureRegistry;
use App\Reservation\ReservableType;
use App\Service\SiteSettingService;

/** One entitlement answer for every web feature; RFID remains deliberately out of scope. */
final class UsageRightsService
{
    public function __construct(
        private readonly UsagePackageRepository $packages,
        private readonly SiteFeatureService $features,
        private readonly SiteFeatureRegistry $registry,
        private readonly UsageCapabilityRegistry $capabilities,
        private readonly UsageRightDecisionPolicy $policy,
        private readonly SiteSettingService $settings,
        // S134. Read only for capabilities the operator has moved; see `verdict()`.
        private readonly UsageGrantRepository $grants,
    ) {
    }

    public function allows(Utilisateur $user, string $feature): bool
    {
        return $this->verdict($user, $feature)->allowed;
    }

    /**
     * What the LEGACY store holds for this person, with no policy applied (S134).
     *
     * ⚠️ **The counterfactual the shadow page needs, and it cannot be got from
     * `verdict()`.** With enforcement off — the state every installation is in
     * today — `verdict()` short-circuits to `not_enforced` and answers "allowed"
     * for everybody, so a shadow comparing against it reports nobody at risk on
     * exactly the installations the activation gate exists to protect. This
     * answers the narrower question the comparison actually needs: does this
     * member hold a package covering this feature, right now, regardless of
     * whether anything is being enforced.
     *
     * ⚠️ Read-only and unused by any authorisation path.
     *
     * @return list<string>
     */
    public function legacyPackages(?Utilisateur $user, string $featureKey, ?\DateTimeImmutable $at = null): array
    {
        if (!$user instanceof Utilisateur) {
            return [];
        }

        return $this->packages->grantingPackages($user, $featureKey, $at ?? $this->now(), null);
    }

    /**
     * ⚠️ **S134 — where the packages come from is now per capability.**
     *
     * The precedence rules below are untouched: `UsageRightDecisionPolicy` still
     * decides, and it still decides on a list of package names. What changed is
     * which store that list is read from, and it is read from grants v2 **only**
     * for a capability an operator has explicitly moved — `isUsageRightsV2Active`,
     * default false, one switch per chokepoint.
     *
     * Two properties this arrangement has, and both are why it is shaped this way:
     *
     *  - **A capability that has not been moved is byte-for-byte unchanged.** The
     *    legacy call is the same call with the same arguments; no capability can
     *    be affected by a change to another one.
     *  - **It cannot refuse more than enforcement already allows it to.** With
     *    enforcement off the policy returns `not_enforced` before any store is
     *    read, so flipping a chokepoint on a non-enforcing installation changes
     *    nothing at all. That is the safe direction for two flags to compose, and
     *    it means the switch can be moved and watched before it can bite.
     *
     * ⚠️ Grants v2 is asked for `Use`. `Manage` never confers `Use` and this
     * method answers the Use question; a Manage-only package must not open a
     * booking, which asking for both would have let it do.
     */
    public function verdict(?Utilisateur $user, string $capability, ?\DateTimeImmutable $from = null, ?\DateTimeImmutable $until = null, ?int $venueId = null): UsageRightVerdict
    {
        $definition = $this->capabilities->get($capability);
        $from ??= $this->now();
        $shouldReadGrants = $definition !== null
            && $this->isEnforced()
            && $this->features->isEnabled($definition->featureKey)
            && $user instanceof Utilisateur
            && !in_array('ROLE_ADMIN', $user->getRoles(), true);

        $names = [];
        if ($shouldReadGrants) {
            $names = $this->settings->isUsageRightsV2Active($capability)
                ? array_values(array_unique(array_map(
                    static fn (array $path): string => $path['package'],
                    $this->grants->paths($user, $definition->featureKey, UsageGrantAction::Use, $venueId, $from),
                )))
                : $this->packages->grantingPackages($user, $definition->featureKey, $from, $until);
        }

        return $this->policy->decide(
            $capability,
            $definition !== null,
            $this->isEnforced(),
            $definition !== null && $this->features->isEnabled($definition->featureKey),
            $user instanceof Utilisateur,
            $user instanceof Utilisateur && in_array('ROLE_ADMIN', $user->getRoles(), true),
            $names,
        );
    }

    /** @return list<array{capability:UsageCapability,verdict:UsageRightVerdict}> */
    public function overview(?Utilisateur $user): array
    {
        $rows = [];
        foreach ($this->capabilities->all() as $capability) {
            if ($this->features->isEnabled($capability->featureKey)) {
                $rows[] = ['capability' => $capability, 'verdict' => $this->verdict($user, $capability->key)];
            }
        }

        return $rows;
    }

    public function allowsReservable(Utilisateur $user, ReservableType $type): bool
    {
        $feature = $this->registry->featureForReservable($type);

        return $feature === null || $this->allows($user, $feature->key);
    }

    /**
     * ⚠️ **`$venueId` is what makes a venue-scoped grant mean anything** (S134b).
     * Until it existed, `verdict()` asked "does this person hold this capability
     * anywhere", every caller passed nothing, and a grant limited to one location
     * behaved exactly like an unrestricted one — the dimension was stored, read,
     * and never enforced. The booking chokepoint passes the location of the
     * resource being booked, which is the question that was actually being asked
     * all along.
     *
     * ⚠️ Null stays permissive on purpose. A caller with no location — an
     * overview, an appointment with a person — must not be refused by a
     * restriction it cannot evaluate.
     */
    public function allowsReservableDuring(Utilisateur $user, ReservableType $type, \DateTimeImmutable $from, \DateTimeImmutable $until, ?int $venueId = null): bool
    {
        $feature = $this->registry->featureForReservable($type);

        return $feature === null || $this->verdict($user, $feature->key, $from, $until, $venueId)->allowed;
    }

    public function isEnforced(): bool
    {
        return $this->settings->isUsageRightsEnforced();
    }

    /** S111 diagnostic only — callers must never substitute it for verdict(). */
    public function v2ShadowVerdict(?Utilisateur $user, string $feature, UsageGrantAction $action, ?int $venueId, ?\DateTimeImmutable $at = null): UsageRightVerdict
    {
        $at ??= $this->now();
        $packages = $user instanceof Utilisateur
            ? $this->packages->v2GrantingPackages($user, $feature, $action, $venueId, $at)
            : [];

        return new UsageRightVerdict(
            $feature . '.' . $action->value,
            $packages !== [],
            false,
            $packages !== [] ? 'shadow_granted' : 'shadow_missing_grant',
            $packages,
        );
    }

    private function now(): \DateTimeImmutable
    {
        return new \DateTimeImmutable('now', new \DateTimeZone($this->settings->getTimezone()));
    }
}
