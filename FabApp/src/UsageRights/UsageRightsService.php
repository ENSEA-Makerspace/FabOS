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
    public function verdict(?Utilisateur $user, string $capability, ?\DateTimeImmutable $from = null, ?\DateTimeImmutable $until = null, ?UsageScope $scope = null): UsageRightVerdict
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
            // ⚠️ The scope carries the venue, the resource and the interval into
            // the reader. It is rebuilt here rather than trusted whole so that a
            // caller which passed an interval but no scope still gets its
            // interval evaluated — the windows are the dimension a booking always
            // has something to say about.
            $askedScope = new UsageScope(
                $scope?->venueId,
                $scope?->reservableType,
                $scope?->reservableId,
                $scope?->categoryLabel,
                $from,
                $until,
            );

            $names = $this->settings->isUsageRightsV2Active($capability)
                ? array_values(array_unique(array_map(
                    static fn (array $path): string => $path['package'],
                    $this->grants->paths($user, $definition->featureKey, UsageGrantAction::Use, $askedScope),
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
    public function allowsReservableDuring(Utilisateur $user, ReservableType $type, \DateTimeImmutable $from, \DateTimeImmutable $until, ?int $venueId = null, ?int $reservableId = null, ?string $categoryLabel = null): bool
    {
        $feature = $this->registry->featureForReservable($type);
        if ($feature === null) {
            return true;
        }

        // ⚠️ **The resource itself, not only its kind** (S144b). "May you book a
        // machine" and "may you book *this* machine" are different questions, and
        // a package sold as access to the laser cutter answers only the second.
        // The type is what the feature already knew; the id and the category are
        // what a grant can now be narrowed to.
        $scope = new UsageScope($venueId, $type->value, $reservableId, $categoryLabel, $from, $until);

        return $this->verdict($user, $feature->key, $from, $until, $scope)->allowed;
    }

    /**
     * Which capability feature a kind of resource belongs to (S144c).
     *
     * The allowance service needs the same answer the booking gate already gets
     * from `featureForReservable()`, and asking the registry a second time from
     * another class is how two callers end up disagreeing about which feature a
     * space is under.
     */
    public function featureKeyForReservable(ReservableType $type): ?string
    {
        return $this->registry->featureForReservable($type)?->key;
    }

    public function isEnforced(): bool
    {
        return $this->settings->isUsageRightsEnforced();
    }

    /**
     * ⚠️ **`v2ShadowVerdict()` was deleted in S144a and must not come back.**
     * It was S111's diagnostic, it had no caller anywhere, and it was the last
     * thing in the codebase reading `USAGE_PACKAGE_GROUP_ASSIGNMENT` — the
     * duplicate group table that S134b converged away. Keeping a second,
     * unexercised implementation of "does this person hold this grant" is how the
     * two answers drift apart: this one still joined `UTILISATEUR_ROLE`, so it
     * would have disagreed with `paths()` about every group created since S133b.
     * The shadow screen reads `UsageGrantRepository::paths()`, which is the
     * reader the live chokepoints use.
     */
    private function now(): \DateTimeImmutable
    {
        return new \DateTimeImmutable('now', new \DateTimeZone($this->settings->getTimezone()));
    }
}
