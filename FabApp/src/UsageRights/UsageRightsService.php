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
    ) {
    }

    public function allows(Utilisateur $user, string $feature): bool
    {
        return $this->verdict($user, $feature)->allowed;
    }

    public function verdict(?Utilisateur $user, string $capability, ?\DateTimeImmutable $from = null, ?\DateTimeImmutable $until = null): UsageRightVerdict
    {
        $definition = $this->capabilities->get($capability);
        $from ??= $this->now();
        $shouldReadGrants = $definition !== null
            && $this->isEnforced()
            && $this->features->isEnabled($definition->featureKey)
            && $user instanceof Utilisateur
            && !in_array('ROLE_ADMIN', $user->getRoles(), true);
        $names = $shouldReadGrants
            ? $this->packages->grantingPackages($user, $definition->featureKey, $from, $until)
            : [];

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

    public function allowsReservableDuring(Utilisateur $user, ReservableType $type, \DateTimeImmutable $from, \DateTimeImmutable $until): bool
    {
        $feature = $this->registry->featureForReservable($type);

        return $feature === null || $this->verdict($user, $feature->key, $from, $until)->allowed;
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
