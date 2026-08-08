<?php

namespace App\UsageRights;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Feature\SiteFeatureRegistry;
use App\Reservation\ReservableType;
use App\Service\SiteSettingService;
use Symfony\Bundle\SecurityBundle\Security;

/** One entitlement answer for every web feature; RFID remains deliberately out of scope. */
final class UsageRightsService
{
    public function __construct(
        private readonly UsagePackageRepository $packages,
        private readonly SiteFeatureService $features,
        private readonly SiteFeatureRegistry $registry,
        private readonly SiteSettingService $settings,
        private readonly Security $security,
    ) {
    }

    public function allows(Utilisateur $user, string $feature): bool
    {
        if (!$this->settings->isUsageRightsEnforced()) {
            return true;
        }
        if ($this->security->isGranted('ROLE_ADMIN')) {
            return true;
        }
        if (!$this->features->isEnabled($feature)) {
            return false;
        }
        return $this->packages->allows($user, $feature, new \DateTimeImmutable('now', new \DateTimeZone($this->settings->getTimezone())));
    }

    public function allowsReservable(Utilisateur $user, ReservableType $type): bool
    {
        $feature = $this->registry->featureForReservable($type);

        return $feature === null || $this->allows($user, $feature->key);
    }

    public function isEnforced(): bool
    {
        return $this->settings->isUsageRightsEnforced();
    }
}
