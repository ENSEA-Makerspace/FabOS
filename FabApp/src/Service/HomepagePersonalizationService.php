<?php

namespace App\Service;

use App\Entity\HomepageUserPreference;
use App\Entity\Utilisateur;
use App\Repository\HomepageUserPreferenceRepository;
use Doctrine\DBAL\Exception as DbalException;
use Doctrine\ORM\Exception\ORMException;
use Symfony\Component\Security\Core\User\UserInterface;

class HomepagePersonalizationService
{
    public function __construct(
        private readonly HomepageVisibilityService $visibility,
        private readonly HomepageUserPreferenceRepository $preferences,
    ) {
    }

    /** @param array<string, bool> $visibilityMap @return string[] */
    public function getSectionOrder(?UserInterface $user, array $visibilityMap): array
    {
        $visibleKeys = $this->getVisibleKeys($visibilityMap);
        if (!$user instanceof Utilisateur) {
            return $visibleKeys;
        }

        $preference = $this->findPreference($user);
        if ($preference === null) {
            return $visibleKeys;
        }

        return $this->normalizeOrder($preference->getSectionOrderArray(), $visibleKeys);
    }

    /** @param array<string, bool> $visibilityMap @return array<int, array{sectionKey: string, label: string}> */
    public function getPersonalizationRows(?UserInterface $user, array $visibilityMap): array
    {
        $labels = $this->getSectionLabels();
        $rows = [];
        foreach ($this->getSectionOrder($user, $visibilityMap) as $sectionKey) {
            $rows[] = [
                'sectionKey' => $sectionKey,
                'label' => $labels[$sectionKey] ?? $sectionKey,
            ];
        }

        return $rows;
    }

    /** @param string[] $submittedOrder @param array<string, bool> $visibilityMap @return string[] */
    public function sanitizeSubmittedOrder(array $submittedOrder, array $visibilityMap): array
    {
        return $this->normalizeOrder($submittedOrder, $this->getVisibleKeys($visibilityMap));
    }

    /** @return array<string, string> */
    public function getSectionLabels(): array
    {
        $labels = [];
        foreach ($this->visibility->getAdminRows() as $row) {
            $labels[$row['sectionKey']] = $row['label'];
        }

        return $labels;
    }

    /** @param array<string, bool> $visibilityMap @return string[] */
    private function getVisibleKeys(array $visibilityMap): array
    {
        $orderedKeys = [];
        foreach ($this->visibility->getAdminRows() as $row) {
            $sectionKey = $row['sectionKey'];
            if (($visibilityMap[$sectionKey] ?? false) === true) {
                $orderedKeys[] = $sectionKey;
            }
        }

        return $orderedKeys;
    }

    /** @param string[] $order @param string[] $visibleKeys @return string[] */
    private function normalizeOrder(array $order, array $visibleKeys): array
    {
        $allowed = array_flip($visibleKeys);
        $result = [];

        foreach ($order as $sectionKey) {
            if (!is_string($sectionKey) || !isset($allowed[$sectionKey]) || in_array($sectionKey, $result, true)) {
                continue;
            }

            $result[] = $sectionKey;
        }

        foreach ($visibleKeys as $sectionKey) {
            if (!in_array($sectionKey, $result, true)) {
                $result[] = $sectionKey;
            }
        }

        return $result;
    }

    private function findPreference(Utilisateur $user): ?HomepageUserPreference
    {
        try {
            return $this->preferences->findOneForUser($user);
        } catch (DbalException|ORMException $exception) {
            return null;
        } catch (\RuntimeException $exception) {
            return null;
        }
    }
}
