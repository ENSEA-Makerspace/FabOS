<?php

namespace App\Portal;

use Doctrine\DBAL\Connection;

/** Read-only S105 inventory; no consolidation is performed here. */
final class PortalConsolidationReport
{
    public function __construct(private readonly PortalRepository $portals, private readonly Connection $db) {}

    /** @return list<array<string, mixed>> */
    public function all(): array
    {
        $report = [];
        foreach ($this->portals->all() as $portal) {
            $scope = $portal->isDefault ? PortalContext::GLOBAL_SCOPE : $portal->id;
            $report[] = [
                'portal' => $portal,
                'scope' => $scope,
                'settings' => $this->rows('SELECT settingKey, settingValue FROM SITE_SETTING WHERE portalId = :scope ORDER BY settingKey', $scope),
                'features' => $this->rows('SELECT moduleKey, enabled FROM SITE_MODULE WHERE portalId = :scope ORDER BY moduleKey', $scope),
                'packages' => $this->rows('SELECT id, name, active, fullAccess FROM USAGE_PACKAGE WHERE portalId = :scope ORDER BY name', $scope),
            ];
        }

        return $report;
    }

    /** @return list<array<string, mixed>> */
    private function rows(string $sql, int $scope): array
    {
        try {
            return $this->db->fetchAllAssociative($sql, ['scope' => $scope]);
        } catch (\Throwable) {
            return [];
        }
    }
}
