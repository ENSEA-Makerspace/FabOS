<?php

namespace App\Reporting;

use Doctrine\DBAL\Connection;

/** Initial reporting adapter: aggregate reservations, never expose member identity or motif. */
final class ReservationReportingAdapter implements ReportingAdapter
{
    public function __construct(private readonly Connection $db) {}

    public function supports(string $workspace): bool { return in_array($workspace, ['equipment', 'spaces'], true); }

    public function report(ReportScope $scope): ReportData
    {
        [$where, $parameters] = $this->where($scope);
        // 🔴 S195 — l'USAGE ne compte que les réservations actives. Avant, le
        // total, les heures et le classement comptaient aussi les ANNULÉES :
        // mesuré sur la boîte de dev, 15 machines annulées sur 38 — un total
        // gonflé de 65 %, et une machine « très utilisée » par des annulations.
        $active = "r.statut NOT IN ('cancelled', 'declined')";
        $summary = $this->db->fetchAssociative("SELECT COALESCE(SUM({$active}), 0) total, COUNT(DISTINCT CASE WHEN {$active} THEN userId END) users, COALESCE(SUM(CASE WHEN {$active} THEN TIMESTAMPDIFF(MINUTE, dateDebut, dateFin) END), 0) minutes, COALESCE(SUM(statut = 'cancelled'), 0) cancelled, COUNT(*) requested FROM RESERVATION r WHERE {$where}", $parameters) ?: [];
        $daily = $this->db->fetchAllAssociative("SELECT DATE(dateDebut) day, COUNT(*) total, COALESCE(SUM(TIMESTAMPDIFF(MINUTE, dateDebut, dateFin)), 0) minutes FROM RESERVATION r WHERE {$where} AND {$active} GROUP BY DATE(dateDebut) ORDER BY day", $parameters);
        $top = $this->db->fetchAllAssociative("SELECT reservableId id, COALESCE(NULLIF(reservableLabel, ''), CONCAT('#', reservableId)) label, COUNT(*) total, COALESCE(SUM(TIMESTAMPDIFF(MINUTE, dateDebut, dateFin)), 0) minutes FROM RESERVATION r WHERE {$where} AND {$active} GROUP BY reservableId, reservableLabel ORDER BY total DESC LIMIT 10", $parameters);

        return new ReportData([
            'total' => (int) ($summary['total'] ?? 0),
            'users' => (int) ($summary['users'] ?? 0),
            'minutes' => (int) ($summary['minutes'] ?? 0),
            'cancelled' => (int) ($summary['cancelled'] ?? 0),
            'requested' => (int) ($summary['requested'] ?? 0),
        ], $daily, $top, $this->idle($scope, $parameters));
    }

    /**
     * S195 — les ressources en service qu'AUCUNE réservation active n'a touchées
     * sur la période. Une machine hors service ou archivée n'est pas « oubliée ».
     *
     * @param array<string, mixed> $parameters
     *
     * @return list<array{id: int, label: string}>
     */
    private function idle(ReportScope $scope, array $parameters): array
    {
        $type = $parameters['type'];
        // ⚠️ « En service » = pas en maintenance ni en panne, lu dans `Machine` :
        // la première version filtrait `statut = 'active'`, un mot qu'AUCUNE
        // machine ne porte (`disponible`, `idle`…) — le constat ne trouvait jamais rien.
        [$table, $label, $inService] = $type === 'machine'
            ? ['MACHINE', 'nom', 'AND LOWER(resource.statut) NOT IN (:outOfService)']
            : ['PLACE', 'nom', ''];
        $types = [];
        if ($type === 'machine') {
            $parameters['outOfService'] = \App\Entity\Machine::outOfServiceStatuses();
            $types['outOfService'] = \Doctrine\DBAL\ArrayParameterType::STRING;
        }
        $venue = $scope->venueId !== null ? 'AND resource.venueId = :venue' : '';
        try {
            $rows = $this->db->fetchAllAssociative(
                "SELECT resource.id, resource.{$label} label FROM {$table} resource
                 WHERE resource.archivedAt IS NULL {$inService} {$venue}
                   AND NOT EXISTS (SELECT 1 FROM RESERVATION r WHERE r.reservableType = :type AND r.reservableId = resource.id
                                   AND r.dateDebut >= :from AND r.dateDebut < :until AND r.statut NOT IN ('cancelled', 'declined'))
                 ORDER BY resource.{$label}",
                $parameters,
                $types,
            );
        } catch (\Throwable) {
            return [];
        }

        return array_map(static fn (array $row): array => ['id' => (int) $row['id'], 'label' => (string) $row['label']], $rows);
    }

    public function export(ReportScope $scope): iterable
    {
        foreach ($this->report($scope)->daily as $row) {
            yield ['date' => $row['day'], 'reservations' => (int) $row['total'], 'minutes' => (int) $row['minutes']];
        }
    }

    /** @return array{string, array<string, mixed>} */
    private function where(ReportScope $scope): array
    {
        $type = $scope->workspace === 'equipment' ? 'machine' : 'place';
        $parameters = [
            'type' => $type,
            'from' => $scope->from->format('Y-m-d H:i:s'),
            'until' => $scope->until->format('Y-m-d H:i:s'),
        ];
        $where = 'r.reservableType = :type AND r.dateDebut >= :from AND r.dateDebut < :until';
        if ($scope->venueId !== null) {
            $table = $type === 'machine' ? 'MACHINE' : 'PLACE';
            $where .= " AND EXISTS (SELECT 1 FROM {$table} resource WHERE resource.id = r.reservableId AND resource.venueId = :venue)";
            $parameters['venue'] = $scope->venueId;
        }

        return [$where, $parameters];
    }
}
