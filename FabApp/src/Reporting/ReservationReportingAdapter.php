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
        $summary = $this->db->fetchAssociative("SELECT COUNT(*) total, COUNT(DISTINCT userId) users, COALESCE(SUM(TIMESTAMPDIFF(MINUTE, dateDebut, dateFin)), 0) minutes, COALESCE(SUM(statut = 'cancelled'), 0) cancelled FROM RESERVATION r WHERE {$where}", $parameters) ?: [];
        $daily = $this->db->fetchAllAssociative("SELECT DATE(dateDebut) day, COUNT(*) total, COALESCE(SUM(TIMESTAMPDIFF(MINUTE, dateDebut, dateFin)), 0) minutes FROM RESERVATION r WHERE {$where} GROUP BY DATE(dateDebut) ORDER BY day", $parameters);
        $top = $this->db->fetchAllAssociative("SELECT COALESCE(NULLIF(reservableLabel, ''), CONCAT('#', reservableId)) label, COUNT(*) total, COALESCE(SUM(TIMESTAMPDIFF(MINUTE, dateDebut, dateFin)), 0) minutes FROM RESERVATION r WHERE {$where} GROUP BY reservableId, reservableLabel ORDER BY total DESC LIMIT 10", $parameters);

        return new ReportData([
            'total' => (int) ($summary['total'] ?? 0),
            'users' => (int) ($summary['users'] ?? 0),
            'minutes' => (int) ($summary['minutes'] ?? 0),
            'cancelled' => (int) ($summary['cancelled'] ?? 0),
        ], $daily, $top);
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
