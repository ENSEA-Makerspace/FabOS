<?php

declare(strict_types=1);

namespace App\Design;

use App\Reporting\ReportingRegistry;
use App\Reporting\ReportScope;

/**
 * « Rapports » qui conduisent à l'action (proposition `rapports`, 2026-10-01,
 * d'après la planche `finalsurface/03-rapports.png`).
 *
 * ⚠️ LECTURE SEULE et rien de neuf : les chiffres viennent du MÊME adaptateur que
 * `/admin/reporting/{workspace}` ({@see ReportingRegistry}), appelé deux fois —
 * sur la période, puis sur la période de même durée qui la précède, pour la
 * comparaison. Aucun graphique, aucune donnée inventée.
 *
 * Règles de « À retenir » (au plus TROIS constats, dans cet ordre de priorité ;
 * une règle sans seuil franchi ne produit rien) :
 *  1. concentration : la ressource la plus réservée porte au moins 40 % des
 *     réservations actives, sur au moins 10 réservations et au moins 2 ressources
 *     réservées → « ajouter une machine / un espace ? » ;
 *  2. annulations : au moins 10 demandes et 20 % annulées (le seuil de la page
 *     actuelle) → règles de réservation ;
 *  3. variation : au moins 10 réservations dans l'une des deux périodes et un
 *     écart d'au moins 30 % → lecture des réservations ;
 *  4. inactives : des ressources en service sans aucune réservation active.
 *
 * Le journal d'accès, les prêts et les adhésions ne sont PAS lus ici : le
 * reporting actuel ne les calcule pas, la proposition ne les invente pas.
 */
final class ReportingBrief
{
    public const PERIODS = [7, 30, 90];

    private const SHARE_MIN = 0.4;
    private const SHARE_MIN_TOTAL = 10;
    private const CANCEL_MIN_REQUESTED = 10;
    private const CANCEL_MIN_RATE = 0.2;
    private const TREND_MIN = 10;
    private const TREND_RATE = 0.3;
    private const MAX_FINDINGS = 3;

    public function __construct(private readonly ReportingRegistry $reporting)
    {
    }

    /**
     * @return array<string, mixed>
     */
    public function build(string $workspace, int $days): array
    {
        $workspace = $workspace === 'spaces' ? 'spaces' : 'equipment';
        $days = \in_array($days, self::PERIODS, true) ? $days : 30;
        $equipment = $workspace === 'equipment';

        $today = new \DateTimeImmutable('today');
        $from = $today->modify('-' . ($days - 1) . ' days');
        $until = $today->modify('+1 day');
        $previousFrom = $from->modify('-' . $days . ' days');

        $adapter = $this->reporting->forWorkspace($workspace);
        $report = $adapter->report(new ReportScope($workspace, $from, $until));
        $previous = $adapter->report(new ReportScope($workspace, $previousFrom, $from));

        $summary = $report->summary;
        $total = (int) ($summary['total'] ?? 0);
        $detailRoute = $equipment ? 'app_machine_detail' : 'app_place_detail';
        $noun = $equipment ? 'machine' : 'espace';

        $top = [];
        foreach ($report->top as $row) {
            $top[] = [
                'id' => (int) $row['id'],
                'label' => (string) $row['label'],
                'total' => (int) $row['total'],
                'hours' => round(((int) $row['minutes']) / 60, 1),
                'share' => $total > 0 ? (int) round(100 * (int) $row['total'] / $total) : 0,
                'route' => $detailRoute,
                'params' => ['id' => (int) $row['id']],
            ];
        }

        $findings = [];

        // 1. Concentration.
        if ($top !== [] && $total >= self::SHARE_MIN_TOTAL && \count($top) >= 2 && $top[0]['total'] / $total >= self::SHARE_MIN) {
            $findings[] = [
                'signal' => 'caution',
                'icon' => 'bolt',
                'text' => $top[0]['label'] . ' porte ' . $top[0]['share'] . ' % des réservations de la période.',
                'hint' => 'Une seule ' . $noun . ' concentre la demande : en ajouter une, ou rediriger vers les autres.',
                'verb' => 'Voir ' . ($equipment ? 'la machine' : 'l’espace'),
                'route' => $detailRoute,
                'params' => ['id' => $top[0]['id']],
            ];
        }

        // 2. Annulations (même règle que la page actuelle).
        $requested = (int) ($summary['requested'] ?? 0);
        $cancelled = (int) ($summary['cancelled'] ?? 0);
        if ($requested >= self::CANCEL_MIN_REQUESTED && $cancelled / $requested >= self::CANCEL_MIN_RATE) {
            $findings[] = [
                'signal' => 'caution',
                'icon' => 'warning',
                'text' => (int) round(100 * $cancelled / $requested) . ' % des demandes sont annulées (' . $cancelled . ' sur ' . $requested . ').',
                'hint' => 'Un délai d’annulation ou une limite de réservations aide souvent.',
                'verb' => 'Revoir les règles de réservation',
                'route' => 'app_admin_booking_policies',
                'params' => [],
            ];
        }

        // 3. Variation par rapport à la période précédente.
        $before = (int) ($previous->summary['total'] ?? 0);
        $trend = null;
        if ($before > 0 || $total > 0) {
            $trend = $before > 0 ? (int) round(100 * ($total - $before) / $before) : null;
        }
        if (max($total, $before) >= self::TREND_MIN && $before > 0 && abs($total - $before) / $before >= self::TREND_RATE) {
            $up = $total > $before;
            $findings[] = [
                'signal' => $up ? 'go' : 'wait',
                'icon' => $up ? 'bolt' : 'history',
                'text' => 'Les réservations ' . ($up ? 'augmentent' : 'baissent') . ' de ' . abs($trend) . ' % sur ' . $days . ' jours (' . $before . ' avant, ' . $total . ' maintenant).',
                'hint' => 'Comparé aux ' . $days . ' jours précédents, annulations exclues.',
                'verb' => 'Voir les réservations',
                'route' => 'app_admin_reservations',
                'params' => [],
            ];
        }

        // 4. Ressources en service jamais réservées.
        $idle = $report->idle;
        if ($idle !== []) {
            $n = \count($idle);
            $findings[] = [
                'signal' => 'muted',
                'icon' => 'hourglass',
                'text' => $n . ($equipment ? ' machine' : ' espace') . ($n > 1 ? 's' : '') . ' en service sans aucune réservation sur ' . $days . ' jours.',
                'hint' => 'Vérifiez qu’' . ($n > 1 ? 'elles sont connues' : ($equipment ? 'elle est connue' : 'il est connu')) . ' des membres, sinon archivez.',
                'verb' => $equipment ? 'Voir les machines' : 'Voir les espaces',
                'route' => $equipment ? 'app_admin_machines' : 'app_admin_places',
                'params' => [],
            ];
        }

        $periods = [];
        foreach (self::PERIODS as $p) {
            $periods[] = ['label' => $p . ' jours', 'query' => ['jours' => $p], 'active' => $p === $days];
        }

        return [
            'workspace' => $workspace,
            'days' => $days,
            'from' => $from,
            'until' => $today,
            'periods' => $periods,
            'summary' => [
                'total' => $total,
                'users' => (int) ($summary['users'] ?? 0),
                'hours' => round(((int) ($summary['minutes'] ?? 0)) / 60, 1),
                'cancelled' => $cancelled,
                'before' => $before,
                'trend' => $trend,
            ],
            'findings' => \array_slice($findings, 0, self::MAX_FINDINGS),
            'top' => $top,
            'topLabel' => $equipment ? 'Machines les plus réservées' : 'Espaces les plus réservés',
            'idle' => array_map(static fn (array $r): array => $r + ['route' => $equipment ? 'app_machine_detail' : 'app_place_detail'], \array_slice($idle, 0, 12)),
            'nounPlural' => $equipment ? 'machines' : 'espaces',
        ];
    }
}
