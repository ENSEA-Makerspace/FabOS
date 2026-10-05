<?php

declare(strict_types=1);

namespace App\Page;

use App\Entity\AccessRfidLog;
use App\Entity\Reservation;
use App\Repository\AccessRfidLogRepository;
use App\Repository\BadgeRepository;
use App\Repository\FormationRepository;
use App\Repository\LoanRepository;
use App\Repository\MachineRepository;
use App\Repository\MaintenanceTaskRepository;
use App\Repository\ProgressionRepository;
use App\Repository\ReservationRepository;
use App\Repository\RfidReaderRepository;
use App\Repository\UtilisateurRepository;
use App\Rfid\AccessIncident;
use App\Rfid\ReaderHealth;
use App\Service\MachineReports;
use App\Service\MaterialStock;
use App\Service\Feedback;
use App\Feature\SiteFeatureService;
use App\Training\PracticalQueue;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * « Ce qui demande votre attention » pour l'accueil admin (proposition
 * `admin-attention`, 2026-10-01, d'après la planche
 * `equipement/equipment-operational-overview.png`).
 *
 * ⚠️ LECTURE SEULE et rien de neuf : chaque groupe est un tri de ce que les
 * écrans dédiés lisent déjà (machines, maintenance, lecteurs, journal RFID,
 * réservations, prêts, file pratique, comptes). Un groupe sans ligne n'est pas
 * rendu ; un groupe sans source n'existe pas ici.
 *
 * Forme d'une ligne : `title`, `where` (texte), `when` (DateTimeInterface|null)
 * + `whenKind` (`utc` = horodatage machine → `|lab_date`, `wall` = heure saisie
 * → `|date`), `state` {label, signal}, `verb`, `route`, `params`, et pour un
 * refus d'accès `fix` (clé de `_cell_fix`).
 */
final class AdminAttention
{
    /** Lignes montrées par groupe : le reste passe par « Voir tout ». */
    private const LIMIT = 6;

    public function __construct(
        private readonly MachineRepository $machines,
        private readonly MaintenanceTaskRepository $maintenance,
        private readonly RfidReaderRepository $readers,
        private readonly ReaderHealth $readerHealth,
        private readonly AccessRfidLogRepository $rfidLogs,
        private readonly AccessIncident $incidents,
        private readonly ReservationRepository $reservations,
        private readonly LoanRepository $loans,
        private readonly PracticalQueue $practicalQueue,
        private readonly UtilisateurRepository $users,
        private readonly FormationRepository $formations,
        private readonly BadgeRepository $badges,
        private readonly ProgressionRepository $progressions,
        private readonly TranslatorInterface $translator,
        private readonly Feedback $feedback,
        private readonly SiteFeatureService $features,
        private readonly MachineReports $machineReports,
        private readonly MaterialStock $stock,
    ) {
    }

    /** @return array{total: int, groups: list<array<string, mixed>>, stats: array<string, int>} */
    public function build(): array
    {
        $groups = array_values(array_filter([
            $this->unavailableMachines(),
            $this->reportedBreakdowns(),
            $this->maintenanceDue(),
            $this->offlineReaders(),
            $this->refusedAccess(),
            $this->pendingReservations(),
            $this->overdueLoans(),
            $this->practicalValidations(),
            $this->pendingAccounts(),
            $this->userFeedback(),
            $this->lowStock(),
        ]));

        return [
            'total' => array_sum(array_column($groups, 'count')),
            'groups' => $groups,
            'stats' => [
                'users' => $this->users->count([]),
                'machines' => $this->machines->count([]),
                'formations' => $this->formations->countVisible(),
                'reservations' => $this->reservations->count([]),
                'rfidLogs' => $this->rfidLogs->count([]),
                'badges' => $this->badges->count([]),
                'completedFormations' => $this->progressions->countCompletedVisible(),
            ],
        ];
    }

    /** Une clé `admin_attention.*` traduite. */
    private function t(string $key): string
    {
        return $this->translator->trans('admin_attention.' . $key);
    }

    /** @param list<array<string, mixed>> $rows */
    private function group(string $key, string $title, string $icon, array $rows, string $allRoute, array $allParams, string $allLabel): ?array
    {
        if ($rows === []) {
            return null;
        }

        return [
            'key' => $key, 'title' => $title, 'icon' => $icon,
            'count' => count($rows), 'rows' => array_slice($rows, 0, self::LIMIT),
            'all' => ['route' => $allRoute, 'params' => $allParams, 'label' => $allLabel],
        ];
    }

    /** Machine::getStatusKey() : maintenance ou panne (hors archivées). */
    private function unavailableMachines(): ?array
    {
        $rows = [];
        foreach ($this->machines->findLive() as $machine) {
            $key = $machine->getStatusKey();
            if ($key === 'machines.st_available') {
                continue;
            }
            $rows[] = [
                'title' => $machine->getNom(),
                'where' => $machine->getLocalisation() ?: '',
                'when' => null, 'whenKind' => 'wall',
                'state' => $key === 'machines.st_broken'
                    ? ['label' => $this->t('st_broken'), 'signal' => 'stop']
                    : ['label' => $this->t('st_maintenance'), 'signal' => 'caution'],
                'verb' => $this->t('verb_machine'), 'route' => 'app_machine_detail', 'params' => ['id' => $machine->getId()],
            ];
        }

        return $this->group('machines', $this->t('g_machines'), 'tool', $rows, 'app_admin_machines', [], $this->t('all_machines'));
    }

    /** S205 — pannes signalées par QR, pas encore résolues (fonction éteinte ou migration absente : aucun groupe). */
    private function reportedBreakdowns(): ?array
    {
        if (!$this->features->allowsSurface('machine_reports') || !$this->machineReports->isReady()) {
            return null;
        }
        $rows = [];
        foreach ($this->machineReports->all(MachineReports::OPEN, null, 50) as $report) {
            $rows[] = [
                'title' => $report['description'],
                'where' => (string) $report['machineName'],
                'when' => $report['createdAt'], 'whenKind' => 'utc',
                'state' => ['label' => $this->t('st_report'), 'signal' => 'caution'],
                'verb' => $this->t('verb_report'), 'route' => 'app_admin_machine_reports', 'params' => ['statut' => 'open'],
            ];
        }

        return $this->group('reports', $this->t('g_reports'), 'warning', $rows, 'app_admin_machine_reports', ['statut' => 'open'], $this->t('all_reports'));
    }

    /** Tâches ouvertes en retard ou à échéance dans les 7 jours. */
    private function maintenanceDue(): ?array
    {
        $rows = [];
        foreach ($this->maintenance->findOpenSafe() as $task) {
            $status = $task->getEffectiveStatus();
            if ($status !== 'overdue' && $status !== 'due_soon') {
                continue;
            }
            $rows[] = [
                'title' => $task->getTitle(),
                'where' => $task->getMachine()?->getNom() ?? '',
                'when' => $task->getDueDate(), 'whenKind' => 'date',
                'state' => $status === 'overdue'
                    ? ['label' => $this->t('st_overdue'), 'signal' => 'stop']
                    : ['label' => $this->t('st_due_soon'), 'signal' => 'caution'],
                'verb' => $this->t('verb_task'), 'route' => 'app_admin_maintenance_edit', 'params' => ['id' => $task->getId()],
                '_sort' => $task->getDueDate()?->getTimestamp() ?? 0,
            ];
        }
        usort($rows, static fn (array $a, array $b): int => $a['_sort'] <=> $b['_sort']);

        return $this->group('maintenance', $this->t('g_maintenance'), 'history', $rows, 'app_admin_maintenance', [], $this->t('all_maintenance'));
    }

    /** ReaderHealth : seuls les lecteurs « hors ligne » (pas les désactivés, pas les jamais vus). */
    private function offlineReaders(): ?array
    {
        $rows = [];
        foreach ($this->readers->findForAdmin() as $reader) {
            if ($this->readerHealth->of($reader)['state'] !== ReaderHealth::OFFLINE) {
                continue;
            }
            $rows[] = [
                'title' => $reader->getName(),
                'where' => $reader->getMachine()?->getNom() ?? $reader->getAccessPoint()?->getNom() ?? '',
                'when' => $reader->getLastSeenAt(), 'whenKind' => 'utc', 'whenPrefix' => $this->t('last_seen') . ' ',
                'state' => ['label' => $this->t('st_offline'), 'signal' => 'stop'],
                'verb' => $this->t('verb_reader'), 'route' => 'app_admin_rfid_reader_show', 'params' => ['id' => $reader->getId()],
            ];
        }

        return $this->group('readers', $this->t('g_readers'), 'bolt', $rows, 'app_admin_rfid_readers', [], $this->t('all_readers'));
    }

    /** Journal RFID : refusés des 7 derniers jours, avec le verbe correctif d'AccessIncident. */
    private function refusedAccess(): ?array
    {
        $rows = [];
        foreach ($this->rfidLogs->search(7, null, null, 'no', 50) as $log) {
            /** @var AccessRfidLog $log */
            $fix = $this->incidents->of($log);
            $rows[] = [
                'title' => $log->getUtilisateur()?->getDisplayName() ?? $this->t('unknown_badge'),
                'where' => $log->getMachine()?->getNom() ?? $log->getReader()?->getName() ?? '',
                'when' => $log->getCreatedAt(), 'whenKind' => 'utc',
                'state' => ['label' => $this->t('st_refused'), 'signal' => 'stop'],
                'status' => $log->getStatus(),
                'fix' => $fix,
                'verb' => $this->t('verb_log'), 'route' => 'app_admin_access_rfid_logs', 'params' => ['days' => 7, 'result' => 'no'],
            ];
        }

        return $this->group('access', $this->t('g_access'), 'forbidden', $rows, 'app_admin_access_rfid_logs', ['days' => 7, 'result' => 'no'], $this->t('all_access'));
    }

    /** Réservations au statut « pending ». */
    private function pendingReservations(): ?array
    {
        $rows = [];
        foreach ($this->reservations->findForAdminFilters(['statut' => Reservation::STATUS_PENDING]) as $reservation) {
            $rows[] = [
                'title' => $reservation->getReservableLabel() ?: $this->t('reservation'),
                'where' => $reservation->getUtilisateur()?->getDisplayName() ?? '',
                'when' => $reservation->getDateDebut(), 'whenKind' => 'wall',
                'state' => ['label' => $this->t('st_to_validate'), 'signal' => 'wait'],
                'verb' => $this->t('verb_review'), 'route' => 'app_reservation_detail', 'params' => ['id' => $reservation->getId()],
            ];
        }

        return $this->group('reservations', $this->t('g_reservations'), 'calendar', $rows, 'app_admin_reservations', ['statut' => Reservation::STATUS_PENDING], $this->t('all_reservations'));
    }

    /** Prêts dont Loan::getEffectiveStatus() vaut « overdue ». */
    private function overdueLoans(): ?array
    {
        $rows = [];
        foreach ($this->loans->findAllSafe() as $loan) {
            if ($loan->getEffectiveStatus() !== 'overdue') {
                continue;
            }
            $rows[] = [
                'title' => $loan->getItem()?->getName() ?? $this->t('loan_item'),
                'where' => $loan->getBorrowerDisplay(),
                'when' => $loan->getExpectedReturnDate(), 'whenKind' => 'wall', 'whenPrefix' => $this->t('due_on') . ' ',
                'state' => ['label' => $this->t('st_overdue'), 'signal' => 'stop'],
                'verb' => $this->t('verb_loan'), 'route' => 'app_admin_loans', 'params' => [],
            ];
        }

        return $this->group('loans', $this->t('g_loans'), 'box', $rows, 'app_admin_loans', [], $this->t('all_loans'));
    }

    /** PracticalQueue : théorie finie, pratique à valider. */
    private function practicalValidations(): ?array
    {
        $rows = [];
        foreach ($this->practicalQueue->pending() as $row) {
            $rows[] = [
                'title' => $row['user']->getDisplayName(),
                'where' => (string) $row['formation']->getTitre(),
                'when' => $row['since'], 'whenKind' => 'wall', 'whenPrefix' => $this->t('since') . ' ',
                'state' => ['label' => $this->t('st_practical'), 'signal' => 'wait'],
                'verb' => $this->t('verb_queue'), 'route' => 'app_admin_practical_queue', 'params' => [],
            ];
        }

        return $this->group('practical', $this->t('g_practical'), 'check', $rows, 'app_admin_practical_queue', [], $this->t('all_practical'));
    }

    /** Comptes dont le statut stocké est « pending » / « en attente ». */
    private function pendingAccounts(): ?array
    {
        $rows = [];
        foreach ($this->users->findBy(['statut' => ['pending', 'en attente']]) as $user) {
            $rows[] = [
                'title' => $user->getDisplayName(),
                'where' => $user->getEmail(),
                'when' => null, 'whenKind' => 'wall',
                'state' => ['label' => $this->t('st_pending'), 'signal' => 'wait'],
                'verb' => $this->t('verb_user'), 'route' => 'app_admin_user_detail', 'params' => ['id' => $user->getId()],
            ];
        }

        return $this->group('accounts', $this->t('g_accounts'), 'key', $rows, 'app_admin_users', ['statut' => 'pending'], $this->t('all_accounts'));
    }

    /** S210 — les retours des usagers encore ouverts (fonction allumée et migration passée). */
    private function userFeedback(): ?array
    {
        if (!$this->features->allowsSurface('feedback') || !$this->feedback->isReady()) {
            return null;
        }
        $signals = ['bug' => 'stop', 'ux' => 'caution', 'idea' => 'go'];
        $rows = [];
        foreach ($this->feedback->list('open') as $row) {
            $rows[] = [
                'title' => mb_strimwidth(preg_replace('/\s+/', ' ', (string) $row['message']) ?? '', 0, 90, '…'),
                'where' => $row['authorName'] ?: (string) $row['authorEmail'],
                'when' => new \DateTimeImmutable((string) $row['createdAt'], new \DateTimeZone('UTC')), 'whenKind' => 'utc',
                'state' => ['label' => $this->translator->trans('feedback.kind_' . $row['kind']), 'signal' => $signals[$row['kind']] ?? 'muted'],
                'verb' => $this->t('verb_feedback'), 'route' => 'app_admin_feedback', 'params' => [],
            ];
        }

        return $this->group('feedback', $this->t('g_feedback'), 'mail', $rows, 'app_admin_feedback', [], $this->t('all_feedback'));
    }

    /** S206 — matériaux épuisés ou sous leur seuil. Sans ligne (donc sans groupe) tant que la fonction est éteinte. */
    private function lowStock(): ?array
    {
        $rows = [];
        foreach ($this->stock->lowMaterials() as $material) {
            $out = $material['state'] === MaterialStock::OUT;
            $rows[] = [
                'title' => $material['name'],
                'where' => $this->stock->format($material['quantity'], $material['unit']),
                'when' => null, 'whenKind' => 'wall',
                'state' => ['label' => $this->t($out ? 'st_stock_out' : 'st_stock_low'), 'signal' => $out ? 'stop' : 'caution'],
                'verb' => $this->t('verb_stock'), 'route' => 'app_admin_material_edit', 'params' => ['id' => $material['id']],
            ];
        }

        return $this->group('stock', $this->t('g_stock'), 'box', $rows, 'app_admin_materials', [], $this->t('all_stock'));
    }
}
