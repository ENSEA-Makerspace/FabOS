<?php

declare(strict_types=1);

namespace App\Design;

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
use App\Training\PracticalQueue;

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
    ) {
    }

    /** @return array{total: int, groups: list<array<string, mixed>>, stats: array<string, int>} */
    public function build(): array
    {
        $groups = array_values(array_filter([
            $this->unavailableMachines(),
            $this->maintenanceDue(),
            $this->offlineReaders(),
            $this->refusedAccess(),
            $this->pendingReservations(),
            $this->overdueLoans(),
            $this->practicalValidations(),
            $this->pendingAccounts(),
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
                    ? ['label' => 'En panne', 'signal' => 'stop']
                    : ['label' => 'En maintenance', 'signal' => 'caution'],
                'verb' => 'Voir la machine', 'route' => 'app_machine_detail', 'params' => ['id' => $machine->getId()],
            ];
        }

        return $this->group('machines', 'Machines indisponibles', 'tool', $rows, 'app_admin_machines', [], 'Voir toutes les machines');
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
                    ? ['label' => 'En retard', 'signal' => 'stop']
                    : ['label' => 'Bientôt due', 'signal' => 'caution'],
                'verb' => 'Ouvrir la tâche', 'route' => 'app_admin_maintenance_edit', 'params' => ['id' => $task->getId()],
                '_sort' => $task->getDueDate()?->getTimestamp() ?? 0,
            ];
        }
        usort($rows, static fn (array $a, array $b): int => $a['_sort'] <=> $b['_sort']);

        return $this->group('maintenance', 'Maintenance à faire', 'history', $rows, 'app_admin_maintenance', [], 'Voir toute la maintenance');
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
                'when' => $reader->getLastSeenAt(), 'whenKind' => 'utc', 'whenPrefix' => 'Dernier contact ',
                'state' => ['label' => 'Hors ligne', 'signal' => 'stop'],
                'verb' => 'Ouvrir le lecteur', 'route' => 'app_admin_rfid_reader_edit', 'params' => ['id' => $reader->getId()],
            ];
        }

        return $this->group('readers', 'Lecteurs hors ligne', 'bolt', $rows, 'app_admin_rfid_readers', [], 'Voir tous les lecteurs');
    }

    /** Journal RFID : refusés des 7 derniers jours, avec le verbe correctif d'AccessIncident. */
    private function refusedAccess(): ?array
    {
        $rows = [];
        foreach ($this->rfidLogs->search(7, null, null, 'no', 50) as $log) {
            /** @var AccessRfidLog $log */
            $fix = $this->incidents->of($log);
            $rows[] = [
                'title' => $log->getUtilisateur()?->getDisplayName() ?? 'Badge inconnu',
                'where' => $log->getMachine()?->getNom() ?? $log->getReader()?->getName() ?? '',
                'when' => $log->getCreatedAt(), 'whenKind' => 'utc',
                'state' => ['label' => 'Refusé', 'signal' => 'stop'],
                'status' => $log->getStatus(),
                'fix' => $fix,
                'verb' => 'Voir le journal', 'route' => 'app_admin_access_rfid_logs', 'params' => ['days' => 7, 'result' => 'no'],
            ];
        }

        return $this->group('access', 'Refus d’accès récents (7 jours)', 'forbidden', $rows, 'app_admin_access_rfid_logs', ['days' => 7, 'result' => 'no'], 'Voir le journal des refus');
    }

    /** Réservations au statut « pending ». */
    private function pendingReservations(): ?array
    {
        $rows = [];
        foreach ($this->reservations->findForAdminFilters(['statut' => Reservation::STATUS_PENDING]) as $reservation) {
            $rows[] = [
                'title' => $reservation->getReservableLabel() ?: 'Réservation',
                'where' => $reservation->getUtilisateur()?->getDisplayName() ?? '',
                'when' => $reservation->getDateDebut(), 'whenKind' => 'wall',
                'state' => ['label' => 'À valider', 'signal' => 'wait'],
                'verb' => 'Examiner', 'route' => 'app_reservation_detail', 'params' => ['id' => $reservation->getId()],
            ];
        }

        return $this->group('reservations', 'Réservations à valider', 'calendar', $rows, 'app_admin_reservations', ['statut' => Reservation::STATUS_PENDING], 'Voir les réservations');
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
                'title' => $loan->getItem()?->getName() ?? 'Objet prêté',
                'where' => $loan->getBorrowerDisplay(),
                'when' => $loan->getExpectedReturnDate(), 'whenKind' => 'wall', 'whenPrefix' => 'À rendre le ',
                'state' => ['label' => 'En retard', 'signal' => 'stop'],
                'verb' => 'Voir le prêt', 'route' => 'app_admin_loans', 'params' => [],
            ];
        }

        return $this->group('loans', 'Prêts en retard', 'box', $rows, 'app_admin_loans', [], 'Voir tous les prêts');
    }

    /** PracticalQueue : théorie finie, pratique à valider. */
    private function practicalValidations(): ?array
    {
        $rows = [];
        foreach ($this->practicalQueue->pending() as $row) {
            $rows[] = [
                'title' => $row['user']->getDisplayName(),
                'where' => (string) $row['formation']->getTitre(),
                'when' => $row['since'], 'whenKind' => 'wall', 'whenPrefix' => 'Depuis le ',
                'state' => ['label' => 'Pratique à valider', 'signal' => 'wait'],
                'verb' => 'Ouvrir la file', 'route' => 'app_admin_practical_queue', 'params' => [],
            ];
        }

        return $this->group('practical', 'Validations pratiques en attente', 'check', $rows, 'app_admin_practical_queue', [], 'Voir la file');
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
                'state' => ['label' => 'En attente', 'signal' => 'wait'],
                'verb' => 'Ouvrir la fiche', 'route' => 'app_admin_user_detail', 'params' => ['id' => $user->getId()],
            ];
        }

        return $this->group('accounts', 'Comptes en attente', 'key', $rows, 'app_admin_users', ['statut' => 'pending'], 'Voir les comptes');
    }
}
