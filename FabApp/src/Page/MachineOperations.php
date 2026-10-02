<?php

declare(strict_types=1);

namespace App\Page;

use App\Entity\Machine;
use App\Entity\MaintenanceTask;
use App\Entity\RfidReader;
use App\Repository\AccessRfidLogRepository;
use App\Repository\LogUtilisationRepository;
use App\Repository\MachineRepository;
use App\Repository\MaintenanceTaskRepository;
use App\Repository\RfidReaderRepository;
use App\Rfid\AccessIncident;
use App\Rfid\ReaderHealth;
use App\Feature\SiteFeatureService;

/**
 * « Exploitation » d'une machine (proposition du 2026-10-01, planche
 * `equipment-machine-operations.png`).
 *
 * ⚠️ LECTURE SEULE et rien de neuf : l'état vient de `Machine::getStatusKey()`,
 * la maintenance de `MaintenanceTaskRepository::findOpenForMachine()` (la requête
 * de la fiche), l'état du lecteur de `ReaderHealth::of()`, les refus de
 * `AccessRfidLogRepository::search()` et leur verbe de `AccessIncident::of()`.
 * Aucune règle n'est réécrite ici.
 */
final class MachineOperations
{
    public function __construct(
        private readonly MachineRepository $machines,
        private readonly RfidReaderRepository $readers,
        private readonly MaintenanceTaskRepository $maintenance,
        private readonly AccessRfidLogRepository $rfidLogs,
        private readonly LogUtilisationRepository $usageLogs,
        private readonly ReaderHealth $readerHealth,
        private readonly AccessIncident $incidents,
        private readonly SiteFeatureService $modules,
    ) {
    }

    /**
     * @param int|null $machineId défaut : la première machine qui a un lecteur
     *                            associé (actif ou non), sinon la première machine.
     *
     * @return array<string, mixed>|null `null` quand il n'existe aucune machine.
     */
    public function for(?int $machineId): ?array
    {
        $machine = $machineId !== null ? $this->machines->find($machineId) : null;
        $machine ??= $this->firstWithReader() ?? ($this->machines->findBy([], ['nom' => 'ASC'], 1)[0] ?? null);
        if (!$machine instanceof Machine) {
            return null;
        }

        $statusKey = $machine->getStatusKey();
        $maintenanceEnabled = $this->modules->isEnabled('maintenance');
        $open = $maintenanceEnabled ? $this->maintenance->findOpenForMachine($machine) : [];
        $reader = $this->readers->findOneBy(['machine' => $machine], ['createdAt' => 'DESC']);

        $incidents = [];
        foreach ($this->rfidLogs->search(30, null, (int) $machine->getId(), 'no', 3) as $log) {
            $incidents[] = ['log' => $log, 'fix' => $this->incidents->of($log)];
        }

        return [
            'machine' => $machine,
            'state' => [
                'label' => $statusKey,
                'signal' => match ($statusKey) {
                    'machines.st_maintenance' => 'caution',
                    'machines.st_broken' => 'stop',
                    default => 'go',
                },
                // Hors service = « en maintenance » ou « en panne » : le verbe est
                // l'inverse de l'état, et mène à l'édition (c'est elle qui l'écrit).
                'outOfService' => $statusKey !== 'machines.st_available',
            ],
            'maintenanceEnabled' => $maintenanceEnabled,
            'task' => $open[0] ?? null,
            'openCount' => \count($open),
            'reader' => $reader instanceof RfidReader ? ['entity' => $reader, 'health' => $this->readerHealth->of($reader)] : null,
            'incidents' => $incidents,
            'counts' => [
                'rfid' => $this->rfidLogs->count(['machine' => $machine]),
                'usage' => $this->usageLogs->count(['machine' => $machine]),
            ],
        ];
    }

    private function firstWithReader(): ?Machine
    {
        foreach ($this->readers->findForAdmin() as $reader) {
            if (!$reader->isArchived() && $reader->getMachine() !== null) {
                return $reader->getMachine();
            }
        }

        return null;
    }
}
