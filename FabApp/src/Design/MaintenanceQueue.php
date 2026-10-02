<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\MaintenanceTask;
use App\Repository\MaintenanceTaskRepository;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * La file d'intervention de la proposition `maintenance` (2026-10-01, d'après la
 * planche `productwide/04-maintenance.png`).
 *
 * ⚠️ LECTURE SEULE, et aucune logique neuve : la source est
 * `MaintenanceTaskRepository::findAllSafe()` (celle de `/admin/maintenance`) et
 * l'état est `MaintenanceTask::getEffectiveStatus()` (`done` | `overdue` |
 * `due_soon` | `pending`). « Cette semaine » = `due_soon` (échéance dans les
 * 7 jours, comme le fait déjà l'accueil admin). Les tâches archivées sont écartées.
 *
 * Tuiles (clé `?tuile=`) : `todo` (ouvertes, par défaut), `overdue`, `week`, `done`.
 */
final class MaintenanceQueue
{
    /** Clés de catalogue des libellés de tuile. */
    private const TILES = [
        'todo' => 'state.todo_f',
        'overdue' => 'state.overdue',
        'week' => 'maintenance_queue.tile_week',
        'done' => 'state.done_m',
    ];

    public function __construct(
        private readonly MaintenanceTaskRepository $tasks,
        private readonly TranslatorInterface $translator,
    ) {
    }

    /** @return array{tiles: list<array<string, mixed>>, active: string, rows: list<array<string, mixed>>, empty: bool} */
    public function build(string $tile): array
    {
        $tasks = array_values(array_filter($this->tasks->findAllSafe(), static fn (MaintenanceTask $t): bool => !$t->isArchived()));
        $tile = isset(self::TILES[$tile]) ? $tile : 'todo';

        $counts = array_fill_keys(array_keys(self::TILES), 0);
        $rows = [];
        foreach ($tasks as $task) {
            $status = $task->getEffectiveStatus();
            $keys = match ($status) {
                'done' => ['done'],
                'overdue' => ['todo', 'overdue'],
                'due_soon' => ['todo', 'week'],
                default => ['todo'],
            };
            foreach ($keys as $key) {
                ++$counts[$key];
            }
            if (!\in_array($tile, $keys, true)) {
                continue;
            }
            $machine = $task->getMachine();
            $rows[] = [
                'id' => $task->getId(),
                'title' => $task->getTitle(),
                'type' => $this->translator->trans($task->getType() === 'corrective' ? 'maintenance.type_corrective' : 'maintenance.type_preventive'),
                'recurrenceDays' => $task->getRecurrenceDays(),
                'machine' => $machine === null ? null : ['id' => $machine->getId(), 'name' => $machine->getNom(), 'photo' => $machine->getPhoto()],
                'due' => $task->getDueDate(),
                'done' => $task->getDoneDate(),
                'state' => match ($status) {
                    'done' => ['label' => $this->translator->trans('state.done_m'), 'signal' => 'go'],
                    'overdue' => ['label' => $this->translator->trans('state.overdue'), 'signal' => 'stop'],
                    'due_soon' => ['label' => $this->translator->trans('state.soon'), 'signal' => 'caution'],
                    default => ['label' => $this->translator->trans('state.todo_f'), 'signal' => 'wait'],
                },
                'open' => $status !== 'done',
            ];
        }

        $tiles = [];
        foreach (self::TILES as $key => $label) {
            $tiles[] = ['key' => $key, 'label' => $this->translator->trans($label), 'count' => $counts[$key], 'active' => $key === $tile, 'signal' => $key === 'overdue' ? 'stop' : ($key === 'week' ? 'caution' : null)];
        }

        return ['tiles' => $tiles, 'active' => $tile, 'rows' => $rows, 'empty' => $rows === []];
    }
}
