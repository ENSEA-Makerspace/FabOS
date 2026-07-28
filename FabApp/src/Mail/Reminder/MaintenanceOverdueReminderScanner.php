<?php

namespace App\Mail\Reminder;

use App\Mail\ReminderSettings;
use App\Repository\MaintenanceTaskRepository;
use App\Repository\UtilisateurRepository;
use App\Service\ModuleService;

/**
 * "The laser cutter's filter change was due a week ago."
 *
 * The only reminder here that isn't addressed to a specific person: maintenance
 * belongs to whoever is on staff, so every overdue task is announced to all of
 * them. The claim key therefore carries the recipient as well as the task —
 * a staff member who joins next month should still hear about a task that was
 * already announced to everyone else, and nobody should hear about it twice.
 *
 * One mail per task rather than a daily digest, and only ever one: a task that
 * stays overdue for a month generates a single mail, not thirty.
 */
final class MaintenanceOverdueReminderScanner implements ReminderScanner
{
    public function __construct(
        private readonly MaintenanceTaskRepository $tasks,
        private readonly UtilisateurRepository $people,
        private readonly ModuleService $modules,
    ) {
    }

    public function kind(): string
    {
        return ReminderSettings::MAINTENANCE_OVERDUE;
    }

    public function scan(\DateTimeImmutable $now): array
    {
        // Same rule as the loan and event scanners: a disabled module goes
        // quiet everywhere, background timer included.
        if (!$this->modules->isEnabled('maintenance')) {
            return [];
        }

        $today = $now->setTime(0, 0);
        $overdue = $this->tasks->findOverdue($today);
        if ($overdue === []) {
            return [];
        }

        $staff = $this->people->findStaff();
        if ($staff === []) {
            return [];
        }

        $candidates = [];
        foreach ($overdue as $task) {
            $taskId = $task->getId();
            $due = $task->getDueDate();
            if ($taskId === null || $due === null) {
                continue;
            }

            $context = [
                'task' => $task->getTitle(),
                'machine' => $task->getMachine()?->getName() ?? '',
                'due' => $due->format(\DATE_ATOM),
                'days_late' => max(0, (int) $today->diff($due)->format('%r%a') * -1),
                'type' => $task->getType(),
                'link' => $task->getLink(),
            ];

            foreach ($staff as $member) {
                $memberId = $member->getId();
                if ($memberId === null) {
                    continue;
                }

                $candidates[] = ReminderCandidate::forUser(
                    sprintf('maintenance_overdue:%d:%d', $taskId, $memberId),
                    $member,
                    'reminder_maintenance_overdue',
                    $context,
                );
            }
        }

        return $candidates;
    }
}
