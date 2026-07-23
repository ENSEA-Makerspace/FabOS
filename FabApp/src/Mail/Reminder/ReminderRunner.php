<?php

namespace App\Mail\Reminder;

use App\Mail\Mailer;
use App\Mail\ReminderLog;
use App\Mail\ReminderSettings;
use Symfony\Component\DependencyInjection\Attribute\AutowireIterator;

/**
 * Turns "what is due?" into "what has been sent?".
 *
 * The whole run is guarded by two questions asked once, up front: is mail
 * working at all, and is this kind of reminder switched on. Both matter because
 * claiming is irreversible — burning a claim for a mail that was never going to
 * be delivered means the admin fixes their SMTP account and then wonders why
 * yesterday's reminders never arrived.
 *
 * Order within a candidate is deliberate: claim first, then queue. The reverse
 * risks sending twice if the process dies between the two, and a duplicate
 * reminder is worse than a missing one.
 */
final class ReminderRunner
{
    /** @param iterable<ReminderScanner> $scanners */
    public function __construct(
        #[AutowireIterator('app.reminder_scanner')]
        private readonly iterable $scanners,
        private readonly ReminderSettings $settings,
        private readonly ReminderLog $log,
        private readonly Mailer $mailer,
    ) {
    }

    /**
     * @return ReminderRunReport what was found, sent and skipped, per kind
     */
    public function run(bool $dryRun = false, ?\DateTimeImmutable $now = null): ReminderRunReport
    {
        $now ??= new \DateTimeImmutable();
        $report = new ReminderRunReport($dryRun);

        if (!$this->mailer->isOperational()) {
            $report->markNotOperational();

            return $report;
        }

        foreach ($this->scanners as $scanner) {
            $kind = $scanner->kind();

            if (!$this->settings->isEnabled($kind)) {
                $report->markDisabled($kind);

                continue;
            }

            try {
                $candidates = $scanner->scan($now);
            } catch (\Throwable $e) {
                // One broken scanner must not stop the others: a missing loans
                // table shouldn't cost the lab its booking reminders.
                $report->markFailed($kind, $e->getMessage());

                continue;
            }

            foreach ($candidates as $candidate) {
                $this->handle($report, $kind, $candidate, $dryRun);
            }
        }

        return $report;
    }

    private function handle(ReminderRunReport $report, string $kind, ReminderCandidate $candidate, bool $dryRun): void
    {
        if ($dryRun) {
            if ($this->log->isClaimed($candidate->key)) {
                $report->recordSkipped($kind);
            } else {
                $report->recordWouldSend($kind, $candidate);
            }

            return;
        }

        if (!$this->log->claim($candidate->key, $kind)) {
            $report->recordSkipped($kind);

            return;
        }

        try {
            $sent = $candidate->user !== null
                // Reminders are never transactional — a user who turned off
                // notifications has opted out of exactly this.
                ? $this->mailer->queueToUser($candidate->user, $candidate->template, $candidate->context, 'reminder', false)
                : $this->mailer->queue((string) $candidate->email, $candidate->name, $candidate->template, $candidate->context, 'reminder');
        } catch (\Throwable $e) {
            $report->markFailed($kind, $e->getMessage());

            return;
        }

        // A refused queue (opted out, bad address) still counts as handled: the
        // claim stands, because re-offering it every hour would change nothing.
        $sent ? $report->recordSent($kind, $candidate) : $report->recordSuppressed($kind);
    }
}
