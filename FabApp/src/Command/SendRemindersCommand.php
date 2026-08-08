<?php

namespace App\Command;

use App\Mail\Reminder\ReminderRunner;
use App\Mail\Reminder\ReminderRunReport;
use App\Mail\ReminderLog;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Input\InputOption;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * The scheduled half of FabOS mail: run this on a timer and it works out what
 * is coming due and tells the people it concerns.
 *
 * Safe to run as often as you like — every candidate is claimed exactly once in
 * MAIL_REMINDER, so the cadence only decides how promptly a reminder goes out,
 * never how many arrive. Hourly is the intended setting (see the systemd timer
 * shipped alongside it); the booking lead time is what actually controls when
 * people hear from us.
 *
 * Nothing is sent for a reminder kind the admin hasn't switched on at
 * /admin/emails, and nothing at all is sent while the sender account is
 * unconfigured — in that case the run is abandoned *before* claiming anything,
 * so today's reminders still go out once SMTP is fixed.
 */
#[AsCommand(
    name: 'app:mail:reminders',
    description: 'Envoie les rappels programmés (réservations, prêts, maintenance) qui sont dus.',
)]
final class SendRemindersCommand extends Command
{
    public function __construct(
        private readonly ReminderRunner $runner,
        private readonly ReminderLog $log,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this
            ->addOption('dry-run', null, InputOption::VALUE_NONE, 'Liste ce qui serait envoyé sans rien envoyer ni marquer comme envoyé.')
            ->addOption('prune', null, InputOption::VALUE_NONE, 'Purge aussi les rappels déjà envoyés il y a plus de 180 jours.');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $dryRun = (bool) $input->getOption('dry-run');

        $report = $this->runner->run($dryRun);

        if (!$report->isOperational()) {
            $io->warning('Aucun envoi : le module « emails » est désactivé ou aucun compte d\'envoi n\'est configuré. Rien n\'a été marqué comme envoyé.');

            return Command::SUCCESS;
        }

        $this->renderReport($io, $report, $dryRun);

        if ($input->getOption('prune') && !$dryRun) {
            $pruned = $this->log->prune();
            if ($pruned > 0) {
                $io->comment(sprintf('%d rappel(s) archivé(s) purgé(s).', $pruned));
            }
        }

        // A scanner that blew up is worth a non-zero exit, so a timer failure is
        // visible in `systemctl status` instead of only in the mail that isn't there.
        return $report->failures() === [] ? Command::SUCCESS : Command::FAILURE;
    }

    private function renderReport(SymfonyStyle $io, ReminderRunReport $report, bool $dryRun): void
    {
        if ($report->disabled() !== []) {
            $io->comment('Rappels désactivés dans l\'administration : ' . implode(', ', $report->disabled()) . '.');
        }

        $kinds = $report->kinds();
        if ($kinds !== []) {
            $rows = [];
            foreach ($kinds as $kind => $counts) {
                $rows[] = [
                    $kind,
                    $dryRun ? $counts['would_send'] : $counts['sent'],
                    $counts['skipped'],
                    $counts['suppressed'],
                ];
            }

            $io->table(['Rappel', $dryRun ? 'À envoyer' : 'Envoyés', 'Déjà envoyés', 'Non envoyés'], $rows);
        }

        if ($dryRun && $report->preview() !== []) {
            $io->section('Aperçu');
            foreach ($report->preview() as $row) {
                $io->writeln(sprintf('  <info>%s</info> → %s <comment>(%s)</comment>', $row['kind'], $row['recipient'], $row['key']));
            }
        }

        foreach ($report->failures() as $kind => $error) {
            $io->error(sprintf('Rappel « %s » en échec : %s', $kind, $error));
        }

        $total = $dryRun ? $report->totalWouldSend() : $report->totalSent();
        $summary = $dryRun
            ? sprintf('%d rappel(s) seraient envoyés (%d déjà envoyés).', $total, $report->totalSkipped())
            : sprintf('%d rappel(s) envoyés, %d déjà envoyés, %d non envoyés (désinscrits ou adresse invalide).', $total, $report->totalSkipped(), $report->totalSuppressed());

        $total > 0 ? $io->success($summary) : $io->text($summary);
    }
}
