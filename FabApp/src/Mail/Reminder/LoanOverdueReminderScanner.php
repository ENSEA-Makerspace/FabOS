<?php

namespace App\Mail\Reminder;

use App\Mail\ReminderSettings;

/**
 * "You were supposed to bring that back last week."
 *
 * Sent once, when a loan first goes past its date — not daily. A borrower who
 * is three weeks late already knows; nagging them every morning only teaches
 * people to filter the lab's mail, which costs more than the drill.
 */
final class LoanOverdueReminderScanner extends AbstractLoanReminderScanner
{
    public function kind(): string
    {
        return ReminderSettings::LOAN_OVERDUE;
    }

    public function scan(\DateTimeImmutable $now): array
    {
        if (!$this->loansEnabled()) {
            return [];
        }

        $today = $this->today($now);
        // Everything dated strictly before today, with no lower bound: a loan
        // that went overdue while reminders were switched off is still overdue.
        $yesterday = $today->modify('-1 day');

        $candidates = [];
        foreach ($this->loans->findOutWithReturnBetween(null, $yesterday) as $loan) {
            $candidate = $this->candidate($loan, 'loan_overdue', 'reminder_loan_overdue', $today);
            if ($candidate !== null) {
                $candidates[] = $candidate;
            }
        }

        return $candidates;
    }
}
