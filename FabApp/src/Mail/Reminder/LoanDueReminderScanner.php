<?php

namespace App\Mail\Reminder;

use App\Mail\ReminderSettings;

/**
 * "The thing you borrowed is due back on Friday."
 *
 * The window runs from today to today-plus-lead, so a loan booked with a return
 * date already inside the window is caught on the next run instead of never.
 * Anything already past its date belongs to the overdue scanner.
 */
final class LoanDueReminderScanner extends AbstractLoanReminderScanner
{
    public function kind(): string
    {
        return ReminderSettings::LOAN_DUE;
    }

    public function scan(\DateTimeImmutable $now): array
    {
        $today = $this->today($now);
        $horizon = $today->modify(sprintf('+%d days', $this->settings->getLoanLeadDays()));

        $candidates = [];
        foreach ($this->loans->findOutWithReturnBetween($today, $horizon) as $loan) {
            $candidate = $this->candidate($loan, 'loan_due', 'reminder_loan_due', $today);
            if ($candidate !== null) {
                $candidates[] = $candidate;
            }
        }

        return $candidates;
    }
}
