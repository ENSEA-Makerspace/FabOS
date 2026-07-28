<?php

namespace App\Mail\Reminder;

use App\Entity\Loan;
use App\Mail\ReminderSettings;
use App\Repository\LoanRepository;
use App\Service\ModuleService;

/**
 * Shared ground for the two loan reminders: both sweep LOAN rows that are still
 * out and dated, and both have to cope with a borrower who may or may not have
 * an account — the loans module records walk-in borrowers as a name and an
 * address, and those people still deserve to hear that their kit is due back.
 */
abstract class AbstractLoanReminderScanner implements ReminderScanner
{
    public function __construct(
        protected readonly LoanRepository $loans,
        protected readonly ReminderSettings $settings,
        protected readonly ModuleService $modules,
    ) {
    }

    /**
     * A lab that has switched the loans module off must stop hearing about
     * loans — including from the background timer. Without this, disabling a
     * feature silently leaves it mailing people, which is the worst kind of
     * "off": invisible in the UI and still reaching inboxes.
     */
    protected function loansEnabled(): bool
    {
        return $this->modules->isEnabled('loans');
    }

    /** Builds the candidate for one loan, or null when there is nobody to tell. */
    protected function candidate(Loan $loan, string $keyPrefix, string $template, \DateTimeImmutable $today): ?ReminderCandidate
    {
        $id = $loan->getId();
        $due = $loan->getExpectedReturnDate();
        if ($id === null || $due === null) {
            return null;
        }

        $key = sprintf('%s:%d', $keyPrefix, $id);
        $context = [
            'item' => $loan->getItem()?->getName() ?? '',
            'due' => $due->format(\DATE_ATOM),
            'taken' => $loan->getDateTaken()->format(\DATE_ATOM),
            // Whole days late, computed here rather than in Twig: the template is
            // rendered later by the worker, when "today" is no longer this today.
            'days_late' => max(0, (int) $today->diff($due)->format('%r%a') * -1),
            'borrower' => $this->borrowerName($loan),
        ];

        $borrower = $loan->getBorrower();
        if ($borrower !== null) {
            return ReminderCandidate::forUser($key, $borrower, $template, $context);
        }

        $email = trim((string) $loan->getBorrowerEmail());

        return $email === ''
            ? null
            : ReminderCandidate::forAddress($key, $email, $loan->getBorrowerName(), $template, $context);
    }

    private function borrowerName(Loan $loan): string
    {
        $borrower = $loan->getBorrower();
        if ($borrower !== null) {
            return $borrower->getDisplayName() ?: $borrower->getEmail();
        }

        return trim((string) $loan->getBorrowerName());
    }

    /** Midnight today — loan dates are date-only, so the comparisons must be too. */
    protected function today(\DateTimeImmutable $now): \DateTimeImmutable
    {
        return $now->setTime(0, 0);
    }

}
