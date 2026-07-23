<?php

namespace App\Mail\Reminder;

use Symfony\Component\DependencyInjection\Attribute\AutoconfigureTag;

/**
 * One kind of scheduled reminder.
 *
 * Scanners are stateless and deliberately forgetful: each run re-asks "what is
 * due right now?" from scratch and returns everything it finds, including
 * things it already reported last hour. Deciding what has already gone out is
 * ReminderLog's job, not theirs — which is what makes the timer's cadence a
 * tuning decision rather than a correctness one.
 *
 * Implementations are auto-registered through the tag below, so adding a
 * reminder is one new class and nothing else.
 */
#[AutoconfigureTag('app.reminder_scanner')]
interface ReminderScanner
{
    /** The ReminderSettings kind this serves — its admin toggle and its log key. */
    public function kind(): string;

    /**
     * Everything that looks due as of $now, duplicates and all.
     *
     * @return ReminderCandidate[]
     */
    public function scan(\DateTimeImmutable $now): array;
}
