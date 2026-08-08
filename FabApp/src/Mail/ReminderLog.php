<?php

namespace App\Mail;

use Doctrine\DBAL\Connection;

/**
 * Remembers which reminders have already been sent.
 *
 * The scanners are deliberately dumb: they re-ask "what is due soon?" on every
 * run and hand every match to this. Nothing else keeps an hourly timer from
 * mailing the same person about the same booking twenty times, so the guard has
 * to be airtight — hence a unique index and an INSERT that is allowed to
 * collide, rather than a SELECT-then-INSERT that two workers could both pass.
 *
 * Raw DBAL and fail-safe like the other mail-adjacent stores, with one
 * deliberate asymmetry: a claim that cannot be recorded returns *false*. If the
 * ledger is unavailable we decline to send, because the failure mode of
 * forgetting is a missing reminder, and the failure mode of guessing is a
 * mail loop.
 */
final class ReminderLog
{
    public function __construct(private readonly Connection $db)
    {
    }

    /**
     * Claims the right to send one reminder, exactly once.
     *
     * @param string $key a caller-built identity for this reminder, e.g. "booking:412:start"
     *
     * @return bool true when this call is the one that gets to send
     */
    public function claim(string $key, string $kind): bool
    {
        try {
            // INSERT IGNORE turns the unique-key collision into "someone got here
            // first" (0 rows) instead of an exception, which is exactly the answer.
            $inserted = $this->db->executeStatement(
                'INSERT IGNORE INTO MAIL_REMINDER (reminderKey, kind, sentAt) VALUES (:key, :kind, NOW())',
                ['key' => mb_substr($key, 0, 190), 'kind' => mb_substr($kind, 0, 50)],
            );
        } catch (\Throwable) {
            return false;
        }

        return $inserted > 0;
    }

    /** Whether a reminder has already gone out, without claiming it — for --dry-run. */
    public function isClaimed(string $key): bool
    {
        try {
            return $this->db->fetchOne('SELECT 1 FROM MAIL_REMINDER WHERE reminderKey = :key', ['key' => mb_substr($key, 0, 190)]) !== false;
        } catch (\Throwable) {
            // Unknown means "assume unsent" here: a dry run sends nothing either way,
            // and over-reporting what *would* go out is the safer way to be wrong.
            return false;
        }
    }

    /**
     * Drops claims older than the retention window. The ledger only has to
     * outlive the thing it describes — a booking that started last year cannot
     * come due again — so it is pruned rather than kept forever.
     */
    public function prune(int $keepDays = 180): int
    {
        try {
            return $this->db->executeStatement(
                'DELETE FROM MAIL_REMINDER WHERE sentAt < DATE_SUB(NOW(), INTERVAL :days DAY)',
                ['days' => max(1, $keepDays)],
            );
        } catch (\Throwable) {
            return 0;
        }
    }

    /** @return array<string, int> kind => count, for the admin screen. */
    public function countsByKind(): array
    {
        try {
            $rows = $this->db->fetchAllAssociative('SELECT kind, COUNT(*) AS total FROM MAIL_REMINDER GROUP BY kind');
        } catch (\Throwable) {
            return [];
        }

        $counts = [];
        foreach ($rows as $row) {
            $counts[(string) $row['kind']] = (int) $row['total'];
        }

        return $counts;
    }
}
