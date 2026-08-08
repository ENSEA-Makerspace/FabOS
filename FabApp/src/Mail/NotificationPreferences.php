<?php

namespace App\Mail;

use Doctrine\DBAL\Connection;

/**
 * Which categories a given person still wants to hear about.
 *
 * Stored as opt-out rows: no row means yes. That keeps the table proportional
 * to the number of people who actually changed something, and lets a new
 * category go live without writing a row for every existing account.
 *
 * **Reads fail open** — an unreadable table means "send it". That is the
 * uncomfortable direction, and it is chosen deliberately: ReminderRunner claims
 * a reminder in MAIL_REMINDER *before* asking whether to send it, so a
 * preference lookup that failed closed would consume the claim and drop that
 * reminder permanently, silently, with no retry. A transient database hiccup
 * costing someone one unwanted mail is recoverable; costing them a booking
 * reminder they never learn was suppressed is not. The master
 * `notificationEmail` switch on Utilisateur still applies either way, and it is
 * read from the already-loaded entity, so it cannot fail this way.
 */
final class NotificationPreferences
{
    public function __construct(private readonly Connection $db)
    {
    }

    /** Whether this person still accepts mail in this category. */
    public function accepts(int $userId, string $category): bool
    {
        if (!NotificationCategory::isOptOutable($category)) {
            return true;
        }

        try {
            $optedOut = $this->db->fetchOne(
                'SELECT 1 FROM USER_NOTIFICATION_OPTOUT WHERE userId = :user AND category = :category',
                ['user' => $userId, 'category' => $category],
            );
        } catch (\Throwable) {
            return true;
        }

        return $optedOut === false;
    }

    /**
     * Every opt-out-able category with its current state, for the preference
     * screen. Categories the person has never touched come back as true.
     *
     * @return array<string, bool> category => accepted
     */
    public function forUser(int $userId): array
    {
        $state = array_fill_keys(NotificationCategory::OPTOUTABLE, true);

        try {
            $rows = $this->db->fetchFirstColumn('SELECT category FROM USER_NOTIFICATION_OPTOUT WHERE userId = :user', ['user' => $userId]);
        } catch (\Throwable) {
            return $state;
        }

        foreach ($rows as $category) {
            if (array_key_exists((string) $category, $state)) {
                $state[(string) $category] = false;
            }
        }

        return $state;
    }

    public function optOut(int $userId, string $category): void
    {
        if (!NotificationCategory::isOptOutable($category)) {
            return;
        }

        try {
            $this->db->executeStatement(
                'INSERT IGNORE INTO USER_NOTIFICATION_OPTOUT (userId, category, optedOutAt) VALUES (:user, :category, NOW())',
                ['user' => $userId, 'category' => $category],
            );
        } catch (\Throwable) {
            // Nothing useful to do here; the caller reports success or failure
            // from its own re-read, not from this.
        }
    }

    public function optIn(int $userId, string $category): void
    {
        try {
            $this->db->executeStatement(
                'DELETE FROM USER_NOTIFICATION_OPTOUT WHERE userId = :user AND category = :category',
                ['user' => $userId, 'category' => $category],
            );
        } catch (\Throwable) {
        }
    }

    /**
     * Applies a whole preference screen at once.
     *
     * @param array<string, bool> $accepted category => accepted
     */
    public function save(int $userId, array $accepted): void
    {
        foreach (NotificationCategory::OPTOUTABLE as $category) {
            empty($accepted[$category])
                ? $this->optOut($userId, $category)
                : $this->optIn($userId, $category);
        }
    }
}
