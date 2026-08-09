<?php

namespace App\Mail;

use Doctrine\DBAL\Connection;

/**
 * The audit trail for outgoing mail: every message is written here *before* it is
 * queued, and the row is the queue payload — the async message carries only its id.
 * That keeps the queue small, survives a worker restart mid-send, and leaves an
 * admin-visible record of what went out (and what failed, with the error).
 *
 * Raw DBAL and fail-safe on reads, like the other config-adjacent stores: a missing
 * table degrades to an empty log rather than taking the admin page down.
 */
final class MailLog
{
    public const STATUS_QUEUED = 'queued';
    public const STATUS_SENT = 'sent';
    public const STATUS_FAILED = 'failed';

    public function __construct(
        private readonly Connection $db,
    ) {
    }

    /**
     * @param array<string, mixed> $context
     * @param int|null             $userId the account this is for, when there is
     *                                     one — it is what lets the renderer
     *                                     build a working unsubscribe link
     */
    public function queue(string $recipient, ?string $recipientName, string $template, array $context, string $locale, string $category, ?int $userId = null): int
    {
        $this->db->executeStatement(
            'INSERT INTO EMAIL_LOG (category, recipient, recipientName, locale, template, contextJson, status, queuedAt, userId)
             VALUES (:category, :recipient, :name, :locale, :template, :context, :status, NOW(), :userId)',
            [
                'category' => $category,
                'recipient' => $recipient,
                'name' => $recipientName,
                'locale' => $locale,
                'template' => $template,
                'context' => json_encode($context, JSON_THROW_ON_ERROR),
                'status' => self::STATUS_QUEUED,
                'userId' => $userId,
            ],
        );

        return (int) $this->db->lastInsertId();
    }

    /** @return array<string, mixed>|null */
    public function find(int $id): ?array
    {
        $row = $this->db->fetchAssociative('SELECT * FROM EMAIL_LOG WHERE id = :id', ['id' => $id]);

        return is_array($row) ? $row : null;
    }

    public function markSent(int $id, string $subject): void
    {
        $this->db->executeStatement(
            'UPDATE EMAIL_LOG SET status = :s, subject = :subject, error = NULL, sentAt = NOW() WHERE id = :id',
            ['s' => self::STATUS_SENT, 'subject' => mb_substr($subject, 0, 255), 'id' => $id],
        );
    }

    public function markFailed(int $id, string $error): void
    {
        $this->db->executeStatement(
            'UPDATE EMAIL_LOG SET status = :s, error = :error WHERE id = :id',
            ['s' => self::STATUS_FAILED, 'error' => mb_substr($error, 0, 500), 'id' => $id],
        );
    }

    /** @return array<int, array<string, mixed>> */
    public function recent(int $limit = 50): array
    {
        try {
            return $this->db->fetchAllAssociative(
                'SELECT id, category, recipient, recipientName, locale, template, subject, status, error, queuedAt, sentAt
                 FROM EMAIL_LOG ORDER BY id DESC LIMIT ' . max(1, min($limit, 200)),
            );
        } catch (\Throwable) {
            return [];
        }
    }

    /** @return array<string, int> status => count */
    public function statusCounts(): array
    {
        try {
            $rows = $this->db->fetchAllAssociative('SELECT status, COUNT(*) AS total FROM EMAIL_LOG GROUP BY status');
        } catch (\Throwable) {
            return [];
        }

        $counts = [];
        foreach ($rows as $row) {
            $counts[(string) $row['status']] = (int) $row['total'];
        }

        return $counts;
    }

    /**
     * Messages still sitting in the Messenger queue. Shown in the admin because a
     * backlog that never drains is the signature of a worker that isn't running —
     * otherwise a stuck queue looks exactly like "nothing was ever sent".
     */
    public function pendingQueueSize(): int
    {
        try {
            return (int) $this->db->fetchOne('SELECT COUNT(*) FROM messenger_messages WHERE delivered_at IS NULL');
        } catch (\Throwable) {
            return 0;
        }
    }
}
