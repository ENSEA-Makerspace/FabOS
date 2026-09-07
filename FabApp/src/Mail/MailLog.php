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

    private ?bool $hasRenderedFrom = null;

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

    /**
     * 🔴 **`$renderedFrom` répond à « pourquoi ce mail dit ça ? » (S162).** Sans
     * lui, un exploitant qui lit un message reçu ne peut pas savoir s'il regarde
     * le texte livré, le sien, ou le texte livré parce que le sien était cassé —
     * et c'est un critère de sortie de la phase.
     *
     * ⚠️ **La colonne peut ne pas exister encore, et l'écriture le tolère.** Le
     * code se déploie avant sa migration ; un `UPDATE` qui échouerait ici
     * laisserait un mail PARTI marqué « en file », c'est-à-dire renvoyé à chaque
     * reprise. La sonde de colonne suit le motif de `MailOverrides` : une fois
     * par processus, pas une fois par mail.
     */
    public function markSent(int $id, string $subject, ?string $renderedFrom = null): void
    {
        $extra = $renderedFrom !== null && $this->hasRenderedFrom() ? ', renderedFrom = :from' : '';
        $params = ['s' => self::STATUS_SENT, 'subject' => mb_substr($subject, 0, 255), 'id' => $id];
        if ($extra !== '') {
            $params['from'] = mb_substr($renderedFrom, 0, 64);
        }

        $this->db->executeStatement(
            'UPDATE EMAIL_LOG SET status = :s, subject = :subject, error = NULL, sentAt = NOW()' . $extra . ' WHERE id = :id',
            $params,
        );
    }

    /**
     * Les couples (gabarit, langue) dont le DERNIER envoi est retombé sur le
     * texte livré — pour que l'éditeur le dise là où on peut corriger (S162).
     *
     * ⚠️ **Dérivé du journal, pas d'un drapeau à tenir à jour.** Un « cassé »
     * stocké dans la table des surcharges serait un second état à remettre à
     * zéro quand quelqu'un répare son texte — et il se tromperait le jour où on
     * oublie.
     *
     * @return array<string, true> clés « gabarit|langue »
     */
    public function fallbackKeys(int $days = 30): array
    {
        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT DISTINCT template, locale FROM EMAIL_LOG
                 WHERE renderedFrom LIKE :pattern AND queuedAt >= :since',
                [
                    'pattern' => '%failed%',
                    // Calculé en PHP comme le reste du fichier : un `INTERVAL`
                    // paramétré ne se prépare pas partout de la même façon.
                    'since' => (new \DateTimeImmutable(sprintf('-%d days', $days)))->format('Y-m-d H:i:s'),
                ],
            );
        } catch (\Throwable) {
            return [];
        }

        $out = [];
        foreach ($rows as $row) {
            $out[$row['template'] . '|' . $row['locale']] = true;
        }

        return $out;
    }

    /** ⚠️ Sondée une fois par processus : un worker traite des centaines de mails. */
    private function hasRenderedFrom(): bool
    {
        if ($this->hasRenderedFrom !== null) {
            return $this->hasRenderedFrom;
        }

        try {
            $columns = $this->db->createSchemaManager()->listTableColumns('EMAIL_LOG');

            return $this->hasRenderedFrom = isset($columns['renderedfrom']);
        } catch (\Throwable) {
            return $this->hasRenderedFrom = false;
        }
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

    /**
     * The log, filtered — period, status and recipient (S132).
     *
     * ⚠️ **Server-side, and that is the point.** `recent(50)` was the whole
     * answer: the fiftieth-most-recent message was the horizon, so "did the
     * reminder reach Camille last Tuesday?" was unanswerable on any installation
     * that sends more than fifty mails a week, and a client-side filter over the
     * same fifty rows would have looked like an answer while being the same
     * horizon with a search box on it.
     *
     * `$days === 0` means no period bound.
     *
     * @return array<int, array<string, mixed>>
     */
    public function search(int $days = 30, ?string $status = null, string $recipient = '', int $limit = 200): array
    {
        [$where, $params] = $this->filters($days, $status, $recipient);

        try {
            return $this->db->fetchAllAssociative(
                'SELECT id, category, recipient, recipientName, locale, template, subject, status, error, queuedAt, sentAt
                 FROM EMAIL_LOG ' . $where . ' ORDER BY id DESC LIMIT ' . max(1, min($limit, 500)),
                $params,
            );
        } catch (\Throwable) {
            return [];
        }
    }

    /** How many rows the same filter matches, which is not the same as how many are shown. */
    public function countMatching(int $days = 30, ?string $status = null, string $recipient = ''): int
    {
        [$where, $params] = $this->filters($days, $status, $recipient);

        try {
            return (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG ' . $where, $params);
        } catch (\Throwable) {
            return 0;
        }
    }

    /**
     * ⚠️ `queuedAt` is written with `NOW()`, so it is a **machine timestamp in the
     * server's zone** and the bound is computed in that same zone. Reading the
     * lab's configured timezone here would shift the window by the offset and
     * silently drop or add a day's worth of rows.
     *
     * @return array{0: string, 1: array<string, mixed>}
     */
    private function filters(int $days, ?string $status, string $recipient): array
    {
        $clauses = [];
        $params = [];

        if ($days > 0) {
            $clauses[] = 'queuedAt >= :since';
            $params['since'] = (new \DateTimeImmutable(sprintf('-%d days', $days)))->format('Y-m-d H:i:s');
        }
        if (in_array($status, [self::STATUS_QUEUED, self::STATUS_SENT, self::STATUS_FAILED], true)) {
            $clauses[] = 'status = :status';
            $params['status'] = $status;
        }
        $recipient = trim($recipient);
        if ($recipient !== '') {
            $clauses[] = '(recipient LIKE :recipient OR recipientName LIKE :recipient)';
            // Escaped: an operator pasting an address with an underscore in it
            // would otherwise get every address with any character in that spot.
            $params['recipient'] = '%' . addcslashes($recipient, '%_\\') . '%';
        }

        return [$clauses === [] ? '' : 'WHERE ' . implode(' AND ', $clauses), $params];
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
