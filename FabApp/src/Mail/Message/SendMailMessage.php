<?php

namespace App\Mail\Message;

/**
 * Async envelope for one logged mail. Carries only the EMAIL_LOG id — the row
 * holds the recipient, template, context and locale, so the queue payload stays
 * tiny and a retry always re-reads the current state of the log.
 */
final readonly class SendMailMessage
{
    public function __construct(public int $logId)
    {
    }
}
