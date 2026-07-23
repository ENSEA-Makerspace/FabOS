<?php

namespace App\Mail;

use App\Mail\Message\SendMailMessage;
use Symfony\Component\Messenger\Attribute\AsMessageHandler;

/**
 * Consumes the async queue. Errors propagate on purpose: the transport failing is
 * exactly the case Messenger's retry strategy exists for, and the log row has
 * already been marked failed with the reason by MailSender.
 */
#[AsMessageHandler]
final class SendMailHandler
{
    public function __construct(private readonly MailSender $sender)
    {
    }

    public function __invoke(SendMailMessage $message): void
    {
        $this->sender->send($message->logId);
    }
}
