<?php

namespace App\Mail;

use Symfony\Component\Mailer\Transport;
use Symfony\Component\Mime\Address;
use Symfony\Component\Mime\Email;
use Symfony\Component\Translation\LocaleSwitcher;
use Twig\Environment;

/**
 * Renders and delivers one row of the mail log. Used by the async handler and,
 * for the admin's "send a test" button, called straight through so a broken SMTP
 * account reports its error to the admin's face instead of into a retry queue.
 *
 * The transport is built per send from the admin-configured DSN rather than from
 * MAILER_DSN, because the whole point of the sender-account screen is that a lab
 * can change where its mail goes without touching the deployment's env.
 */
final class MailSender
{
    public function __construct(
        private readonly MailSettings $settings,
        private readonly MailLog $log,
        private readonly Environment $twig,
        private readonly LocaleSwitcher $localeSwitcher,
    ) {
    }

    /**
     * Sends the logged mail, updating its status either way.
     *
     * @throws \Throwable the transport error, so Messenger can retry a queued send
     */
    public function send(int $logId): void
    {
        $row = $this->log->find($logId);
        if ($row === null) {
            return;
        }

        if (!$this->settings->isConfigured()) {
            $this->log->markFailed($logId, 'No sender account configured (admin → Réglages → Envoi d\'e-mails).');

            return;
        }

        try {
            [$subject, $html, $text] = $this->render(
                (string) $row['template'],
                json_decode((string) $row['contextJson'], true, 512, JSON_THROW_ON_ERROR) ?: [],
                (string) $row['locale'],
            );

            $email = (new Email())
                ->from(new Address($this->settings->getFromAddress(), $this->settings->getFromName()))
                ->to(new Address((string) $row['recipient'], (string) ($row['recipientName'] ?? '')))
                ->subject($subject)
                ->text($text)
                ->html($html);

            if (($replyTo = $this->settings->getReplyTo()) !== '') {
                $email->replyTo($replyTo);
            }

            Transport::fromDsn($this->settings->getTransportDsn())->send($email);
        } catch (\Throwable $e) {
            $this->log->markFailed($logId, $e->getMessage());

            throw $e;
        }

        $this->log->markSent($logId, $subject);
    }

    /**
     * @param array<string, mixed> $context
     *
     * @return array{0: string, 1: string, 2: string} subject, html, text
     */
    private function render(string $template, array $context, string $locale): array
    {
        $name = preg_replace('/[^a-z0-9_-]/', '', $template) ?? '';
        if ($name === '') {
            throw new \RuntimeException(sprintf('Invalid mail template "%s".', $template));
        }

        // Each recipient's own language, which in a worker is nobody's request locale.
        // Every template can brand itself with the configured sender name without
        // each caller having to remember to pass it.
        $context += ['sender_name' => $this->settings->getFromName()];

        return $this->localeSwitcher->runWithLocale($locale, function () use ($name, $context): array {
            $tpl = $this->twig->load('emails/' . $name . '.html.twig');
            // The full render is the wrapped document; the body block on its own is
            // what the plain-text alternative is derived from, without the chrome.
            $body = $tpl->renderBlock('body', $context);

            return [
                // The subject block is rendered by an HTML-escaping Twig, but a subject
                // header is plain text — an apostrophe must not arrive as &#039;.
                trim(html_entity_decode($tpl->renderBlock('subject', $context), ENT_QUOTES, 'UTF-8')),
                $tpl->render($context),
                $tpl->hasBlock('body_text', $context)
                    ? trim($tpl->renderBlock('body_text', $context))
                    : trim(html_entity_decode(strip_tags((string) preg_replace('#<br\s*/?>|</p>|</h\d>|</li>#i', "\n", $body)), ENT_QUOTES, 'UTF-8')),
            ];
        });
    }
}
