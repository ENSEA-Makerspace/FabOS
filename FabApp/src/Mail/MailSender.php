<?php

namespace App\Mail;

use Symfony\Component\Mailer\Transport;
use Symfony\Component\Mime\Address;
use Symfony\Component\Mime\Email;
use Symfony\Component\Translation\LocaleSwitcher;
use Symfony\Contracts\Translation\TranslatorInterface;
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
        private readonly UnsubscribeLinker $unsubscribe,
        private readonly TranslatorInterface $translator,
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
            $context = json_decode((string) $row['contextJson'], true, 512, JSON_THROW_ON_ERROR) ?: [];

            // Built here rather than by the caller: only the log row knows both
            // who the mail is for and which category it went out under, and the
            // link has to survive a retry that re-renders from that row alone.
            $unsubscribeUrl = $this->unsubscribe->urlFor(
                isset($row['userId']) ? (int) $row['userId'] : null,
                (string) $row['category'],
            );
            if ($unsubscribeUrl !== null) {
                $context['unsubscribe_url'] = $unsubscribeUrl;
            }

            [$subject, $html, $text] = $this->render(
                (string) $row['template'],
                $context,
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

            if ($unsubscribeUrl !== null) {
                // RFC 8058: lets a mail client show its own unsubscribe button and
                // POST to the link directly, which is why the route accepts POST
                // without a CSRF token — the signature is the authorisation.
                $email->getHeaders()->addTextHeader('List-Unsubscribe', '<' . $unsubscribeUrl . '>');
                $email->getHeaders()->addTextHeader('List-Unsubscribe-Post', 'List-Unsubscribe=One-Click');
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

            $text = $tpl->hasBlock('body_text', $context)
                ? trim($tpl->renderBlock('body_text', $context))
                : trim(html_entity_decode(strip_tags((string) preg_replace('#<br\s*/?>|</p>|</h\d>|</li>#i', "\n", $body)), ENT_QUOTES, 'UTF-8'));

            // The text alternative is built from the body block alone, so it never
            // picks up the layout's footer — and a client showing the text part
            // would otherwise offer no way out at all. Append it explicitly.
            if (isset($context['unsubscribe_url'])) {
                $text .= "\n\n" . $this->translator->trans('mail.footer.unsubscribe') . ' : ' . $context['unsubscribe_url'];
            }

            return [
                // The subject block is rendered by an HTML-escaping Twig, but a subject
                // header is plain text — an apostrophe must not arrive as &#039;.
                trim(html_entity_decode($tpl->renderBlock('subject', $context), ENT_QUOTES, 'UTF-8')),
                $tpl->render($context),
                $text,
            ];
        });
    }
}
