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
        private readonly MailOverrides $overrides,
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
     * Le rendu d'une surcharge : le texte de l'exploitant, dans le chrome livré.
     *
     * ⚠️ **L'objet retombe sur celui du gabarit livré quand la surcharge n'en
     * donne pas.** Un mail sans objet arrive « (aucun objet) » dans la plupart
     * des clients : c'est pire que le texte d'origine.
     *
     * @param array{subject: ?string, body: ?string} $override
     * @param array<string, mixed> $context
     *
     * @return array{0: string, 1: string, 2: string} subject, html, text
     */
    private function renderOverride(string $name, array $override, array $context): array
    {
        $delivered = $this->twig->load('emails/' . $name . '.html.twig');

        $subject = $override['subject'] !== null
            ? $this->overrides->fill($override['subject'], $context)
            : trim(html_entity_decode($delivered->renderBlock('subject', $context), ENT_QUOTES, 'UTF-8'));

        $bodyText = $this->overrides->fill((string) $override['body'], $context);

        $wrapped = $this->twig->load('emails/_override.html.twig')->render($context + [
            'override_subject' => $subject,
            // ⚠️ Échappé PUIS `nl2br` : l'ordre compte. `nl2br` d'abord
            // produirait des `<br>` que l'échappement transformerait en texte.
            'override_body' => nl2br(htmlspecialchars($bodyText, ENT_QUOTES, 'UTF-8')),
        ]);

        $text = trim($bodyText);
        if (isset($context['unsubscribe_url'])) {
            $text .= "\n\n" . $this->translator->trans('mail.footer.unsubscribe') . ' : ' . $context['unsubscribe_url'];
        }

        return [$subject, $wrapped, $text];
    }

    /**
     * Rend un mail sans l'envoyer — objet, HTML, texte.
     *
     * 🔴 **PUBLIQUE depuis S161, et c'est tout l'intérêt.** L'aperçu de l'éditeur
     * passe par CETTE méthode, pas par une approximation : la mesure de sortie
     * de la session est « l'aperçu rend le vrai gabarit ». Un second moteur de
     * rendu pour la prévisualisation finirait par diverger de celui qui envoie —
     * exactement le défaut qu'un aperçu est censé prévenir.
     *
     * ⚠️ Elle ne touche ni la file ni le transport : rendre n'est pas envoyer.
     *
     * @param array<string, mixed> $context
     *
     * @return array{0: string, 1: string, 2: string} subject, html, text
     */
    public function render(string $template, array $context, string $locale): array
    {
        $name = preg_replace('/[^a-z0-9_-]/', '', $template) ?? '';
        if ($name === '') {
            throw new \RuntimeException(sprintf('Invalid mail template "%s".', $template));
        }

        // Each recipient's own language, which in a worker is nobody's request locale.
        // Every template can brand itself with the configured sender name without
        // each caller having to remember to pass it.
        $context += ['sender_name' => $this->settings->getFromName()];

        return $this->localeSwitcher->runWithLocale($locale, function () use ($name, $context, $locale): array {
            /*
             * 🔴 **S160 — le texte de l'exploitant, s'il en a écrit un.**
             *
             * ⚠️ **Le repli est la RÈGLE, pas le cas d'erreur.** Sans surcharge,
             * sans table, ou si quoi que ce soit lève, on charge le gabarit
             * livré : un mot de passe oublié doit partir quelle que soit la
             * bêtise saisie dans l'éditeur. `MailOverrides` ne peut
             * structurellement pas empêcher un envoi — toutes ses lectures sont
             * enveloppées, et le `catch` ci-dessous couvre le rendu lui-même.
             *
             * ⚠️ **Le texte saisi ne passe JAMAIS par le compilateur Twig.** Il
             * est substitué en PHP sur une liste fermée de champs, échappé, puis
             * injecté dans `_override.html.twig`, qui n'apporte que le chrome du
             * layout. Compiler du texte d'exploitant, c'est offrir l'exécution
             * de code arbitraire à qui édite un e-mail.
             */
            try {
                $override = $this->overrides->find($name, $locale);
                if ($override !== null) {
                    return $this->renderOverride($name, $override, $context);
                }
            } catch (\Throwable) {
                // Une surcharge cassée ne bloque pas le mail : on retombe sur le
                // texte livré, sans que le destinataire voie quoi que ce soit.
            }

            $tpl = $this->twig->load('emails/' . $name . '.html.twig');
            // The full render is the wrapped document; the body block on its own is
            // what the plain-text alternative is derived from, without the chrome.
            $body = $tpl->renderBlock('body', $context);

            $text = $tpl->hasBlock('body_text', $context)
                ? trim($tpl->renderBlock('body_text', $context))
                : trim(html_entity_decode(strip_tags((string) preg_replace(
                    [
                        // Flatten links to "label: url" BEFORE stripping tags.
                        // strip_tags keeps the label and throws the href away, which
                        // silently turns an actionable link into dead words — and
                        // for a guest cancelling an event registration, that link is
                        // the only lever they have.
                        '#<a\b[^>]*href=(["\'])(.*?)\1[^>]*>(.*?)</a>#is',
                        '#<br\s*/?>|</p>|</h\d>|</li>#i',
                    ],
                    ['$3 : $2', "\n"],
                    $body,
                )), ENT_QUOTES, 'UTF-8'));

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
