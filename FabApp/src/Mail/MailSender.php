<?php

namespace App\Mail;

use Psr\Log\LoggerInterface;
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
        private readonly LoggerInterface $logger,
    ) {
    }

    /**
     * Quelle version a servi — c'est la question « pourquoi ce mail dit ça ? »
     * (S162), et c'est un critère de sortie de la phase.
     *
     * ⚠️ **Une trace, pas un booléen.** Trois sources se combinent : le corps
     * (livré, réécrit, ou réécrit-mais-cassé) et les deux parties du chrome.
     * « Réécrit » tout court ne dirait pas si c'est le pied commun qui a changé
     * le mail, alors que c'est justement le cas qui touche vingt gabarits d'un
     * coup.
     */
    public const RENDER_DELIVERED = 'delivered';
    public const RENDER_OVERRIDE = 'override';
    public const RENDER_FAILED = 'override_failed';

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

            [$subject, $html, $text, $renderedFrom] = $this->render(
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

        $this->log->markSent($logId, $subject, $renderedFrom);
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
        $this->assertRenderable($override['subject'], $override['body']);

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
     * @return array{0: string, 1: string, 2: string, 3: string} subject, html, text,
     *         et la TRACE de ce qui a servi (S162) — les appelants qui n'en ont
     *         pas besoin la laissent tomber, `[$a, $b, $c] = …` ignore le reste
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
            [$context, $chrome] = $this->withLayoutParts($context, $locale);

            $bodyTrace = self::RENDER_DELIVERED;
            try {
                $override = $this->overrides->find($name, $locale);
                if ($override !== null) {
                    return [...$this->renderOverride($name, $override, $context), self::RENDER_OVERRIDE . $chrome];
                }
            } catch (\Throwable $e) {
                /*
                 * 🔴 **L'incident est JOURNALISÉ, il n'est plus avalé (S162).**
                 * Le repli lui-même était déjà la bonne réponse — un mot de
                 * passe oublié part quoi qu'il arrive — mais un repli SILENCIEUX
                 * rend « pourquoi ce mail dit ça ? » insoluble : l'exploitant
                 * voit son texte enregistré dans l'éditeur et le texte livré
                 * dans sa boîte, sans rien qui explique l'écart.
                 */
                $bodyTrace = self::RENDER_FAILED;
                $this->logIncident($name, $locale, 'body', $e);
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
                $bodyTrace . $chrome,
            ];
        });
    }
    /**
     * L'en-tête et le pied réécrits, prêts à injecter dans le layout (S162).
     *
     * 🔴 **Forcés dans le contexte, jamais fusionnés.** Un appelant qui passerait
     * `layout_header` dans son contexte réécrirait le chrome de son mail sans
     * passer par l'éditeur — `+=` lui aurait laissé la priorité. Ici la valeur
     * est écrasée dans tous les cas, y compris par `null`.
     *
     * ⚠️ **Chaque partie a son propre repli.** Un pied cassé ne doit pas emporter
     * l'en-tête avec lui : ce sont deux textes indépendants, saisis à deux
     * moments différents.
     *
     * @param array<string, mixed> $context
     *
     * @return array{0: array<string, mixed>, 1: string} le contexte, et le suffixe de trace
     */
    private function withLayoutParts(array $context, string $locale): array
    {
        $chrome = '';

        foreach (['_header' => 'layout_header', '_footer' => 'layout_footer'] as $key => $var) {
            $context[$var] = null;

            try {
                $part = $this->overrides->find($key, $locale);
                if ($part === null || $part['body'] === null) {
                    continue;
                }

                $this->assertRenderable(null, $part['body']);

                // Même contrat que le corps : substitution en PHP sur une liste
                // fermée, échappement, puis `nl2br` — le texte de l'exploitant ne
                // voit jamais le compilateur Twig.
                $context[$var] = nl2br(htmlspecialchars(
                    $this->overrides->fill($part['body'], $context),
                    ENT_QUOTES,
                    'UTF-8',
                ));
                $chrome .= '+' . ltrim($key, '_');
            } catch (\Throwable $e) {
                $context[$var] = null;
                $chrome .= '+' . ltrim($key, '_') . '_failed';
                $this->logIncident($key, $locale, 'layout', $e);
            }
        }

        return [$context, $chrome];
    }

    /**
     * 🔴 **Ce qui rend une surcharge INRENDABLE — et donc ce qui déclenche le
     * repli (S162).**
     *
     * **Un objet qui contient un saut de ligne**, d'abord, et c'est le cas
     * réellement atteignable : le champ Objet est un `<input>`, dont un
     * navigateur retire les retours — mais un POST fabriqué à la main, non. Un
     * objet à rallonge sur deux lignes est un en-tête SMTP mal formé ; Symfony
     * encode ses en-têtes et l'injection n'aboutit pas, mais le mail part avec
     * un objet illisible, ou pas du tout, selon le transport.
     *
     * ⚠️ **L'UTF-8 invalide, ensuite, et il faut dire ce qu'il vaut** :
     * `htmlspecialchars(…, ENT_QUOTES, 'UTF-8')` rend la CHAÎNE VIDE sur une
     * séquence invalide, sans lever — un corps abîmé enverrait donc un mail vide
     * avec le chrome et rien dedans. 🅿️ Mais la colonne est en `utf8mb4` et
     * MariaDB refuse déjà ces octets à l'écriture : c'est une ceinture par-dessus
     * les bretelles, pour le jour où le texte arrive d'ailleurs — une
     * restauration, une colonne binaire, un import.
     */
    private function assertRenderable(?string $subject, ?string $body): void
    {
        foreach (['subject' => $subject, 'body' => $body] as $field => $text) {
            if ($text === null) {
                continue;
            }

            if (!mb_check_encoding($text, 'UTF-8')) {
                throw new \RuntimeException(sprintf('Override %s is not valid UTF-8.', $field));
            }
        }

        // ⚠️ Le CORPS, lui, a le droit d'avoir des retours à la ligne : c'est un
        // texte, et `nl2br` en fait des `<br>`. Seul l'objet est un en-tête.
        if ($subject !== null && preg_match('/[\r\n]/', $subject) === 1) {
            throw new \RuntimeException('Override subject contains a line break.');
        }
    }

    /**
     * ⚠️ **`error` et pas `warning`.** Un mail parti avec un autre texte que
     * celui que l'exploitant croit avoir écrit est un défaut à corriger, pas une
     * curiosité — et le niveau est ce qui décide s'il apparaît quelque part.
     */
    private function logIncident(string $key, string $locale, string $part, \Throwable $e): void
    {
        $this->logger->error('Mail override fell back to the delivered text.', [
            'template' => $key,
            'locale' => $locale,
            'part' => $part,
            'error' => $e->getMessage(),
        ]);
    }
}
