<?php

namespace App\Feature;

use App\Mail\MailSettings;
use App\Mail\ReminderSettings;
use App\Service\SiteSettingService;

/**
 * What is not yet configured, and why it matters.
 *
 * Several parts of FabOS degrade **silently by design** — the alternative to a
 * missing setting is usually worse than doing nothing. `Mailer` refuses rather
 * than throwing from the middle of a booking. An absent `public_base_url` means
 * mail carries *no link* rather than a link to `http://localhost`. A ticket with
 * no public URL carries no QR code.
 *
 * Every one of those is the right behaviour and every one of them is invisible.
 * This is where they become discoverable, so an operator finds out from a panel
 * rather than from someone telling them the confirmation mail never arrived.
 *
 * Severity is about consequence, not about how unusual the state is:
 *  - **blocking** — something people will try to use simply does not work.
 *  - **degraded** — it works, but quietly does less than the operator expects.
 *  - **info** — a deliberate choice worth confirming, not a problem.
 */
final class SetupHealth
{
    public const BLOCKING = 'blocking';
    public const DEGRADED = 'degraded';
    public const INFO = 'info';

    public function __construct(
        private readonly SiteSettingService $settings,
        private readonly MailSettings $mail,
        private readonly ReminderSettings $reminders,
        private readonly SiteFeatureService $features,
    ) {
    }

    /**
     * @return list<array{severity: string, ok: bool, title: string, consequence: string, route: ?string}>
     */
    public function checks(): array
    {
        $checks = [];

        // Mail first: it is the one whose absence is hardest to notice, because
        // nothing anywhere errors — the message simply never goes out.
        $checks[] = [
            'severity' => self::BLOCKING,
            'ok' => $this->mail->isConfigured(),
            'title' => 'Compte d’envoi e-mail',
            'consequence' => 'Sans compte d’envoi, aucun e-mail ne part : ni confirmation de réservation, ni billet d’événement, ni rappel. Rien n’échoue visiblement — les messages ne sont simplement jamais envoyés.',
            'route' => 'app_admin_emails',
        ];

        // Pause is a deliberate act, so it is reported separately from "never
        // configured" — telling someone to configure what they just switched off
        // would be answering the wrong question.
        if ($this->mail->isConfigured() && $this->mail->isPaused()) {
            $checks[] = [
                'severity' => self::BLOCKING,
                'ok' => false,
                'title' => 'Envoi d’e-mails en pause',
                'consequence' => 'L’envoi est configuré mais suspendu. Les e-mails transactionnels — confirmations, billets — ne partent pas non plus.',
                'route' => 'app_admin_emails',
            ];
        }

        $checks[] = [
            'severity' => self::DEGRADED,
            'ok' => $this->settings->getPublicBaseUrl() !== '',
            'title' => 'URL publique du site',
            'consequence' => 'Les e-mails sont rendus par un worker, hors requête : sans cette adresse ils ne contiennent **aucun lien** plutôt qu’un lien cassé, et les billets d’événement ne portent pas de QR code.',
            'route' => 'app_admin_settings',
        ];

        $checks[] = [
            'severity' => self::DEGRADED,
            'ok' => $this->settings->getLabAddress() !== '',
            'title' => 'Adresse du lieu',
            'consequence' => 'Les événements sur place affichent l’adresse de l’organisation. Sans elle, la page de l’événement ne dit pas où il se tient et le bouton d’itinéraire disparaît.',
            'route' => 'app_admin_settings',
        ];

        // "Nothing is on" is a coherent state to be in for five minutes during
        // setup and a broken one to leave: the site renders a home page over an
        // app that does nothing.
        $enabled = array_filter($this->features->all());
        $checks[] = [
            'severity' => self::BLOCKING,
            'ok' => $enabled !== [],
            'title' => 'Au moins une fonctionnalité active',
            'consequence' => 'Toutes les fonctionnalités sont désactivées : le site affiche une page d’accueil au-dessus d’une application qui ne fait rien.',
            'route' => 'app_admin_features',
        ];

        $checks[] = [
            'severity' => self::INFO,
            'ok' => $this->features->hasCalendarLayer(),
            'title' => 'Calendrier de réservation',
            'consequence' => 'Aucune ressource ne s’affiche sur le calendrier partagé, donc la page et son lien se retirent. C’est le bon comportement pour un déploiement qui ne réserve rien — à confirmer si ce n’est pas voulu.',
            'route' => 'app_admin_features',
        ];

        $activeReminders = array_filter(
            ReminderSettings::KINDS,
            fn (string $kind): bool => $this->reminders->isEnabled($kind),
        );
        $checks[] = [
            'severity' => self::INFO,
            'ok' => $activeReminders !== [],
            'title' => 'Rappels programmés',
            'consequence' => 'Aucun rappel n’est activé. Les e-mails transactionnels partent toujours ; ce sont les rappels « votre réservation commence demain » qui ne partent pas.',
            'route' => 'app_admin_emails',
        ];

        return $checks;
    }

    /** @return array{blocking: int, degraded: int, info: int} */
    public function counts(): array
    {
        $counts = [self::BLOCKING => 0, self::DEGRADED => 0, self::INFO => 0];
        foreach ($this->checks() as $check) {
            if (!$check['ok']) {
                ++$counts[$check['severity']];
            }
        }

        return $counts;
    }

    /** Whether anything at all needs the operator's attention. */
    public function isHealthy(): bool
    {
        $counts = $this->counts();

        return $counts[self::BLOCKING] === 0 && $counts[self::DEGRADED] === 0;
    }
}
