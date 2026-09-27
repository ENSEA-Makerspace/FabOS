<?php

namespace App\Command;

use App\Mail\Mailer;
use App\Mail\MailSender;
use App\Mail\NotificationCategory;
use App\Mail\MailSettings;
use Doctrine\DBAL\Connection;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Contracts\HttpClient\HttpClientInterface;

/**
 * Le mode test du courrier : ce qui ARRIVE vraiment, lu dans la boîte de capture.
 *
 * ⚠️ Ne tourne que si le compte d'envoi est Mailpit (le serveur de capture de
 * CT 210) : la sonde envoie pour de bon, puis lit le message reçu par l'API de
 * Mailpit — destinataire, objet, en-têtes. Face à un vrai SMTP, elle refuse.
 *
 *   1. Mode test actif → le courrier pour un membre arrive à l'adresse de test,
 *      objet « [TEST → membre] … », en-tête d'origine, AUCUN lien de désinscription ;
 *      et RIEN n'arrive au membre.
 *   2. Adresse de redirection invalide → rien ne part, pas même au membre.
 *   3. Mode test vide → envoi normal au destinataire (la mesure voit la différence).
 *
 * ✅ Réglages et journal : transaction annulée. Les messages de sonde sont
 * effacés de Mailpit à la fin.
 */
#[AsCommand(name: 'app:mail:redirect-probe', description: 'Mode test du courrier : tout part vers une seule adresse, vérifié dans Mailpit (CT 210). Transaction annulée, messages de sonde effacés.')]
final class MailRedirectProbeCommand extends Command
{
    private const MAILPIT = 'http://127.0.0.1:32769';
    private const REDIRECT = 'mode-test@sonde.example.org';

    public function __construct(
        private readonly Connection $db,
        private readonly Mailer $mailer,
        private readonly MailSender $sender,
        private readonly MailSettings $settings,
        private readonly HttpClientInterface $http,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];
        if (!str_contains($this->settings->getTransportDsn(), '127.0.0.1:32768')) {
            $io->error('Le compte d’envoi n’est pas Mailpit : cette sonde enverrait de vrais courriers. Refus.');

            return Command::FAILURE;
        }
        $tag = 'sonde-redirect-' . bin2hex(random_bytes(3));
        $member = 'membre-' . $tag . '@example.org';
        $before = (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG');
        $redirectBefore = $this->settings->getRedirectTo();

        $this->db->beginTransaction();
        try {
            $io->section('1. Mode test actif');
            $this->settings->setRedirectTo(self::REDIRECT);
            $error = $this->mailer->sendNow($member, 'Membre Sonde', 'test', ['sent_at' => $tag]);
            $this->check($io, $failures, 'l’envoi réussit (' . ($error ?? 'ok') . ')', $error === null);
            $got = $this->find($tag);
            $to = array_map(static fn (array $a): string => (string) $a['Address'], $got['To'] ?? []);
            $this->check($io, $failures, '🔴 arrivé à l’adresse de test, et à elle seule (' . implode(', ', $to) . ')', $to === [self::REDIRECT]);
            $this->check($io, $failures, 'objet « [TEST → membre] … » (' . ($got['Subject'] ?? '—') . ')', str_starts_with((string) ($got['Subject'] ?? ''), '[TEST → ' . $member . ']'));
            $headers = $got !== null ? $this->headers((string) $got['ID']) : [];
            $this->check($io, $failures, 'l’en-tête X-FabOS-Original-To nomme le vrai destinataire', ($headers['X-Fabos-Original-To'][0] ?? $headers['X-FabOS-Original-To'][0] ?? null) === $member);
            // ⚠️ Mesuré sur un courrier QUI EN AURAIT UN : adressé à un compte,
            // catégorie dont on peut se désabonner. Le test d'envoi n'en a jamais
            // — le constat serait vrai pour une mauvaise raison.
            $withLink = $this->sendToAccount($tag . '-optout');
            $optoutHeaders = $withLink !== null ? $this->headers((string) $withLink['ID']) : [];
            $this->check($io, $failures, 'aucun List-Unsubscribe, sur un courrier qui en porterait un (il désinscrirait le vrai destinataire)', $withLink !== null && !isset($optoutHeaders['List-Unsubscribe']));
            $this->check($io, $failures, '🔴 rien n’est arrivé au membre', $this->countTo($member) === 0);

            $io->section('2. Adresse de redirection invalide');
            $this->settings->setRedirectTo('pas-une-adresse');
            $error = $this->mailer->sendNow($member, 'Membre Sonde', 'test', ['sent_at' => $tag . '-bad']);
            $this->check($io, $failures, 'rien ne part — ni à l’adresse invalide, ni au membre', $this->find($tag . '-bad') === null && $this->countTo($member) === 0);

            $io->section('3. Mode test vide : envoi normal');
            $this->settings->setRedirectTo('');
            $this->mailer->sendNow($member, 'Membre Sonde', 'test', ['sent_at' => $tag . '-normal']);
            $normal = $this->find($tag . '-normal');
            $this->check($io, $failures, 'arrivé au membre lui-même, objet sans préfixe', $normal !== null && ($normal['To'][0]['Address'] ?? null) === $member && !str_starts_with((string) $normal['Subject'], '[TEST'));
            $normalLink = $this->sendToAccount($tag . '-optout-normal');
            $this->check($io, $failures, 'et le même courrier « désabonnable » porte bien List-Unsubscribe en mode normal (la mesure voit la différence)', $normalLink !== null && isset($this->headers((string) $normalLink['ID'])['List-Unsubscribe']));
        } finally {
            $this->db->rollBack();
            $this->cleanup($tag);
        }

        $io->section('4. Rien n’est resté');
        $this->check($io, $failures, 'EMAIL_LOG inchangé', (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG') === $before);
        $this->check($io, $failures, 'réglage « mode test » inchangé', $this->settings->getRedirectTo() === $redirectBefore);
        $this->check($io, $failures, 'messages de sonde effacés de Mailpit', $this->find($tag) === null && $this->find($tag . '-normal') === null);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde « mode test » verte.');

        return Command::SUCCESS;
    }

    /** Le message dont le corps porte `$tag` (le gabarit `test` imprime `sent_at`). @return array<string, mixed>|null */
    private function find(string $tag): ?array
    {
        $found = $this->http->request('GET', self::MAILPIT . '/api/v1/search', ['query' => ['query' => '"' . $tag . '"']])->toArray();
        foreach ($found['messages'] ?? [] as $message) {
            $full = $this->http->request('GET', self::MAILPIT . '/api/v1/message/' . $message['ID'])->toArray();
            // `tag` ne doit pas trouver `tag-normal` : fin de mot exigée.
            if (preg_match('/' . preg_quote($tag, '/') . '(?![-\w])/', (string) ($full['Text'] ?? '') . ' ' . (string) ($full['HTML'] ?? ''))) {
                return $message;
            }
        }

        return null;
    }

    /**
     * Un courrier adressé à un COMPTE, dans une catégorie dont on peut se
     * désabonner (celui qui porte un lien de désinscription), envoyé tout de
     * suite. @return array<string, mixed>|null le message reçu par Mailpit
     */
    private function sendToAccount(string $tag): ?array
    {
        $userId = (int) $this->db->fetchOne("SELECT id FROM UTILISATEUR WHERE statut = 'actif' ORDER BY id LIMIT 1");
        $this->mailer->queue('compte-' . $tag . '@example.org', 'Compte Sonde', 'test', ['sent_at' => $tag], NotificationCategory::NEWS, null, $userId);
        $logId = (int) $this->db->fetchOne('SELECT MAX(id) FROM EMAIL_LOG');
        if ($this->db->fetchOne('SELECT status FROM EMAIL_LOG WHERE id = ?', [$logId]) !== 'sent') {
            $this->sender->send($logId);
        }

        return $this->find($tag);
    }

    private function countTo(string $address): int
    {
        return (int) ($this->http->request('GET', self::MAILPIT . '/api/v1/search', ['query' => ['query' => 'to:' . $address]])->toArray()['messages_count'] ?? 0);
    }

    /** @return array<string, list<string>> */
    private function headers(string $id): array
    {
        return $this->http->request('GET', self::MAILPIT . '/api/v1/message/' . $id . '/headers')->toArray();
    }

    private function cleanup(string $tag): void
    {
        $ids = [];
        foreach (['"' . $tag, 'to:' . self::REDIRECT, 'to:membre-' . $tag . '@example.org', 'to:compte-' . $tag . '-optout-normal@example.org'] as $query) {
            foreach ($this->http->request('GET', self::MAILPIT . '/api/v1/search', ['query' => ['query' => $query]])->toArray()['messages'] ?? [] as $m) {
                $ids[] = $m['ID'];
            }
        }
        if ($ids !== []) {
            $this->http->request('DELETE', self::MAILPIT . '/api/v1/messages', ['json' => ['IDs' => array_values(array_unique($ids))]])->getStatusCode();
        }
    }

    /** @param list<string> $failures */
    private function check(SymfonyStyle $io, array &$failures, string $what, bool $ok): void
    {
        $io->writeln(($ok ? '   <info>✓</info> ' : '   <error>✗</error> ') . $what);
        if (!$ok) {
            $failures[] = $what;
        }
    }
}
