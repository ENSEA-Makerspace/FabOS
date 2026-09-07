<?php

namespace App\Command;

use App\Mail\MailOverrides;
use App\Mail\MailSender;
use Doctrine\DBAL\Connection;
use Monolog\Handler\TestHandler;
use Monolog\Logger;
use Psr\Log\LoggerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S162 — la sonde du chrome réécrivable et de la garde du transactionnel.
 *
 * 🔴 **La mesure de sortie de la phase, mot pour mot** : « une surcharge
 * volontairement cassée sur `password_reset` : le mail part quand même, avec le
 * texte livré, et l'incident est journalisé ».
 *
 * ⚠️ **Casser une surcharge demande de la MALICE, et c'est une bonne nouvelle.**
 * Le texte de l'exploitant ne voit jamais le compilateur Twig : il n'y a ni
 * boucle, ni condition, ni filtre à faire échouer. Le seul défaut réellement
 * atteignable est un OBJET SUR DEUX LIGNES — un en-tête SMTP mal formé — et il
 * n'est atteignable que par un POST fabriqué à la main, parce qu'un navigateur
 * retire les retours d'un `<input>`. La sonde l'écrit donc en SQL direct, en
 * court-circuitant l'éditeur qui le refuse maintenant avec une phrase.
 *
 * 🅿️ **Ce n'est donc pas un défaut observé en production** — c'est une panne
 * fabriquée pour mettre le repli à l'épreuve, et le dire ainsi vaut mieux que de
 * laisser croire qu'on a réparé quelque chose de cassé.
 *
 * ✅ **Elle n'envoie AUCUN courrier** : tout passe par `render()`, qui ne touche
 * ni la file ni le transport.
 */
#[AsCommand(name: 'app:s162:layout-probe', description: 'S162 : prouve qu\'une surcharge cassée n\'empêche pas un mail de partir, que l\'incident est journalisé, et que l\'en-tête et le pied se réécrivent séparément. N\'envoie rien.')]
final class S162LayoutProbeCommand extends Command
{
    private const TEMPLATE = 'password_reset';
    private const LOCALE = 'fr';

    public function __construct(
        private readonly MailOverrides $overrides,
        private readonly MailSender $sender,
        private readonly Connection $db,
        private readonly LoggerInterface $logger,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        // ⚠️ La sonde écrit puis efface. Effacer le texte d'un exploitant pour se
        // prouver un point serait indéfendable.
        if ($this->overrides->existingKeys() !== []) {
            $io->error('Des surcharges existent déjà. La sonde écrit puis efface : elle ne tournera pas sur des données réelles.');

            return Command::FAILURE;
        }

        // Un contexte de mot de passe oublié, plus un lien de désinscription :
        // c'est la seule façon de vérifier qu'un pied réécrit ne l'emporte pas.
        $context = [
            'resetUrl' => 'https://exemple/reinitialiser',
            'validHours' => 2,
            'sender_name' => 'FabOS',
            'unsubscribe_url' => 'https://exemple/desinscription',
        ];

        $io->section('1. Le texte livré, avant quoi que ce soit');
        [$subject0, $html0, , $trace0] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        $fingerprint = hash('sha256', $subject0 . "\0" . $html0);
        $io->writeln('   trace : ' . $trace0);
        $this->check($io, $failures, 'la trace dit « livré »', $trace0 === MailSender::RENDER_DELIVERED);

        $io->section('2. 🔴 Une surcharge VOLONTAIREMENT CASSÉE sur password_reset');
        // Écrit en SQL direct : l'éditeur refuse désormais un objet sur deux
        // lignes avec une phrase, et c'est précisément ce qu'on veut contourner
        // pour mettre le repli à l'épreuve.
        $this->writeRaw(self::TEMPLATE, self::LOCALE, "OBJET CASSÉ\r\nBcc: ailleurs@exemple", 'Un corps parfaitement valide.');
        $this->check($io, $failures, 'la ligne cassée est bien en base', $this->overrides->find(self::TEMPLATE, self::LOCALE) !== null);

        $records = $this->captureLogs(function () use (&$subjectB, &$htmlB, &$traceB, $context): void {
            [$subjectB, $htmlB, , $traceB] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        });

        $io->writeln('   trace : ' . $traceB);
        $this->check($io, $failures, '🔴 le mail part quand même — objet et corps non vides', $subjectB !== '' && trim(strip_tags($htmlB)) !== '');
        $this->check($io, $failures, '🔴 avec le texte LIVRÉ, au bit près', hash('sha256', $subjectB . "\0" . $htmlB) === $fingerprint);
        $this->check($io, $failures, 'l\'objet cassé n\'a pas fui dans l\'en-tête', !str_contains($subjectB, 'Bcc:'));
        $this->check($io, $failures, '🔴 la trace dit que la surcharge a ÉCHOUÉ', $traceB === MailSender::RENDER_FAILED);
        // ⚠️ Non mesurable ⇒ EN ÉCHEC, jamais « passé ». Une assertion qu'on ne
        // peut pas mesurer et qu'on laisse verte est pire que pas d'assertion.
        if ($records === null) {
            $io->warning('Le logger n\'est pas un Monolog\Logger : « l\'incident est journalisé » n\'a pas pu être MESURÉ ici.');
        }
        $this->check($io, $failures, '🔴 l\'incident est journalisé', $records !== null && $this->mentions($records, 'password_reset'));

        $io->section('3. On retire la surcharge cassée');
        $this->overrides->save(self::TEMPLATE, self::LOCALE, '', '');
        [$s, $h, , $t] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        $this->check($io, $failures, 'le rendu revient identique au bit près', hash('sha256', $s . "\0" . $h) === $fingerprint);
        $this->check($io, $failures, 'la trace redit « livré »', $t === MailSender::RENDER_DELIVERED);

        $io->section('4. L\'en-tête et le pied se réécrivent SÉPARÉMENT');
        $this->check($io, $failures, 'le pied s\'enregistre', $this->overrides->save('_footer', self::LOCALE, '', 'Écrit par {{ sender_name }}.'));
        [, $htmlF, , $traceF] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        $io->writeln('   trace : ' . $traceF);
        $this->check($io, $failures, 'le pied réécrit apparaît', str_contains($htmlF, 'Écrit par FabOS.'));
        $this->check($io, $failures, 'le champ y est REMPLACÉ, pas écrit tel quel', !str_contains($htmlF, '{{ sender_name }}'));
        $this->check($io, $failures, 'l\'en-tête n\'a PAS bougé', str_contains($htmlF, '>FabOS</span>'));
        // 🔴 Le pied porte une obligation — se désinscrire — que l'exploitant ne
        // doit pas pouvoir retirer en réécrivant sa phrase.
        $this->check($io, $failures, '🔴 le lien de désinscription est TOUJOURS là', str_contains($htmlF, 'https://exemple/desinscription'));
        $this->check($io, $failures, 'la trace nomme le pied', $traceF === MailSender::RENDER_DELIVERED . '+footer');

        $this->check($io, $failures, 'l\'en-tête s\'enregistre', $this->overrides->save('_header', self::LOCALE, '', 'ATELIER DES DRYADES'));
        [, $htmlH, , $traceH] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        $this->check($io, $failures, 'l\'en-tête réécrit apparaît', str_contains($htmlH, 'ATELIER DES DRYADES'));
        $this->check($io, $failures, 'la trace nomme les deux', $traceH === MailSender::RENDER_DELIVERED . '+header+footer');

        $io->section('5. 🔴 Le chrome ne peut pas contenir de code non plus');
        $this->overrides->save('_footer', self::LOCALE, '', '{{ 7 * 7 }} <script>alert(1)</script>');
        [, $htmlEvil] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        $this->check($io, $failures, 'une expression Twig n\'est pas évaluée', !str_contains($htmlEvil, '49'));
        $this->check($io, $failures, 'une balise script est échappée', !str_contains($htmlEvil, '<script>alert(1)</script>'));
        $this->overrides->save('_footer', self::LOCALE, '', 'Écrit par {{ sender_name }}.');

        $io->section('6. 🔴 Les trois replis sont INDÉPENDANTS');
        // Un corps cassé ne doit pas emporter le chrome réécrit avec lui : ce
        // sont trois textes distincts, saisis à trois moments différents, et
        // c'est le seul point que la trace combinée sert à prouver.
        $this->writeRaw(self::TEMPLATE, self::LOCALE, "OBJET CASSÉ\r\nBcc: ailleurs@exemple", 'Corps.');
        [, $htmlMix, , $traceMix] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        $io->writeln('   trace : ' . $traceMix);
        $this->check($io, $failures, 'l\'en-tête réécrit tient', str_contains($htmlMix, 'ATELIER DES DRYADES'));
        $this->check($io, $failures, 'le pied réécrit tient', str_contains($htmlMix, 'Écrit par FabOS.'));
        $this->check($io, $failures, 'le corps, lui, revient au texte livré', str_contains($htmlMix, 'https://exemple/reinitialiser'));
        $this->check($io, $failures, 'la trace dit les trois d\'un coup', $traceMix === MailSender::RENDER_FAILED . '+header+footer');
        $this->overrides->save(self::TEMPLATE, self::LOCALE, '', '');

        $io->section('6bis. La garde d\'écriture refuse ce que le repli sait absorber');
        // ⚠️ Le repli est un FILET, pas une porte d'entrée : ce qui est
        // inrendable ne doit pas pouvoir entrer en base par l'éditeur.
        $this->check($io, $failures, 'un objet sur deux lignes est refusé à l\'écriture', !$this->overrides->save(self::TEMPLATE, self::LOCALE, "A\r\nB", 'x'));
        $this->check($io, $failures, 'de l\'UTF-8 invalide aussi', !$this->overrides->save(self::TEMPLATE, self::LOCALE, 'ok', "abc\xC3\x28def"));
        $this->check($io, $failures, 'et rien n\'a été écrit', $this->overrides->find(self::TEMPLATE, self::LOCALE) === null);

        $io->section('7. On retire tout — le texte livré revient à l\'identique');
        foreach (['_header', '_footer'] as $part) {
            $this->overrides->save($part, self::LOCALE, '', '');
        }
        [$sEnd, $hEnd, , $tEnd] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        $this->check($io, $failures, 'le rendu est identique au bit près', hash('sha256', $sEnd . "\0" . $hEnd) === $fingerprint);
        $this->check($io, $failures, 'la trace redit « livré », sans chrome', $tEnd === MailSender::RENDER_DELIVERED);
        $this->check($io, $failures, 'la table est rendue vide', $this->overrides->existingKeys() === []);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S162 verte. Aucun courrier envoyé, table rendue vide.');

        return Command::SUCCESS;
    }

    /**
     * Écrit une ligne SANS passer par `save()`, qui refuse désormais exactement
     * ce que la sonde a besoin de mettre en base.
     *
     * 🅿️ **Ce qu'on ne peut PAS écrire ici, et il faut le dire** : de l'UTF-8
     * invalide. La colonne est en `utf8mb4` et MariaDB refuse la séquence — le
     * garde-fou du rendu contre ces octets est donc une ceinture par-dessus les
     * bretelles, pour un texte qui arriverait d'ailleurs (restauration, import),
     * et non la réparation d'un défaut observable. La sonde le mesure côté
     * écriture (section 6bis) plutôt que de le mettre en scène côté rendu.
     */
    private function writeRaw(string $key, string $locale, ?string $subject, string $body): void
    {
        $this->db->executeStatement(
            'INSERT INTO EMAIL_TEMPLATE_OVERRIDE (templateKey, locale, subject, body, updatedAt)
             VALUES (?, ?, ?, ?, NOW())
             ON DUPLICATE KEY UPDATE subject = VALUES(subject), body = VALUES(body)',
            [$key, $locale, $subject, $body],
        );
    }

    /**
     * Branche un collecteur sur le logger de l'application le temps d'un rendu.
     *
     * 🔴 **C'est la seule mesure honnête de « l'incident est journalisé ».** Lire
     * un fichier de log dirait où il est écrit, pas qu'il l'a été par ce rendu-ci
     * — et en production ces lignes partent sur `stderr`, donc dans le journal
     * systemd, qu'un processus ne peut pas relire de l'intérieur.
     *
     * @return list<string>|null `null` quand le logger n'est pas un Monolog
     */
    private function captureLogs(callable $run): ?array
    {
        if (!$this->logger instanceof Logger) {
            $run();

            return null;
        }

        $handler = new TestHandler();
        $this->logger->pushHandler($handler);

        try {
            $run();
        } finally {
            $this->logger->popHandler();
        }

        // ⚠️ Monolog 3 rend des `LogRecord`, Monolog 2 des tableaux. Les deux
        // formes sont acceptées : la sonde ne doit pas casser sur une montée de
        // version d'une dépendance qu'elle ne fait qu'observer.
        return array_map(
            static function (mixed $r): string {
                $message = is_array($r) ? $r['message'] : $r->message;
                $ctx = is_array($r) ? ($r['context'] ?? []) : $r->context;

                return (string) $message . ' ' . json_encode($ctx, JSON_UNESCAPED_SLASHES);
            },
            $handler->getRecords(),
        );
    }

    /** @param list<string> $records */
    private function mentions(array $records, string $needle): bool
    {
        foreach ($records as $line) {
            if (str_contains($line, 'fell back') && str_contains($line, $needle)) {
                return true;
            }
        }

        return false;
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
