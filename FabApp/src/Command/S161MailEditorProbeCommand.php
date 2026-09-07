<?php

namespace App\Command;

use App\Mail\MailOverrides;
use App\Mail\MailSender;
use App\Mail\MailTemplateCatalog;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S161 — la sonde de l'éditeur de textes d'e-mail.
 *
 * 🔴 **Les deux mesures de sortie de la session, dans l'ordre** : « un champ
 * inconnu est refusé avec une phrase » et « l'aperçu rend le vrai gabarit, pas
 * une approximation ».
 *
 * ⚠️ **Elle ÉCRIT une surcharge, puis la retire.** C'est la seule façon de
 * prouver qu'une surcharge s'applique ET que son retrait rend exactement le
 * texte livré. La table est vérifiée vide avant et après ; la sonde refuse de
 * démarrer si elle ne l'est pas, plutôt que de risquer d'écraser le travail de
 * quelqu'un.
 *
 * ✅ **Elle n'envoie AUCUN courrier** : `render()` ne touche ni la file ni le
 * transport. C'est justement pour ça qu'elle a été rendue publique.
 */
#[AsCommand(name: 'app:s161:mail-editor-probe', description: 'S161 : prouve le refus d\'un champ inconnu, et qu\'une surcharge s\'applique puis se retire proprement. N\'envoie rien.')]
final class S161MailEditorProbeCommand extends Command
{
    private const TEMPLATE = 'password_reset';
    private const LOCALE = 'fr';

    public function __construct(
        private readonly MailTemplateCatalog $catalog,
        private readonly MailOverrides $overrides,
        private readonly MailSender $sender,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        // ⚠️ On refuse de tourner si quelqu'un a déjà écrit quelque chose : la
        // sonde écrit puis efface, et effacer le texte d'un exploitant pour se
        // prouver un point serait indéfendable.
        if ($this->overrides->existingKeys() !== []) {
            $io->error('Des surcharges existent déjà. La sonde écrit puis efface : elle ne tournera pas sur des données réelles.');

            return Command::FAILURE;
        }

        $context = [
            'resetUrl' => 'https://exemple/reinitialiser',
            'validHours' => 2,
            'sender_name' => 'FabOS',
        ];

        $io->section('1. Les champs sont DÉDUITS du gabarit');
        $fields = $this->catalog->fieldsOf(self::TEMPLATE);
        $io->writeln('   ' . implode(', ', $fields));
        $this->check($io, $failures, 'le champ que ce gabarit utilise est proposé', in_array('resetUrl', $fields, true));
        $this->check($io, $failures, 'un champ d\'un AUTRE gabarit ne l\'est pas', !in_array('machine', $fields, true));

        $io->section('2. Un champ inconnu est refusé');
        $this->check($io, $failures, '« {{ machine }} » est signalé comme inconnu', $this->catalog->unknownFieldsIn('Bonjour {{ machine }}', self::TEMPLATE) === ['machine']);
        $this->check($io, $failures, '« {{ resetUrl }} » ne l\'est pas', $this->catalog->unknownFieldsIn('Voici {{ resetUrl }}', self::TEMPLATE) === []);

        $io->section('3. Le texte livré, avant toute surcharge');
        [$subjectBefore, $htmlBefore] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        $io->writeln('   objet : ' . $subjectBefore);
        $fingerprintBefore = hash('sha256', $subjectBefore . "\0" . $htmlBefore);

        $io->section('4. On écrit une surcharge — elle doit s\'appliquer');
        $written = $this->overrides->save(self::TEMPLATE, self::LOCALE, 'OBJET RÉÉCRIT', "Bonjour,\nvoici votre lien : {{ resetUrl }}");
        $this->check($io, $failures, 'la surcharge s\'enregistre', $written);

        [$subjectAfter, $htmlAfter] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        $this->check($io, $failures, 'l\'objet est celui de l\'exploitant', $subjectAfter === 'OBJET RÉÉCRIT');
        $this->check($io, $failures, 'le corps aussi', str_contains($htmlAfter, 'voici votre lien'));
        $this->check($io, $failures, 'le champ est REMPLACÉ, pas écrit tel quel', str_contains($htmlAfter, 'https://exemple/reinitialiser') && !str_contains($htmlAfter, '{{ resetUrl }}'));
        // 🔴 Le chrome du layout n'appartient pas à l'exploitant : il porte des
        // obligations (se désinscrire) qu'il ne doit pas pouvoir retirer.
        $this->check($io, $failures, 'le chrome du layout est conservé', str_contains($htmlAfter, '<!DOCTYPE html>'));

        $io->section('5. 🔴 Une surcharge ne peut pas contenir de code');
        $this->overrides->save(self::TEMPLATE, self::LOCALE, 'X', '{{ 7 * 7 }} et <script>alert(1)</script>');
        [, $htmlEvil] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        $this->check($io, $failures, 'une expression Twig n\'est pas évaluée', !str_contains($htmlEvil, '49'));
        $this->check($io, $failures, 'une balise script est échappée, pas exécutable', !str_contains($htmlEvil, '<script>alert(1)</script>'));

        $io->section('6. On retire la surcharge — le texte livré revient à l\'identique');
        $removed = $this->overrides->save(self::TEMPLATE, self::LOCALE, '', '');
        $this->check($io, $failures, 'vider les deux champs supprime la ligne', $removed && $this->overrides->find(self::TEMPLATE, self::LOCALE) === null);

        [$subjectBack, $htmlBack] = $this->sender->render(self::TEMPLATE, $context, self::LOCALE);
        $this->check($io, $failures, 'le rendu est identique au bit près', hash('sha256', $subjectBack . "\0" . $htmlBack) === $fingerprintBefore);
        $this->check($io, $failures, 'la table est rendue vide', $this->overrides->existingKeys() === []);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S161 verte. Aucun courrier envoyé, table rendue vide.');

        return Command::SUCCESS;
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
