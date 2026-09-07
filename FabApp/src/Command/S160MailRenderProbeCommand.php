<?php

namespace App\Command;

use App\Mail\MailSettings;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\Translation\LocaleSwitcher;
use Twig\Environment;

/**
 * S160 — l'empreinte de TOUS les gabarits d'e-mail, avant et après.
 *
 * 🔴 **C'est la mesure de sortie de la session, mot pour mot** : « sans aucune
 * surcharge, les 23 mails rendent exactement ce qu'ils rendent aujourd'hui —
 * comparaison octet à octet, sinon la phase a déjà cassé quelque chose ».
 *
 * ⚠️ **Le contexte est SYNTHÉTIQUE et c'est ce qui rend la comparaison valide.**
 * Chaque gabarit attend ses propres variables ; il n'existe pas de contexte réel
 * commun. Ce qu'on compare n'est donc pas « le mail qu'un membre recevrait »,
 * c'est **le même gabarit rendu deux fois avec les mêmes entrées**. Une seule
 * différence entre les deux passes signifie que le code de rendu a bougé — ce
 * qui est exactement la question.
 *
 * ⚠️ **La production n'a pas `strict_variables`**, donc une variable absente rend
 * une chaîne vide au lieu de lever. C'est un piège partout ailleurs ; ici c'est
 * ce qui permet de rendre les 20 gabarits avec un contexte unique.
 *
 * ✅ **Elle n'envoie AUCUN courrier** : elle charge le gabarit et rend ses blocs,
 * elle ne touche ni la file ni le transport.
 */
#[AsCommand(name: 'app:s160:mail-render-probe', description: 'S160 : empreinte SHA-256 de chaque gabarit d\'e-mail, pour comparer avant/après. N\'envoie rien.')]
final class S160MailRenderProbeCommand extends Command
{
    public function __construct(
        private readonly Environment $twig,
        private readonly LocaleSwitcher $localeSwitcher,
        private readonly MailSettings $settings,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);

        $names = [];
        foreach (glob(\dirname(__DIR__, 2) . '/templates/emails/*.html.twig') ?: [] as $path) {
            $base = basename($path, '.html.twig');
            if (str_starts_with($base, '_')) {
                continue; // les partiels et le layout ne se rendent pas seuls
            }
            $names[] = $base;
        }
        sort($names);

        /*
         * ⚠️ Un contexte FIXE, écrit ici et jamais tiré de la base : deux passes
         * doivent voir exactement les mêmes entrées, sinon la comparaison mesure
         * les données et pas le code.
         */
        /*
         * 🔴 **Chaque date est ABSOLUE, et c'est une correction du 2026-09-07.**
         * La première version passait `'start' => '10:00'` : les gabarits font
         * `{{ start|date('d/m/Y H:i') }}`, et Twig résout « 10:00 » en
         * AUJOURD'HUI à 10 h. L'empreinte changeait donc tous les jours, et
         * l'instrument censé prouver une égalité au bit près mesurait le
         * calendrier. Mesuré : 17 gabarits sur 20 différaient d'un jour à
         * l'autre, table vide, sans qu'une ligne de code ait bougé.
         *
         * ⚠️ **Une sonde de comparaison qui dépend de l'heure ne prouve rien**,
         * et elle est pire qu'absente : elle donne du vert un jour et du rouge
         * le lendemain, et c'est le vert qu'on croit.
         */
        $context = [
            'sender_name' => $this->settings->getFromName(),
            'event' => 'ÉVÉNEMENT', 'attendee' => 'PERSONNE', 'machine' => 'MACHINE',
            'formation' => 'FORMATION', 'subject' => 'OBJET', 'body' => 'CORPS',
            'author' => 'AUTEUR', 'item' => 'OBJET',
            'date' => new \DateTimeImmutable('2026-01-02 09:00:00'),
            'days' => 3, 'hours' => 24, 'task' => 'TÂCHE', 'resetUrl' => 'https://exemple/x',
            'validHours' => 2, 'place' => 'LIEU',
            'start' => new \DateTimeImmutable('2026-01-02 10:00:00'),
            'end' => new \DateTimeImmutable('2026-01-02 11:00:00'),
            'dueDate' => new \DateTimeImmutable('2026-01-05 12:00:00'),
            'unsubscribe_url' => null,
        ];

        $total = 0;
        foreach (['fr', 'en'] as $locale) {
            $io->section($locale);
            foreach ($names as $name) {
                $hash = $this->localeSwitcher->runWithLocale($locale, function () use ($name, $context): string {
                    $tpl = $this->twig->load('emails/' . $name . '.html.twig');

                    return hash('sha256', $tpl->renderBlock('subject', $context) . "\0" . $tpl->render($context));
                });
                $io->writeln(sprintf('   %-34s %s', $name, substr($hash, 0, 16)));
                ++$total;
            }
        }

        $io->success($total . ' rendus. Aucun courrier envoyé.');

        return Command::SUCCESS;
    }
}
