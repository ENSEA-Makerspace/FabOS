<?php

namespace App\Command;

use App\Service\ThemeManager;
use App\Theme\ContrastGate;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S166 — la sonde du contraste.
 *
 * 🔴 **La mesure de sortie, mot pour mot** : « le contraste est MESURÉ, pas
 * affirmé — une palette qui échoue est refusée, pas signalée ». La sonde vérifie
 * donc les deux moitiés : que les nombres sont justes, et que le refus a bien
 * lieu au point de passage.
 *
 * ⚠️ **Les nombres attendus sont écrits en dur ici, à la main.** Comparer la
 * sortie du code à elle-même ne prouverait rien ; ces valeurs viennent de la
 * formule WCAG appliquée séparément, et c'est ce qui rend la sonde capable de
 * détecter une erreur de luminance.
 *
 * ✅ **Elle n'écrit RIEN** : ni brouillon, ni réglage. `check()` est pur, et le
 * refus se mesure en appelant `saveDraft()` dans un `try` — sur une couleur
 * refusée, donc sans écriture possible.
 */
#[AsCommand(name: 'app:s166:contrast-probe', description: 'S166 : prouve que le contraste est calculé juste, que le second contrôle n\'est pas redondant, et qu\'une palette illisible est REFUSÉE. N\'écrit rien.')]
final class S166ContrastProbeCommand extends Command
{
    /**
     * ⚠️ Calculées à part, à la main, depuis la formule WCAG — pas reprises de la
     * sortie du code. Tolérance 0,02 : l'éclaircissement arrondit un canal.
     *
     * @var array<string, array{0: float, 1: float, 2: bool}> couleur => [primaire, sombre, accepté]
     */
    private const EXPECTED = [
        '#9E1B56' => [7.65, 5.12, true],
        '#0044CC' => [7.78, 5.25, true],
        '#1976d2' => [4.60, 6.63, true],
        '#000000' => [21.00, 3.39, false],
        '#FFD400' => [1.43, 11.05, false],
        '#4caf50' => [2.78, 8.23, false],
    ];

    public function __construct(
        private readonly ContrastGate $gate,
        private readonly ThemeManager $themes,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $io->section('1. Les nombres sont JUSTES — comparés à un calcul fait à part');
        foreach (self::EXPECTED as $hex => [$primary, $dark, $accepted]) {
            $verdict = $this->gate->check($hex);
            $got = array_column($verdict['checks'], 'ratio', 'key');
            $io->writeln(sprintf(
                '   %-8s primaire %5.2f  sombre %5.2f  → %s',
                $hex,
                $got['primary'],
                $got['dark'],
                $verdict['ok'] ? 'accepté' : 'REFUSÉ',
            ));
            $this->check($io, $failures, $hex . ' — contraste du primaire', abs($got['primary'] - $primary) < 0.02);
            $this->check($io, $failures, $hex . ' — contraste en thème sombre', abs($got['dark'] - $dark) < 0.02);
            $this->check($io, $failures, $hex . ' — verdict', $verdict['ok'] === $accepted);
        }

        $io->section('2. 🔴 Le second contrôle n\'est PAS redondant');
        // Sans lui, « noir » serait accepté comme couleur d'accent et casserait
        // le thème sombre de tout le site.
        $black = $this->gate->check('#000000');
        $byKey = array_column($black['checks'], null, 'key');
        $this->check($io, $failures, 'le noir PASSE le premier contrôle (21:1)', $byKey['primary']['ok']);
        $this->check($io, $failures, '🔴 et ÉCHOUE le second (3,39:1)', !$byKey['dark']['ok']);
        $this->check($io, $failures, 'donc il est refusé', !$black['ok']);

        $io->section('3. ⚠️ Le contraste est SYMÉTRIQUE — deux contrôles, pas trois');
        // Les lister séparément aurait affiché deux lignes toujours identiques.
        $this->check($io, $failures, 'blanc/couleur et couleur/blanc donnent le MÊME nombre',
            abs(ContrastGate::ratio('#ffffff', '#9E1B56') - ContrastGate::ratio('#9E1B56', '#ffffff')) < 0.0001);

        $io->section('4. La forme courte est DÉVELOPPÉE avant mesure');
        // Mesurer `#abc` octet par octet donnerait une luminance fausse.
        $this->check($io, $failures, '#abc vaut #aabbcc', ContrastGate::normalise('#abc') === '#aabbcc');
        $this->check($io, $failures, 'et son contraste est celui de #aabbcc',
            abs(ContrastGate::ratio('#abc', '#fff') - ContrastGate::ratio('#aabbcc', '#ffffff')) < 0.0001);
        $this->check($io, $failures, 'une valeur qui n\'est pas une couleur ne passe pas', ContrastGate::normalise('rouge') === null);

        $io->section('5. 🔴 Une palette illisible est REFUSÉE, pas signalée');
        /*
         * 🔴 **Le brouillon est capturé AVANT et remis APRÈS, quoi qu'il arrive.**
         * La sonde compte sur le refus pour ne rien écrire — mais si la garde
         * venait à laisser passer, l'appel écraserait le thème de l'opérateur
         * avec « Sonde ». Une sonde ne doit pas dépendre de ce qu'elle mesure
         * pour être inoffensive.
         */
        $before = $this->themes->draft();
        $refused = null;
        try {
            // ⚠️ Le point de passage, pas le formulaire : un import ou une
            // commande doit buter sur la même règle.
            $this->themes->saveDraft(['orgName' => 'Sonde', 'venueLabel' => 'Sonde', 'primaryColor' => '#4caf50', 'logoPath' => '']);
        } catch (\InvalidArgumentException $e) {
            $refused = $e->getMessage();
        } finally {
            $this->themes->saveDraft($before);
        }

        $this->check($io, $failures, '🔴 `saveDraft()` REFUSE', $refused !== null);
        if ($refused !== null) {
            $io->writeln('   « ' . $refused . ' »');
            // ⚠️ Le message porte les NOMBRES : « contraste insuffisant » n'aide
            // personne à choisir la couleur suivante.
            $this->check($io, $failures, 'et le message porte le nombre MESURÉ', str_contains($refused, '2,78'));
            $this->check($io, $failures, 'et le seuil à atteindre', str_contains($refused, '4,5'));
        }

        $io->section('6. Le brouillon n\'a pas bougé');
        // Le refus est une exception AVANT l'écriture : rien n'a pu être écrit.
        $io->writeln('   couleur du brouillon : ' . ($this->themes->draft()['primaryColor'] ?: '(vide — couleur du produit)'));
        $this->check($io, $failures, 'la couleur refusée n\'est PAS en base', $this->themes->draft()['primaryColor'] !== '#4caf50');
        $this->check($io, $failures, 'et le brouillon est identique à l\'état de départ', $this->themes->draft() == $before);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S166 verte. Rien écrit.');

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
