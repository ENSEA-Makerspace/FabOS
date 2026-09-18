<?php

namespace App\Command;

use App\Service\ThemeManager;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Twig\Environment;

/**
 * S168 — la sonde de l'icône d'onglet.
 *
 * 🔴 **Le défaut mesuré** : `asset('images/favicon.png')` était écrit à la main
 * dans HUIT gabarits, dont les quatre kiosques. Un labo qui posait son logo dans
 * `/admin/themes` gardait l'icône de FabOS dans l'onglet ET sur le mur de son
 * atelier — la moitié la plus visible d'une identité, et la seule que personne ne
 * pensait à changer parce qu'elle n'était proposée nulle part.
 *
 * ⚠️ **Elle compte les émissions dans les SOURCES**, pas dans un rendu. Rendre
 * les huit pages demanderait huit contextes différents — dont un kiosque, qui
 * n'existe que pour un lieu donné. Ce qui est vérifiable partout et sans
 * décor : qu'il ne reste qu'un seul endroit qui écrit cette balise.
 *
 * 🅿️ **La seule exception attendue est `event-ticket`**, délibérément autonome :
 * il ne lit ni la feuille du site ni la médiathèque, parce qu'un billet s'ouvre
 * sur un téléphone avec un mauvais réseau et s'imprime sur ce qui traîne. La
 * sonde l'attend explicitement plutôt que de l'ignorer en silence.
 */
#[AsCommand(name: 'app:s168:favicon-probe', description: 'S168 : prouve qu\'un seul gabarit émet l\'icône d\'onglet, que les kiosques n\'en écrivent plus en dur, et que le thème la porte. N\'écrit rien.')]
final class S168FaviconProbeCommand extends Command
{
    private const ALLOWED = ['_favicon.html.twig', 'event-ticket.html.twig'];

    public function __construct(
        private readonly ThemeManager $themes,
        private readonly Environment $twig,
        private readonly string $templateDir,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $io->section('1. Un SEUL endroit écrit l\'icône');
        $writers = [];
        foreach ($this->twigFiles() as $path) {
            $body = (string) @file_get_contents($path);
            if (preg_match('/<link[^>]+rel="icon"/', $body) === 1) {
                $writers[] = basename($path);
            }
        }
        sort($writers);
        $io->writeln('   ' . implode(', ', $writers));
        $this->check($io, $failures, 'aucun gabarit inattendu n\'écrit `rel="icon"`', array_diff($writers, self::ALLOWED) === []);
        $this->check($io, $failures, 'et le partiel partagé existe bien', in_array('_favicon.html.twig', $writers, true));

        $io->section('2. 🔴 Les quatre kiosques passent par le partiel');
        // C'était le vrai trou : un mur d'atelier affiche l'icône en grand dans
        // l'onglet du navigateur en plein écran, et c'est la première chose que
        // voient les visiteurs du labo.
        foreach (['kiosk-entries', 'kiosk-events', 'kiosk-machine', 'kiosk-stats'] as $kiosk) {
            $body = (string) @file_get_contents($this->templateDir . '/site/' . $kiosk . '.html.twig');
            $this->check($io, $failures, $kiosk . ' inclut `_favicon.html.twig`', str_contains($body, "site/_favicon.html.twig"));
            $this->check($io, $failures, $kiosk . ' n\'écrit plus le chemin en dur', !str_contains($body, "asset('images/favicon.png')"));
        }

        $io->section('3. Le thème PORTE l\'icône');
        $draft = $this->themes->draft();
        $this->check($io, $failures, 'le brouillon a un champ `faviconPath`', array_key_exists('faviconPath', $draft));
        $this->check($io, $failures, 'le publié aussi', array_key_exists('faviconPath', $this->themes->published()));
        $io->writeln('   valeur actuelle : ' . ($draft['faviconPath'] ?: '(vide — icône du produit)'));

        $io->section('4. ⚠️ Une icône choisie est PROTÉGÉE comme le logo');
        // Supprimer de la médiathèque l'image servant d'icône laisserait une
        // icône cassée sur chaque onglet et chaque kiosque.
        $refs = $this->themes->referencedMediaIds();
        $io->writeln('   références du thème : ' . (implode(', ', $refs) ?: '(aucune)'));
        $expected = array_values(array_filter([$draft['faviconPath'], $draft['logoPath']]));
        $this->check($io, $failures, 'les images du brouillon sont bien dans les références', array_diff($expected, $refs) === []);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S168 verte. Rien écrit.');

        return Command::SUCCESS;
    }

    /** @return list<string> */
    private function twigFiles(): array
    {
        $out = [];
        $it = new \RecursiveIteratorIterator(new \RecursiveDirectoryIterator($this->templateDir, \FilesystemIterator::SKIP_DOTS));
        foreach ($it as $file) {
            if ($file instanceof \SplFileInfo && str_ends_with($file->getFilename(), '.html.twig')) {
                $out[] = $file->getPathname();
            }
        }

        return $out;
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
