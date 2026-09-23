<?php

namespace App\Command;

use App\Search\SiteSearch;
use App\Security\ConsoleRenderAuthenticator;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\HttpKernelInterface;
use Symfony\Component\HttpKernel\KernelInterface;
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;

/**
 * S194 — « un résultat ouvre toujours la fiche de son objet », mesuré sur TOUT
 * ce que la recherche sait trouver.
 *
 * La sonde cherche chaque voyelle (et quelques chiffres) : c'est ce qui ramène
 * presque tous les enregistrements de toutes les familles sans avoir à connaître
 * leurs tables. Puis elle OUVRE chaque résultat distinct, en admin, et exige :
 *   - une réponse 200 ;
 *   - une FICHE, pas une liste filtrée (`?q=`) ni une ancre dans une liste.
 * Les créations n'ont pas encore de fiche (S195) : comptées à part, pas en échec.
 * Lecture seule.
 */
#[AsCommand(name: 'app:s194:search-probe', description: 'S194 : chaque résultat de recherche (toutes familles) ouvre une fiche qui répond 200 ; la page de résultats saute par type. Lecture seule.')]
final class S194SearchProbeCommand extends Command
{
    public function __construct(
        private readonly SiteSearch $search,
        private readonly KernelInterface $kernel,
        private readonly ConsoleRenderAuthenticator $renderAs,
        private readonly TokenStorageInterface $tokens,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $byGroup = [];
        foreach (['a', 'e', 'i', 'o', 'u', 'y', '1', '2', '3'] as $q) {
            foreach ($this->search->groups($q, true) as $group) {
                if ($group['key'] === 'destinations') {
                    continue;
                }
                foreach ($group['items'] as $item) {
                    $byGroup[$group['key']][$item['url']] = $item['title'];
                }
            }
        }

        $io->section('1. Chaque résultat ouvre une FICHE qui répond');
        $lists = $broken = $pending = [];
        $total = 0;
        foreach ($byGroup as $key => $urls) {
            $ok = 0;
            foreach ($urls as $url => $title) {
                ++$total;
                if ($key === 'creations') {
                    $pending[] = $title;
                    continue;
                }
                if (str_contains($url, '?q=') || preg_match('~^/[a-z-]+/?#~', $url)) {
                    $lists[] = "$key : $title → $url";
                    continue;
                }
                $status = $this->status(strtok($url, '#'));
                if ($status !== 200) {
                    $broken[] = "$key : $title → $url ($status)";
                    continue;
                }
                ++$ok;
            }
            $io->writeln(sprintf('   %-12s %3d résultat(s), %3d fiche(s) qui répondent', $key, \count($urls), $ok));
        }
        $this->check($io, $failures, sprintf('%d résultats distincts ouverts', $total), $total > 0);
        $this->check($io, $failures, '🔴 aucun résultat ne mène à une LISTE (catalogue filtré, ancre)', $lists === []);
        foreach (array_slice($lists, 0, 5) as $line) {
            $io->writeln('   ' . $line);
        }
        $this->check($io, $failures, 'aucune fiche en erreur', $broken === []);
        foreach (array_slice($broken, 0, 5) as $line) {
            $io->writeln('   ' . $line);
        }
        if ($pending !== []) {
            $io->writeln(sprintf('   ⏳ %d création(s) : pas encore de fiche — S195', \count($pending)));
        }

        $io->section('2. La page de résultats saute par type');
        $page = $this->get('/recherche?q=a');
        $jumps = preg_match_all('~href="#results-([a-z_]+)"~', $page, $m);
        $anchors = preg_match_all('~id="results-([a-z_]+)"~', $page);
        $this->check($io, $failures, sprintf('%d saut(s), chacun vers un groupe présent', $jumps), $jumps > 1 && $jumps === $anchors);
        $this->check($io, $failures, '« / » est branché sur la recherche de l\'en-tête', str_contains($page, 'data-controller="search-shortcut"') && str_contains($page, 'aria-keyshortcuts="/"'));

        (new \ReflectionProperty(ConsoleRenderAuthenticator::class, 'identifier'))->setValue($this->renderAs, null);
        if ($failures !== []) {
            $io->error(count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S194 verte.');

        return Command::SUCCESS;
    }

    private function status(string $path): int
    {
        return $this->handle($path)->getStatusCode();
    }

    private function get(string $path): string
    {
        return (string) $this->handle($path)->getContent();
    }

    private function handle(string $path): \Symfony\Component\HttpFoundation\Response
    {
        $this->tokens->setToken(null);
        $this->renderAs->renderAs();
        $request = Request::create($path);
        $request->setSession(new Session(new MockArraySessionStorage()));

        return $this->kernel->handle($request, HttpKernelInterface::MAIN_REQUEST, true);
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
