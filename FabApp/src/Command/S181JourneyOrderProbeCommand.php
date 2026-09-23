<?php

namespace App\Command;

use App\Entity\Section;
use App\Repository\FormationRepository;
use App\Repository\SectionRepository;
use App\Security\ConsoleRenderAuthenticator;
use App\Training\JourneyOrder;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Input\InputOption;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\HttpKernelInterface;
use Symfony\Component\HttpKernel\KernelInterface;

/**
 * S181 — déplacer une étape du parcours, pour de vrai, puis tout annuler.
 *
 * Les déplacements s'exécutent sur la vraie table, DANS une transaction que la
 * sonde annule. 🔴 Elle relit ensuite `SECTION` et exige l'état de départ à la
 * ligne près : une sonde qui laisserait un parcours réordonné derrière elle
 * changerait ce que voient les apprenants.
 */
#[AsCommand(name: 'app:s181:journey-order-probe', description: 'S181 : déplace des étapes d\'un parcours dans une transaction annulée ; prouve la renumérotation 1..n, les bornes, que les blocs de page ne bougent pas, et que la base revient à l\'identique.')]
final class S181JourneyOrderProbeCommand extends Command
{
    public function __construct(
        private readonly JourneyOrder $order,
        private readonly FormationRepository $formations,
        private readonly SectionRepository $sections,
        private readonly EntityManagerInterface $entityManager,
        private readonly Connection $db,
        private readonly KernelInterface $kernel,
        private readonly ConsoleRenderAuthenticator $renderAs,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('formation', null, InputOption::VALUE_REQUIRED, 'Formation à éprouver', '2');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $formation = $this->formations->find((int) $input->getOption('formation'));
        if ($formation === null) {
            $io->error('Formation introuvable.');

            return Command::FAILURE;
        }

        $snapshot = fn (): array => $this->db->fetchAllAssociative(
            'SELECT id, ordre, titre FROM SECTION WHERE formationId = ? ORDER BY id',
            [$formation->getId()],
        );
        $ids = static fn (array $sections): array => array_map(static fn (Section $s): int => (int) $s->getId(), $sections);

        $before = $snapshot();
        $journey = $ids($this->sections->findJourneySections($formation));
        $blocksBefore = array_map(static fn (Section $s): array => [$s->getId(), $s->getOrdre()], $this->sections->findPageContentBlocks($formation));
        $io->writeln(sprintf('   formation #%d : %d étapes, %d blocs de page', $formation->getId(), count($journey), count($blocksBefore)));
        if (count($journey) < 3) {
            $io->error('Il faut au moins trois étapes pour éprouver le déplacement.');

            return Command::FAILURE;
        }

        $this->db->beginTransaction();
        try {
            $io->section('1. Monter la dernière étape');
            $last = $this->sections->find(end($journey));
            $this->check($io, $failures, 'le déplacement est accepté', $this->order->move($formation, $last, -1));
            $expected = $journey;
            $n = count($expected);
            [$expected[$n - 1], $expected[$n - 2]] = [$expected[$n - 2], $expected[$n - 1]];
            $after = $this->sections->findJourneySections($formation);
            $this->check($io, $failures, 'elle a échangé sa place avec sa voisine, et avec elle seule', $ids($after) === $expected);
            $this->check($io, $failures, 'les numéros valent 1..n, sans trou ni doublon', array_map(static fn (Section $s): int => $s->getOrdre(), $after) === range(1, $n));

            $io->section('2. Les bornes');
            $first = $this->sections->find($expected[0]);
            $this->check($io, $failures, 'monter la première est refusé', !$this->order->move($formation, $first, -1));
            $this->check($io, $failures, 'descendre la dernière est refusé', !$this->order->move($formation, $this->sections->find($expected[$n - 1]), 1));
            $this->check($io, $failures, 'et rien n\'a bougé', $ids($this->sections->findJourneySections($formation)) === $expected);

            $io->section('3. Un aller-retour rend l\'ordre de départ');
            $this->order->move($formation, $this->sections->find($expected[$n - 2]), 1);
            $this->check($io, $failures, 'descendre la même étape la remet à sa place', $ids($this->sections->findJourneySections($formation)) === $journey);

            $io->section('4. Les blocs de la page ne sont jamais touchés');
            $blocksAfter = array_map(static fn (Section $s): array => [$s->getId(), $s->getOrdre()], $this->sections->findPageContentBlocks($formation));
            $this->check($io, $failures, 'mêmes blocs, mêmes numéros', $blocksAfter === $blocksBefore);

            $io->section('5. Une section d\'une AUTRE formation est refusée');
            $other = null;
            foreach ($this->formations->findAll() as $candidate) {
                if ($candidate->getId() !== $formation->getId() && ($found = $this->sections->findJourneySections($candidate)) !== []) {
                    $other = $found[0];
                    break;
                }
            }
            $this->check($io, $failures, 'move() rend faux', $other !== null && !$this->order->move($formation, $other, 1));

            $io->section('6. La ROUTE, comme un navigateur : page, jeton, POST');
            $this->entityManager->clear();
            $this->renderAs->renderAs();
            $session = new Session(new MockArraySessionStorage());
            $page = Request::create('/admin/formations/' . $formation->getId() . '/content?ouvrir=sections');
            $page->setSession($session);
            $html = (string) $this->kernel->handle($page, HttpKernelInterface::MAIN_REQUEST, true)->getContent();
            $target = $journey[1];
            $token = preg_match('#/sections/' . $target . '/move">\\s*<input type="hidden" name="_token" value="([^"]+)"#', $html, $m) ? $m[1] : null;
            $this->check($io, $failures, 'la page porte un formulaire et un jeton par étape', $token !== null && substr_count($html, 'class="admin-step-move"') === count($journey));

            $post = fn (string $tok): Response => $this->kernel->handle(
                (function () use ($formation, $target, $tok, $session): Request {
                    $r = Request::create('/admin/formations/' . $formation->getId() . '/sections/' . $target . '/move', 'POST', ['_token' => $tok, 'direction' => 'up']);
                    $r->setSession($session);

                    return $r;
                })(),
                HttpKernelInterface::MAIN_REQUEST,
                true,
            );

            $forged = $post('faux');
            $this->entityManager->clear();
            $this->check($io, $failures, 'un jeton faux est refusé (' . $forged->getStatusCode() . ')', $forged->getStatusCode() === 403);
            $this->check($io, $failures, 'et rien n\'a bougé', $ids($this->sections->findJourneySections($formation)) === $journey);

            $moved = $post((string) $token);
            $this->entityManager->clear();
            $location = (string) $moved->headers->get('Location');
            $this->check($io, $failures, 'le bon jeton : 303 vers la liste ouverte, ancrée sur l\'étape', $moved->getStatusCode() === 303
                && str_contains($location, 'ouvrir=sections') && str_contains($location, 'deplace=' . $target) && str_ends_with($location, '#section-' . $target));
            $up = $journey;
            [$up[0], $up[1]] = [$up[1], $up[0]];
            $this->check($io, $failures, 'et l\'étape 2 est passée en tête', $ids($this->sections->findJourneySections($formation)) === $up);
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
        }

        $io->section('7. La base est revenue à l\'identique');
        $this->check($io, $failures, 'SECTION de cette formation, ligne pour ligne', $snapshot() === $before);

        if ($failures !== []) {
            $io->error(count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S181 verte. Base rendue à l\'identique.');

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
