<?php

namespace App\Command;

use App\Entity\Badge;
use App\Entity\Machine;
use App\Entity\Utilisateur;
use App\Repository\MachineBadgeRepository;
use App\Repository\MachineRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Repository\UtilisateurRepository;
use App\Security\ConsoleRenderAuthenticator;
use App\Service\MachineAccessService;
use App\UsageRights\RightsExplainer;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
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
 * S192 — l'explication dit-elle VRAI ? Quatre mesures, sur toute la base.
 *
 *   1. 🔴 La règle du badge, extraite de `authorize()`, rend les mêmes verdicts
 *      que l'algorithme d'avant — recopié ICI ligne pour ligne — sur chaque paire
 *      (membre, machine).
 *   2. 🔴 « Ce que le badge ouvre » (l'écran) = ce que le lecteur répond (le
 *      scan), machine par machine. Le scan écrit au journal : transaction annulée.
 *   3. Chaque droit ACCORDÉ a au moins un chemin, et ses chemins nomment
 *      exactement les forfaits du verdict — ni plus, ni moins.
 *   4. 🔴 Aucun identifiant de badge entier sur le profil ni sur la fiche admin.
 */
#[AsCommand(name: 'app:s192:rights-probe', description: 'S192 : l\'explication des droits dit vrai — règle du badge identique à l\'ancienne, écran = scan, chemins = verdicts, aucun UID de badge affiché. Transaction annulée.')]
final class S192RightsProbeCommand extends Command
{
    public function __construct(
        private readonly UtilisateurRepository $users,
        private readonly MachineRepository $machines,
        private readonly MachineBadgeRepository $machineBadges,
        private readonly UtilisateurBadgeRepository $userBadges,
        private readonly MachineAccessService $access,
        private readonly RightsExplainer $explainer,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
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
        $users = $this->users->findAll();
        $machines = array_values(array_filter($this->machines->findAll(), static fn (Machine $m): bool => $m->getArchivedAt() === null));
        $logsBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM ACCESS_RFID_LOG');

        $io->section('1. 🔴 La règle extraite = l\'algorithme d\'avant');
        $pairs = 0;
        $disagree = [];
        foreach ($users as $user) {
            $held = $this->held($user);
            foreach ($machines as $machine) {
                $required = $this->required($machine);
                ++$pairs;
                $old = self::legacyRule($required, $held);
                $new = MachineAccessService::badgeRule($required, $held)['status'];
                if ($old !== $new) {
                    $disagree[] = sprintf('#%d × %s : avant %s, après %s', $user->getId(), $machine->getNom(), $old, $new);
                }
            }
        }
        // Plus des cas fabriqués : la base n'a peut-être pas tous les cas limites.
        $a = $this->fakeBadge(1);
        $b = $this->fakeBadge(2);
        $c = $this->fakeBadge(3);
        foreach ([[[], []], [[], [$a]], [[$a], []], [[$a], [$a]], [[$a, $b], [$b]], [[$a, $b], [$c]], [[$a], [$b, $c]], [[$a, $b], [$a, $b, $c]]] as [$required, $held]) {
            ++$pairs;
            if (self::legacyRule($required, $held) !== MachineAccessService::badgeRule($required, $held)['status']) {
                $disagree[] = 'cas fabriqué ' . json_encode([count($required), count($held)]);
            }
        }
        $io->writeln(sprintf('   %d paires comparées', $pairs));
        $this->check($io, $failures, 'aucun désaccord', $disagree === []);
        foreach (array_slice($disagree, 0, 5) as $line) {
            $io->writeln('   ' . $line);
        }

        $io->section('2. 🔴 L\'écran « ce que le badge ouvre » = le scan');
        $mismatch = [];
        $scans = 0;
        $this->db->beginTransaction();
        try {
            foreach ($users as $user) {
                $rfid = trim((string) $user->getIdentifiantRfid());
                if ($rfid === '') {
                    // Un badge de sonde, le temps de la transaction.
                    $rfid = 'SONDE-S192-' . $user->getId() . '-' . bin2hex(random_bytes(3));
                    $this->db->executeStatement('UPDATE UTILISATEUR SET identifiantRfid = ? WHERE id = ?', [$rfid, $user->getId()]);
                }
                $screen = [];
                foreach ($this->access->reachFor($user) as $row) {
                    $screen[$row['machine']->getId()] = $row['status'];
                }
                foreach ($machines as $machine) {
                    if ((string) $machine->getMachineToken() === '') {
                        continue;
                    }
                    ++$scans;
                    $scan = (string) ($this->access->authorize((string) $machine->getMachineToken(), $rfid)['status'] ?? '');
                    if (($screen[$machine->getId()] ?? '—') !== $scan) {
                        $mismatch[] = sprintf('#%d × %s : écran %s, scan %s', $user->getId(), $machine->getNom(), $screen[$machine->getId()] ?? '—', $scan);
                    }
                }
            }
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
        }
        $io->writeln(sprintf('   %d scans comparés', $scans));
        $this->check($io, $failures, 'l\'écran et le lecteur disent la même chose', $mismatch === [] && $scans > 0);
        foreach (array_slice($mismatch, 0, 5) as $line) {
            $io->writeln('   ' . $line);
        }
        $this->check($io, $failures, 'le journal d\'accès est revenu à l\'identique', (int) $this->db->fetchOne('SELECT COUNT(*) FROM ACCESS_RFID_LOG') === $logsBefore);

        $io->section('3. Chaque droit accordé a un chemin, et seulement les siens');
        $granted = 0;
        $bad = [];
        $heldCount = $directCount = 0;
        foreach ($this->users->findAll() as $user) {
            $explained = $this->explainer->explain($user);
            foreach ($explained['badges'] as $held) {
                ++$heldCount;
                $directCount += $held['direct'] ? 1 : 0;
            }
            foreach ($explained['capabilities'] as $row) {
                if ($row['verdict']->reason !== 'granted') {
                    continue;
                }
                ++$granted;
                $fromPaths = array_values(array_unique(array_map(static fn (array $p): string => $p['package'], $row['paths'])));
                $fromVerdict = array_values(array_unique($row['verdict']->packages));
                sort($fromPaths);
                sort($fromVerdict);
                if ($fromPaths === [] || $fromPaths !== $fromVerdict) {
                    $bad[] = sprintf('#%d %s : chemins [%s], verdict [%s]', $user->getId(), $row['capability']->key, implode(', ', $fromPaths), implode(', ', $fromVerdict));
                }
                foreach ($row['paths'] as $path) {
                    if ($path['until'] !== null && strtotime($path['until']) === false) {
                        $bad[] = sprintf('#%d %s : échéance illisible « %s »', $user->getId(), $row['capability']->key, $path['until']);
                    }
                }
            }
        }
        $io->writeln(sprintf('   %d droits accordés expliqués', $granted));
        // Une MESURE, pas une assertion : le lecteur ouvre à tout badge détenu,
        // « Mes badges » ne montre que ceux dont la formation est validée.
        $io->writeln(sprintf('   %d badge(s) détenu(s), dont %d attribué(s) hors formation validée — ouvrent au lecteur, invisibles dans « Mes badges »', $heldCount, $directCount));
        $this->check($io, $failures, 'chemins = forfaits du verdict, partout', $bad === [] && $granted > 0);
        foreach (array_slice($bad, 0, 5) as $line) {
            $io->writeln('   ' . $line);
        }

        $io->section('4. 🔴 Aucun identifiant de badge entier à l\'écran');
        $pages = 0;
        $leaks = [];
        foreach ($this->users->findAll() as $user) {
            $rfid = trim((string) $user->getIdentifiantRfid());
            if (mb_strlen($rfid) <= 4) {
                continue;
            }
            foreach ([['/profil', $user->getEmail()], ['/admin/utilisateurs/' . $user->getId(), null]] as [$path, $as]) {
                $html = $this->render($path, $as);
                ++$pages;
                if (str_contains($html, $rfid)) {
                    $leaks[] = $path . ' (compte #' . $user->getId() . ')';
                }
                if (!str_contains($html, RightsExplainer::maskRfid($rfid))) {
                    $leaks[] = $path . ' : le badge masqué n\'apparaît pas — page rendue ? (compte #' . $user->getId() . ')';
                }
            }
        }
        $io->writeln(sprintf('   %d pages rendues', $pages));
        $this->check($io, $failures, 'aucun UID entier, et le badge masqué est bien là', $leaks === [] && $pages > 0);
        foreach (array_slice($leaks, 0, 5) as $line) {
            $io->writeln('   ' . $line);
        }

        if ($failures !== []) {
            $io->error(count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S192 verte.');

        return Command::SUCCESS;
    }

    /**
     * L'algorithme d'AVANT l'extraction, recopié tel quel de `authorize()`.
     *
     * @param list<Badge> $required
     * @param list<Badge> $held
     */
    private static function legacyRule(array $required, array $held): string
    {
        $matchedBadges = [];
        foreach ($required as $requiredBadge) {
            foreach ($held as $userBadge) {
                if ($requiredBadge->getId() !== null && $requiredBadge->getId() === $userBadge->getId()) {
                    $matchedBadges[] = $requiredBadge->getNom();
                    break;
                }
            }
        }
        if ($required === []) {
            return 'no_badge_required';
        }

        return $matchedBadges !== [] ? 'authorized' : 'missing_badge';
    }

    /** @return list<Badge> */
    private function required(Machine $machine): array
    {
        return array_values(array_filter(array_map(static fn ($mb) => $mb->getBadge(), $this->machineBadges->findRequiredForMachine($machine))));
    }

    /** @return list<Badge> */
    private function held(Utilisateur $user): array
    {
        return array_values(array_filter(array_map(static fn ($ub) => $ub->getBadge(), $this->userBadges->findBy(['utilisateur' => $user]))));
    }

    private function fakeBadge(int $id): Badge
    {
        $badge = (new Badge())->setNom('sonde-' . $id);
        (new \ReflectionProperty(Badge::class, 'id'))->setValue($badge, 900000 + $id);

        return $badge;
    }

    private function render(string $path, ?string $as): string
    {
        $this->tokens->setToken(null);
        $this->renderAs->renderAs($as);
        $request = Request::create($path);
        $request->setSession(new Session(new MockArraySessionStorage()));

        return (string) $this->kernel->handle($request, HttpKernelInterface::MAIN_REQUEST, true)->getContent();
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
