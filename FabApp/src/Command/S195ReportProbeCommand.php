<?php

namespace App\Command;

use App\Repository\UtilisateurRepository;
use App\Security\ConsoleRenderAuthenticator;
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
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;

/**
 * S195 — le rapport dit VRAI et propose d'agir ; chaque création a sa fiche.
 *
 * Le rapport est recalculé ICI en SQL indépendant (réservations actives,
 * annulations, ressources en service jamais réservées) et comparé à la page :
 * une page qui affiche un joli constat faux serait pire que pas de constat.
 * Le vote depuis une fiche s'éprouve dans une transaction annulée.
 */
#[AsCommand(name: 'app:s195:report-probe', description: 'S195 : le rapport compte les réservations actives, propose ses actions (ressources jamais réservées, annulations) et lie ses ressources ; chaque création publiée a sa fiche. Transaction annulée pour le vote.')]
final class S195ReportProbeCommand extends Command
{
    private const FROM = '2026-07-01';
    private const TO = '2026-12-31';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly ConsoleRenderAuthenticator $renderAs,
        private readonly TokenStorageInterface $tokens,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('from', null, InputOption::VALUE_REQUIRED, 'Début de période (Y-m-d)', self::FROM);
        $this->addOption('to', null, InputOption::VALUE_REQUIRED, 'Fin de période (Y-m-d)', self::TO);
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];
        $from = (string) $input->getOption('from');
        $to = (string) $input->getOption('to');

        $io->section('1. Le rapport « équipement », recalculé à part');
        $window = ['from' => $from . ' 00:00:00', 'until' => (new \DateTimeImmutable($to))->modify('+1 day')->format('Y-m-d 00:00:00')];
        $active = (int) $this->db->fetchOne("SELECT COUNT(*) FROM RESERVATION WHERE reservableType = 'machine' AND dateDebut >= :from AND dateDebut < :until AND statut NOT IN ('cancelled','declined')", $window);
        $cancelled = (int) $this->db->fetchOne("SELECT COUNT(*) FROM RESERVATION WHERE reservableType = 'machine' AND dateDebut >= :from AND dateDebut < :until AND statut = 'cancelled'", $window);
        $all = (int) $this->db->fetchOne("SELECT COUNT(*) FROM RESERVATION WHERE reservableType = 'machine' AND dateDebut >= :from AND dateDebut < :until", $window);
        // ⚠️ Jugé SANS reprendre la règle de l'adaptateur : la première sonde
        // recopiait son `statut = 'active'` et les deux avaient tort ENSEMBLE. Ici :
        // toute machine non archivée, sauf maintenance/panne ÉCRITES EN CLAIR.
        $idle = $this->db->fetchFirstColumn("SELECT m.id FROM MACHINE m WHERE m.archivedAt IS NULL AND LOWER(m.statut) NOT LIKE '%maintenance%' AND LOWER(m.statut) NOT LIKE '%panne%' AND LOWER(m.statut) NOT IN ('hors service','broken','down') AND NOT EXISTS (SELECT 1 FROM RESERVATION r WHERE r.reservableType = 'machine' AND r.reservableId = m.id AND r.dateDebut >= :from AND r.dateDebut < :until AND r.statut NOT IN ('cancelled','declined'))", $window);
        $io->writeln(sprintf('   attendu : %d actives (%d demandes, %d annulées), %d machine(s) jamais réservée(s)', $active, $all, $cancelled, \count($idle)));

        $page = $this->get('/admin/reporting/equipment?from=' . $from . '&to=' . $to, new Session(new MockArraySessionStorage()));
        preg_match_all('~<article><strong>([^<]*)</strong>~', $page, $m);
        $shown = array_map(static fn (string $v): string => trim($v), $m[1]);
        $this->check($io, $failures, sprintf('🔴 le total affiché (%s) = les réservations ACTIVES (%d), plus les annulées', $shown[0] ?? '?', $active), ($shown[0] ?? '') === (string) $active);
        $this->check($io, $failures, sprintf('les annulées restent comptées à part (%s)', $shown[3] ?? '?'), ($shown[3] ?? '') === (string) $cancelled);

        $idleShown = preg_match_all('~<li><a href="/admin/machines/(\d+)/edit">~', $page, $ids) ? array_map('intval', $ids[1]) : [];
        sort($idle);
        sort($idleShown);
        $this->check($io, $failures, 'le constat « jamais réservées » liste EXACTEMENT ces machines, chacune vers sa fiche', array_map('intval', $idle) === $idleShown || (\count($idle) > 12 && \count($idleShown) === 12));
        $rate = $all > 0 ? $cancelled / $all : 0;
        $expectCancelFinding = $all >= 10 && $rate >= 0.2;
        $this->check($io, $failures, sprintf('le constat « annulations » (%d %%) %s, avec son action', (int) round($rate * 100), $expectCancelFinding ? 'apparaît' : 'n\'apparaît pas'), str_contains($page, 'btn-primary-admin" href="/admin/quotas-reservation"') === $expectCancelFinding);
        $topLinked = preg_match_all('~<tr><td><a href="/admin/machines/\d+/edit">~', $page);
        $this->check($io, $failures, sprintf('les ressources du classement ouvrent leur fiche (%d)', $topLinked), $active === 0 || $topLinked > 0);

        $io->section('2. Chaque création publiée a sa fiche');
        $published = $this->db->fetchAllAssociative('SELECT id, title FROM CREATION WHERE isPublished = 1 AND archivedAt IS NULL');
        $ok = 0;
        foreach ($published as $row) {
            $html = $this->get('/creations/' . $row['id'], new Session(new MockArraySessionStorage()), false);
            $ok += str_contains($html, '<h1 class="creation-card__title">') ? 1 : 0;
        }
        $this->check($io, $failures, sprintf('%d/%d fiches s\'ouvrent, le titre en h1', $ok, \count($published)), $ok === \count($published) && $ok > 0);
        $hidden = $this->db->fetchFirstColumn('SELECT id FROM CREATION WHERE isPublished = 0 OR archivedAt IS NOT NULL');
        foreach ($hidden as $id) {
            $this->check($io, $failures, sprintf('création #%d retirée : 404', $id), $this->handle(Request::create('/creations/' . $id), new Session(new MockArraySessionStorage()), false)->getStatusCode() === 404);
        }
        $gallery = $this->get('/creations', new Session(new MockArraySessionStorage()), false);
        $this->check($io, $failures, 'la galerie mène aux fiches (titres, podium)', preg_match_all('~href="/creations/\d+"~', $gallery) >= \count($published));

        $io->section('3. Voter depuis la fiche y ramène');
        $member = $this->users->findOneBy(['statut' => 'actif', 'isVerified' => true]);
        if ($member !== null && $published !== []) {
            $id = (int) $published[0]['id'];
            $this->db->beginTransaction();
            try {
                $session = new Session(new MockArraySessionStorage());
                $html = $this->get('/creations/' . $id, $session, true, $member->getEmail());
                $token = preg_match('~name="_token" value="([^"]+)">\s*<input type="hidden" name="back" value="detail">~', $html, $t) ? $t[1] : '';
                $r = $this->handle(Request::create('/creations/' . $id . '/vote', 'POST', ['_token' => $token, 'rating' => '4', 'back' => 'detail']), $session, true, $member->getEmail());
                $this->check($io, $failures, 'le vote revient sur la fiche', $r->isRedirect() && str_ends_with((string) $r->headers->get('Location'), '/creations/' . $id));
            } finally {
                $this->db->rollBack();
                $this->entityManager->clear();
            }
        }

        $this->tokens->setToken(null);
        (new \ReflectionProperty(ConsoleRenderAuthenticator::class, 'identifier'))->setValue($this->renderAs, null);
        if ($failures !== []) {
            $io->error(count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S195 verte.');

        return Command::SUCCESS;
    }

    private function get(string $path, Session $session, bool $signedIn = true, ?string $as = null): string
    {
        return (string) $this->handle(Request::create($path), $session, $signedIn, $as)->getContent();
    }

    private function handle(Request $request, Session $session, bool $signedIn = true, ?string $as = null): Response
    {
        $this->tokens->setToken(null);
        if ($signedIn) {
            $this->renderAs->renderAs($as);
        } else {
            (new \ReflectionProperty(ConsoleRenderAuthenticator::class, 'identifier'))->setValue($this->renderAs, null);
        }
        $request->setSession($session);
        if ($request->isMethod('POST')) {
            $request->headers->set('Origin', $request->getSchemeAndHttpHost());
        }

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
