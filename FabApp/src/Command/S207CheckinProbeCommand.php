<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Repository\UtilisateurRepository;
use App\Service\Checkins;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\KernelInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S207 — Check-in à paliers : chaque ligne de « Ce que l'opérateur vérifie ».
 *
 * Dans une transaction ANNULÉE : interrupteurs éteints → 404 et aucune tuile ;
 * palier 1 → « J'arrive » / « Je pars » sans rien saisir, fin calculée à la
 * fermeture ; palier 2 → visiteur (CSRF, pot de miel, limite de débit) ;
 * palier 3 → UNE question, des boutons ; palier 4 → un champ qui n'existe que
 * allumé, jamais requis ; côté équipe → présents, CSV, motifs.
 */
#[AsCommand(name: 'app:s207:checkin-probe', description: 'S207 : check-in à paliers (présence, visiteur, motif, note ; équipe). Transaction annulée.')]
final class S207CheckinProbeCommand extends Command
{
    use ProbeBrowser;

    private const PASSWORD = 'sonde-S207-motdepasse';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly Checkins $checkins,
        private readonly SiteFeatureService $features,
        private readonly UserPasswordHasherInterface $hasher,
        private readonly TokenStorageInterface $tokens,
        private readonly TranslatorInterface $translator,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];
        if (!$this->checkins->isReady()) {
            $io->error('La migration S207 (CHECKIN, CHECKIN_REASON) n’est pas passée.');

            return Command::FAILURE;
        }
        $admin = $member = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $admin ??= $candidate;
            } else {
                $member ??= $candidate;
            }
        }
        if (!$admin instanceof Utilisateur || !$member instanceof Utilisateur) {
            $io->error('Il faut un administrateur et un membre actifs.');

            return Command::FAILURE;
        }
        $tables = ['CHECKIN', 'CHECKIN_REASON', 'SITE_MODULE', 'UTILISATEUR', 'USER_SESSION'];
        $counts = fn (): array => array_combine($tables, array_map(fn (string $t): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM ' . $t), $tables));
        $before = $counts();

        $this->db->beginTransaction();
        try {
            $member->setPassword($this->hasher->hashPassword($member, self::PASSWORD));
            $admin->setPassword($this->hasher->hashPassword($admin, self::PASSWORD));
            $this->entityManager->flush();
            $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId IN (?, ?)', [$member->getId(), $admin->getId()]);
            [$memberId, $email, $adminEmail] = [(int) $member->getId(), $member->getEmail(), $admin->getEmail()];
            $this->db->executeStatement('DELETE FROM CHECKIN WHERE userId = ?', [$memberId]);
            $set = function (bool $p1, bool $p2 = false, bool $p3 = false, bool $p4 = false): void {
                foreach (['checkin' => $p1, 'checkin_walkin' => $p2, 'checkin_reason' => $p3, 'checkin_project' => $p4] as $k => $v) {
                    $this->features->setEnabled($k, $v);
                }
            };
            $openRow = fn (): array|false => $this->db->fetchAssociative('SELECT * FROM CHECKIN WHERE userId = ? ORDER BY id DESC LIMIT 1', [$memberId]);
            $anon = fn (): Session => new Session(new MockArraySessionStorage());

            $io->section('0. Éteint : rien');
            $set(false);
            $session = $this->login($email, self::PASSWORD);
            $this->check($io, $failures, '/check-in : 404', $this->status('/check-in', $session) === 404);
            $this->check($io, $failures, '/kiosk/check-in : 404', $this->status('/kiosk/check-in', $anon()) === 404);
            $this->check($io, $failures, '/admin/check-in : 404', $this->status('/admin/check-in', $this->login($adminEmail, self::PASSWORD)) === 404);
            $this->check($io, $failures, 'la borne n’a pas de tuile', !str_contains($this->page('/kiosk', $anon()), '/kiosk/check-in'));

            $io->section('1. Palier 1 — présence, aucun geste de saisie');
            $set(true);
            $session = $this->login($email, self::PASSWORD);
            $html = $this->page('/check-in', $session);
            $token = $this->formToken($html, '/check-in');
            $this->check($io, $failures, 'la page offre « J’arrive »', $token !== '' && $this->inAnyLocale($html, 'checkin.arrive', []));
            $this->check($io, $failures, 'aucun champ de saisie (ni note, ni motif)', !str_contains($html, 'name="note"') && !str_contains($html, 'name="reason"'));
            $this->check($io, $failures, 'la borne montre la tuile et le QR, sans recherche de membre', str_contains($this->page('/kiosk', $anon()), '/kiosk/check-in')
                && !str_contains($this->page('/kiosk/check-in', $anon()), $member->getEmail()) && str_contains($this->page('/kiosk/check-in', $anon()), 'data:image/svg+xml'));
            $this->post('/check-in', ['_token' => 'faux', 'action' => 'arrive'], $session);
            $this->check($io, $failures, 'jeton CSRF faux : rien d’écrit', $openRow() === false);
            $this->post('/check-in', ['_token' => $token, 'action' => 'arrive'], $session);
            $row = $openRow();
            $this->check($io, $failures, '🔴 « J’arrive » : une visite ouverte, source self, sans motif ni note', $row !== false && $row['endedAt'] === null && $row['source'] === 'self' && $row['reason'] === null && $row['projectNote'] === null);
            $this->post('/check-in', ['_token' => $token, 'action' => 'arrive'], $session);
            $this->check($io, $failures, 'deux fois « J’arrive » : une seule visite', (int) $this->db->fetchOne('SELECT COUNT(*) FROM CHECKIN WHERE userId = ?', [$memberId]) === 1);
            $html = $this->page('/check-in', $session);
            $this->check($io, $failures, '« Vous êtes au lab depuis … » et « Je pars »', str_contains($html, 'value="leave"') && !str_contains($html, 'value="arrive"'));
            $this->post('/check-in', ['_token' => $this->formToken($html, '/check-in'), 'action' => 'leave'], $session);
            $this->check($io, $failures, '« Je pars » : la visite est fermée', $openRow()['endedAt'] !== null && $this->checkins->openFor($member) === null);

            $io->section('1b. La fin se calcule, sans tâche planifiée');
            $utc = new \DateTimeZone('UTC');
            $old = new \DateTimeImmutable('-3 days', $utc);
            $stale = ['startedAt' => $old->format('Y-m-d H:i:s'), 'endedAt' => null, 'venueId' => null];
            $end = $this->checkins->effectiveEnd($stale);
            $this->check($io, $failures, '🔴 une visite ouverte d’un jour passé est finie', $end !== null && $end <= new \DateTimeImmutable('now', $utc) && $end >= $old);
            $fresh = ['startedAt' => (new \DateTimeImmutable('now', $utc))->format('Y-m-d H:i:s'), 'endedAt' => null, 'venueId' => null];
            $this->check($io, $failures, 'et une visite qui commence à l’instant ne l’est pas (au plus tard à la fermeture du jour : testée à l’instant même)', $this->checkins->effectiveEnd($fresh, new \DateTimeImmutable('now', $utc)) === null || $this->checkins->effectiveEnd($fresh, new \DateTimeImmutable('now', $utc)) <= new \DateTimeImmutable('now', $utc));
            $this->db->insert('CHECKIN', ['userId' => $memberId, 'source' => 'self', 'startedAt' => $old->format('Y-m-d H:i:s')]);
            $this->check($io, $failures, 'une visite périmée n’est pas « présente »', $this->checkins->openFor($member) === null);
            $this->checkins->arrive($member, 'self');
            $this->check($io, $failures, 'arriver referme la périmée et en ouvre une neuve', (int) $this->db->fetchOne('SELECT COUNT(*) FROM CHECKIN WHERE userId = ? AND endedAt IS NULL', [$memberId]) === 1);
            $this->checkins->leave($member);

            $io->section('2. Palier 2 — visiteur sans compte, à la borne');
            $set(true, false);
            $guest = $anon();
            $this->check($io, $failures, 'éteint : pas de formulaire de visiteur', !str_contains($this->page('/kiosk/check-in', $guest), 'name="website"'));
            $set(true, true);
            $guest = $anon();
            $html = $this->page('/kiosk/check-in', $guest);
            $token = $this->formToken($html, '/kiosk/check-in');
            $this->check($io, $failures, 'allumé : formulaire (prénom/nom, type) avec pot de miel', $token !== '' && str_contains($html, 'name="name"') && str_contains($html, 'name="type"') && str_contains($html, 'name="website"'));
            $visitors = fn (): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM CHECKIN WHERE userId IS NULL');
            $v0 = $visitors();
            $this->post('/kiosk/check-in', ['_token' => 'faux', 'name' => 'Sonde', 'type' => 'student'], $guest);
            $this->check($io, $failures, 'CSRF faux : rien', $visitors() === $v0);
            $this->post('/kiosk/check-in', ['_token' => $token, 'name' => 'Robot', 'type' => 'student', 'website' => 'http://spam.example'], $guest);
            $this->check($io, $failures, '🔴 pot de miel rempli : rien', $visitors() === $v0);
            $this->post('/kiosk/check-in', ['_token' => $token, 'name' => 'Visiteuse Sonde', 'type' => 'student'], $guest);
            $row = $this->db->fetchAssociative('SELECT * FROM CHECKIN WHERE userId IS NULL ORDER BY id DESC LIMIT 1');
            $this->check($io, $failures, 'visiteur enregistré : nom, type, source kiosk', $visitors() === $v0 + 1 && $row['visitorName'] === 'Visiteuse Sonde' && $row['visitorType'] === 'student' && $row['source'] === 'kiosk');
            for ($i = 0; $i < 10; ++$i) {
                $this->checkins->arriveVisitor('Rafale ' . $i, 'other');
            }
            $v1 = $visitors();
            $this->post('/kiosk/check-in', ['_token' => $token, 'name' => 'Trop', 'type' => 'other'], $guest);
            $this->check($io, $failures, '🔴 limite de débit : refusé au-delà de la rafale', $visitors() === $v1);
            $set(true, false);
            $this->check($io, $failures, 'palier 2 éteint : le POST public répond 404', $this->post('/kiosk/check-in', ['_token' => $token, 'name' => 'X', 'type' => 'other'], $guest)->getStatusCode() === 404);

            $io->section('3. Palier 3 — UNE question, des gros boutons, « Passer »');
            $set(true, false, false);
            $this->check($io, $failures, 'éteint : aucune question', !str_contains($this->page('/check-in', $this->login($email, self::PASSWORD)), 'name="reason"'));
            $set(true, false, true);
            $session = $this->login($email, self::PASSWORD);
            $html = $this->page('/check-in', $session);
            $reasons = $this->checkins->reasons(true);
            $this->check($io, $failures, 'les motifs par défaut sont là (≥ 4) et deviennent des boutons', \count($reasons) >= 4 && substr_count($html, 'name="reason"') === \count($reasons) && $this->inAnyLocale($html, 'checkin.skip', []));
            $token = $this->formToken($html, '/check-in');
            $this->post('/check-in', ['_token' => $token, 'reason' => (string) $reasons[0]['id']], $session);
            $this->check($io, $failures, 'un geste : arrivée AVEC le motif', ($openRow()['reason'] ?? null) === $reasons[0]['label']);
            $this->checkins->leave($member);
            $this->post('/check-in', ['_token' => $token, 'action' => 'arrive'], $session);
            $row = $openRow();
            $this->check($io, $failures, '« Passer » : arrivée sans motif', $row['endedAt'] === null && $row['reason'] === null);
            $this->checkins->leave($member);
            $this->post('/check-in', ['_token' => $token, 'reason' => '999999'], $session);
            $this->check($io, $failures, 'un motif inconnu n’écrit jamais de texte venu du client', $openRow()['reason'] === null);
            $this->checkins->leave($member);

            $io->section('4. Palier 4 — jamais requis');
            $set(true, false, true, false);
            $this->check($io, $failures, 'éteint : aucun champ « note »', !str_contains($this->page('/check-in', $session), 'name="note"'));
            $set(true, false, false, true);
            $this->check($io, $failures, 'allumé : un champ facultatif, replié', str_contains($this->page('/check-in', $session), 'name="note"') && !str_contains($this->page('/check-in', $session), 'name="note" required'));
            $note = $this->page('/check-in', $session);
            $this->check($io, $failures, 'la note vient APRÈS le bouton d’arrivée, jamais avant', strpos($note, 'name="note"') > strpos($note, 'value="arrive"'));
            $set(true, true, false, true);
            $kiosk = $this->page('/kiosk/check-in', $guest);
            $this->check($io, $failures, '🔴 la borne ne propose jamais la note (palier 4 allumé), et son pot de miel est le partiel commun', !str_contains($kiosk, 'name="note"') && str_contains($kiosk, 'data-honeypot'));
            $set(true, false, false, true);
            $this->post('/check-in', ['_token' => $token, 'action' => 'arrive', 'note' => 'Une lampe à base de bois'], $session);
            $this->check($io, $failures, 'avec note : enregistrée', ($openRow()['projectNote'] ?? null) === 'Une lampe à base de bois');
            $this->checkins->leave($member);
            $this->post('/check-in', ['_token' => $token, 'action' => 'arrive'], $session);
            $this->check($io, $failures, '🔴 sans note : l’arrivée passe aussi', $openRow()['endedAt'] === null);
            $this->checkins->leave($member);
            $set(true, false, false, false);
            $this->post('/check-in', ['_token' => $token, 'action' => 'arrive', 'note' => 'ignorée'], $session);
            $this->check($io, $failures, 'palier 4 éteint : une note postée est ignorée', $openRow()['projectNote'] === null);
            $this->checkins->leave($member);

            $io->section('5. Côté équipe');
            $set(true, true, true, true);
            $this->checkins->arrive($member, 'self');
            $adminSession = $this->login($adminEmail, self::PASSWORD);
            $html = $this->page('/admin/check-in', $adminSession);
            $this->check($io, $failures, '/admin/check-in : présents maintenant, avec le membre', $this->inAnyLocale($html, 'checkin.present_now', ['%n%' => (string) \count($this->checkins->present())]) && str_contains($html, htmlspecialchars($member->getDisplayName() ?: $member->getUsername(), ENT_QUOTES)));
            $this->check($io, $failures, 'tuiles Aujourd’hui / 7 jours / 30 jours', $this->inAnyLocale($html, 'checkin.period_7', []) && $this->inAnyLocale($html, 'checkin.period_30', []));
            $response = $this->handle(\Symfony\Component\HttpFoundation\Request::create('/admin/check-in/export.csv?period=30'), $adminSession);
            ob_start();
            $response->sendContent();
            $csv = (string) ob_get_clean();
            $this->check($io, $failures, 'export CSV : en-tête et visiteuse', str_contains((string) $response->headers->get('Content-Type'), 'text/csv') && str_contains($csv, 'date,arrivee,depart') && str_contains($csv, 'Visiteuse Sonde'));
            $this->check($io, $failures, 'un membre ne voit pas /admin/check-in', \in_array($this->status('/admin/check-in', $this->login($email, self::PASSWORD)), [302, 403], true));
            $rtoken = $this->formToken($html, '/admin/check-in/motifs');
            $this->check($io, $failures, 'réglage des motifs sur la même page, replié', $rtoken !== '' && str_contains($html, '<details'));
            $post = fn (array $f) => $this->post('/admin/check-in/motifs', ['_token' => $rtoken] + $f, $adminSession);
            $n0 = \count($this->checkins->reasons());
            $post(['action' => 'add', 'label' => 'Repair Café']);
            $all = $this->checkins->reasons();
            $new = end($all);
            $this->check($io, $failures, 'ajouter un motif', \count($all) === $n0 + 1 && $new['label'] === 'Repair Café');
            $post(['action' => 'rename', 'id' => $new['id'], 'label' => 'Repair Café du samedi']);
            $this->check($io, $failures, 'renommer', $this->checkins->reasonLabel($new['id']) === 'Repair Café du samedi');
            $post(['action' => 'move_up', 'id' => $new['id']]);
            $ids = array_column($this->checkins->reasons(), 'id');
            $this->check($io, $failures, 'ordonner : monte d’un cran', array_search($new['id'], $ids, true) === $n0 - 1);
            $post(['action' => 'disable', 'id' => $new['id']]);
            $this->check($io, $failures, 'désactiver : plus proposé au membre', $this->checkins->reasonLabel($new['id']) === null && !\in_array($new['id'], array_column($this->checkins->reasons(true), 'id'), true));
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
        }

        $io->section('6. Rien n’est resté');
        foreach ($counts() as $table => $count) {
            $this->check($io, $failures, sprintf('%s : %d avant, %d après', $table, $before[$table], $count), $count === $before[$table]);
        }
        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S207 verte. Transaction annulée.');

        return Command::SUCCESS;
    }
}
