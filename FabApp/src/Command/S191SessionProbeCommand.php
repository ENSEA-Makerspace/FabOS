<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use App\Security\ConsoleRenderAuthenticator;
use App\Security\SessionRegistry;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\HttpKernelInterface;
use Symfony\Component\HttpKernel\KernelInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;

/**
 * S191a — les sessions, éprouvées comme plusieurs navigateurs, et tout annulé.
 *
 * ⚠️ Deux régimes. **Sans la table** (migration S191 pas encore passée) : la
 * sonde prouve que RIEN ne casse — connexion, profil, pas de lien vers une page
 * qui n'aurait rien à montrer. **Avec la table** : deux, puis trois appareils ;
 * en fermer un depuis l'autre, « fermer les autres », changer de mot de passe,
 * se déconnecter, et l'admin qui ferme tout.
 *
 * ⚠️ Pièges connus de ces sondes (voir la mémoire) : le cookie de session est
 * posé sur chaque requête, l'état est écrit en SQL, et le jeton est remis à
 * zéro entre deux requêtes.
 */
#[AsCommand(name: 'app:s191:session-probe', description: 'S191a : sessions visibles et fermables — sans la table, rien ne casse ; avec, fermer une session, les autres, changer de mot de passe, déconnexion, admin. Transaction annulée.')]
final class S191SessionProbeCommand extends Command
{
    private const PASSWORD = 'sonde-S191-motdepasse';
    private const NEW_PASSWORD = 'sonde-S191-nouveau-mdp';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly SessionRegistry $registry,
        private readonly UserPasswordHasherInterface $hasher,
        private readonly TokenStorageInterface $tokens,
        private readonly ConsoleRenderAuthenticator $renderAs,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];
        // Un compte NON admin : l'admin du rendu console doit être quelqu'un d'autre.
        $member = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            if (!in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $member = $candidate;
                break;
            }
        }
        if (!$member instanceof Utilisateur) {
            $io->error('Aucun compte actif non admin.');

            return Command::FAILURE;
        }
        $id = (int) $member->getId();
        $email = $member->getEmail();

        $this->db->beginTransaction();
        try {
            $this->db->executeStatement('UPDATE UTILISATEUR SET password = ? WHERE id = ?', [$this->hasher->hashPassword($member, self::PASSWORD), $id]);

            if (!$this->registry->isReady()) {
                $io->section('Table USER_SESSION absente — migration S191 en attente : RIEN ne doit casser');
                $a = $this->login($email, 'Firefox');
                $this->check($io, $failures, 'la connexion marche', $this->status('/profil', $a) === 200);
                $this->check($io, $failures, 'le profil ne propose pas « Sessions ouvertes »', !str_contains($this->get('/profil', $a), '/profil/sessions'));
                $this->check($io, $failures, '/profil/sessions renvoie au profil au lieu de planter', $this->status('/profil/sessions', $a) === 302);
                $io->note('Lancer la migration, redémarrer le service, puis relancer cette sonde pour éprouver les sessions.');
            } else {
                $rowsBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM USER_SESSION');

                $io->section('1. Deux appareils');
                $a = $this->login($email, 'Mozilla/5.0 (Macintosh; Intel Mac OS X 14_0) Gecko/20100101 Firefox/130.0');
                $b = $this->login($email, 'Mozilla/5.0 (Linux; Android 14) AppleWebKit/537.36 Chrome/128.0 Mobile Safari/537.36');
                $page = $this->get('/profil/sessions', $a);
                $this->check($io, $failures, 'A voit deux sessions, dont la sienne marquée', substr_count($page, 'class="sessions-item') === 2 && substr_count($page, 'is-current') === 1);
                $this->check($io, $failures, 'les appareils sont nommés (Firefox · macOS, Chrome · Android)', str_contains($page, 'Firefox · macOS') && str_contains($page, 'Chrome · Android'));
                $this->check($io, $failures, '🔴 aucune clé de session dans la page', !str_contains($page, (string) $a->get(SessionRegistry::SESSION_KEY)));

                $io->section('2. A ferme la session de B');
                $bId = $this->otherSessionId($page);
                $this->post('/profil/sessions/' . $bId . '/fermer', ['_token' => $this->token($page, '/profil/sessions/' . $bId . '/fermer')], $a);
                $cut = $this->handle(Request::create('/profil'), $b);
                $this->check($io, $failures, '🔴 la requête suivante de B renvoie à /login', $cut->isRedirect() && str_ends_with((string) $cut->headers->get('Location'), '/login'));
                $this->check($io, $failures, 'et la connexion dit pourquoi', str_contains($this->get('/login', $b), 'auth-alert-error'));
                $this->check($io, $failures, 'A, elle, continue', $this->status('/profil', $a) === 200);

                $io->section('3. « Fermer toutes les autres »');
                $c = $this->login($email, 'Safari/605 (iPhone)');
                $page = $this->get('/profil/sessions', $a);
                $this->post('/profil/sessions/fermer-les-autres', ['_token' => $this->token($page, '/profil/sessions/fermer-les-autres')], $a);
                $this->check($io, $failures, 'C est coupée', $this->status('/profil', $c) === 302);
                $this->check($io, $failures, 'A continue', $this->status('/profil', $a) === 200);

                $io->section('4. Changer de mot de passe ferme les autres');
                $d = $this->login($email, 'Firefox/130');
                $form = $this->get('/profil/password', $a);
                $pwToken = preg_match('#name="_token" value="([^"]+)"#', $form, $m) ? $m[1] : '';
                $this->post('/profil/password', ['_token' => $pwToken, 'currentPassword' => self::PASSWORD, 'newPassword' => self::NEW_PASSWORD, 'confirmPassword' => self::NEW_PASSWORD], $a);
                $this->check($io, $failures, 'D est coupée', $this->status('/profil', $d) === 302);
                $this->check($io, $failures, 'A continue', $this->status('/profil', $a) === 200);
                $this->check($io, $failures, 'et la liste ne montre plus qu\'elle', substr_count($this->get('/profil/sessions', $a), 'class="sessions-item') === 1);

                $io->section('5. L\'équipe ferme tout');
                $e = $this->login($email, 'Firefox/130', self::NEW_PASSWORD);
                $adminId = (int) $this->users->findOneBy(['email' => $this->renderAs->renderAs()])?->getId();
                $adminRowsBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM USER_SESSION WHERE userId = ?', [$adminId]);
                $admin = new Session(new MockArraySessionStorage());
                $detail = $this->get('/admin/utilisateurs/' . $id, $admin);
                $adminToken = $this->token($detail, '/admin/utilisateurs/' . $id . '/sessions/fermer');
                $this->check($io, $failures, 'la fiche admin propose « Fermer toutes ses sessions »', $adminToken !== '');
                $this->post('/admin/utilisateurs/' . $id . '/sessions/fermer', ['_token' => $adminToken], $admin);
                $this->renderAsNobody();
                $this->check($io, $failures, 'A et E sont coupées', $this->status('/profil', $a) === 302 && $this->status('/profil', $e) === 302);
                $this->check($io, $failures, 'le rendu console n\'a ouvert AUCUNE session suivie pour l\'admin', (int) $this->db->fetchOne('SELECT COUNT(*) FROM USER_SESSION WHERE userId = ?', [$adminId]) === $adminRowsBefore);

                $io->section('6. Se déconnecter ferme la ligne');
                $f = $this->login($email, 'Firefox/130', self::NEW_PASSWORD);
                $this->get('/logout', $f);
                $open = (int) $this->db->fetchOne('SELECT COUNT(*) FROM USER_SESSION WHERE userId = ? AND revokedAt IS NULL', [$id]);
                $this->check($io, $failures, 'aucune session de ce compte ne reste ouverte', $open === 0);
                $ip = (string) $this->db->fetchOne('SELECT ipPrefix FROM USER_SESSION WHERE userId = ? ORDER BY id DESC LIMIT 1', [$id]);
                $this->check($io, $failures, 'l\'IP est tronquée (' . $ip . ')', str_ends_with($ip, '.0/24') || str_ends_with($ip, '/48'));
            }
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
            $this->renderAsNobody();
        }

        if ($this->registry->isReady()) {
            $this->check($io, $failures, 'USER_SESSION revenue à son compte de départ', (int) $this->db->fetchOne('SELECT COUNT(*) FROM USER_SESSION') === ($rowsBefore ?? -1));
        }
        $this->check($io, $failures, 'le mot de passe du compte est intact', $this->users->find($id)?->getPassword() === $member->getPassword());

        if ($failures !== []) {
            $io->error(count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S191a verte.');

        return Command::SUCCESS;
    }

    private function login(string $email, string $agent, string $password = self::PASSWORD): Session
    {
        $session = new Session(new MockArraySessionStorage());
        $session->set('_probe_agent', $agent);
        $form = $this->get('/login', $session);
        $token = preg_match('#name="_csrf_token"\s+value="([^"]+)"#', $form, $m) ? $m[1] : '';
        $this->post('/login', ['_username' => $email, '_password' => $password, '_csrf_token' => $token], $session);

        return $session;
    }

    private function otherSessionId(string $page): int
    {
        preg_match_all('#<li class="sessions-item( is-current)?">.*?/profil/sessions/(\d+)/fermer#s', $page, $m, PREG_SET_ORDER);
        foreach ($m as $row) {
            if ($row[1] === '') {
                return (int) $row[2];
            }
        }

        return 0;
    }

    private function token(string $page, string $action): string
    {
        return preg_match('#action="' . preg_quote($action, '#') . '">\s*<input type="hidden" name="_token" value="([^"]+)"#', $page, $m) ? $m[1] : '';
    }

    private function get(string $path, Session $session): string
    {
        return (string) $this->handle(Request::create($path), $session)->getContent();
    }

    private function status(string $path, Session $session): int
    {
        return $this->handle(Request::create($path), $session)->getStatusCode();
    }

    /** @param array<string, string> $data */
    private function post(string $path, array $data, Session $session): Response
    {
        return $this->handle(Request::create($path, 'POST', $data), $session);
    }

    private function handle(Request $request, Session $session): Response
    {
        $this->tokens->setToken(null);
        $request->setSession($session);
        $request->headers->set('User-Agent', (string) $session->get('_probe_agent', 'Sonde'));
        if ($session->getId() !== '') {
            $request->cookies->set($session->getName(), $session->getId());
        }

        return $this->kernel->handle($request, HttpKernelInterface::MAIN_REQUEST, true);
    }

    private function renderAsNobody(): void
    {
        (new \ReflectionProperty(ConsoleRenderAuthenticator::class, 'identifier'))->setValue($this->renderAs, null);
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
