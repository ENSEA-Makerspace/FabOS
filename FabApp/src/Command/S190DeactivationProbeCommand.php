<?php

namespace App\Command;

use App\Account\AccountDeactivation;
use App\Entity\AccessPoint;
use App\Entity\Machine;
use App\Entity\Utilisateur;
use App\Repository\MachineRepository;
use App\Repository\UtilisateurRepository;
use App\Rfid\DoorAccessDecision;
use App\Service\MachineAccessService;
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
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S190 — « désactiver » coupe tout, tout de suite, et la sonde le prouve.
 *
 * Trois chemins, chacun mesuré AVANT et APRÈS la bascule, sur le même compte,
 * pour que ce soit bien le statut qui change la réponse :
 *   1. le badge à une machine (et donc le démarrage d'une session machine) ;
 *   2. le badge à une porte ;
 *   3. une session web DÉJÀ OUVERTE — la page, puis l'API.
 * S190d — puis PAR L'ÉCRAN : un administrateur connecté désactive depuis la
 * fiche (le nombre écrit à côté du bouton = celui annulé), les réservations à
 * venir sont annulées et les passées gardées, soi-même est refusé, et
 * « Réactiver » rend l'accès sans ressusciter les réservations.
 *
 * ✅ Transaction annulée : le mot de passe de sonde, le badge posé s'il en
 * manquait un, la bascule et les lignes du journal d'accès disparaissent. La
 * sonde compare les comptes de tables avant/après.
 */
#[AsCommand(name: 'app:s190:deactivation-probe', description: 'S190 : un compte passé « inactif » perd son badge (machines, portes) et ses sessions ouvertes (page et API), immédiatement. Transaction annulée.')]
final class S190DeactivationProbeCommand extends Command
{
    private const PASSWORD = 'sonde-S190-motdepasse';

    private string $lastLogin = '';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly MachineRepository $machines,
        private readonly MachineAccessService $machineAccess,
        private readonly DoorAccessDecision $doors,
        private readonly UserPasswordHasherInterface $hasher,
        private readonly TokenStorageInterface $tokens,
        private readonly AccountDeactivation $deactivation,
        private readonly TranslatorInterface $translator,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $tables = ['UTILISATEUR', 'ACCESS_RFID_LOG', 'MACHINE', 'EMAIL_LOG', 'RESERVATION', 'USER_SESSION'];
        $counts = function () use ($tables): array {
            $out = [];
            foreach ($tables as $table) {
                try {
                    $out[$table] = (int) $this->db->fetchOne('SELECT COUNT(*) FROM ' . $table);
                } catch (\Throwable) {
                    $out[$table] = -1;
                }
            }

            return $out;
        };
        $before = $counts();
        $statutBefore = $this->db->fetchAllKeyValue('SELECT id, statut FROM UTILISATEUR');
        $reservationsBefore = $this->db->fetchAllKeyValue('SELECT id, statut FROM RESERVATION');

        // Le membre n'est PAS administrateur (sinon « dernier administrateur »
        // pourrait refuser la désactivation par l'écran) ; l'administrateur est un autre compte.
        $member = null;
        $admin = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            $isAdmin = \in_array('ROLE_ADMIN', $candidate->getRoles(), true);
            if ($isAdmin && $admin === null) {
                $admin = $candidate;
            } elseif (!$isAdmin && $member === null) {
                $member = $candidate;
            }
        }
        $machine = null;
        foreach ($this->machines->findAll() as $candidate) {
            if ($candidate instanceof Machine && (string) $candidate->getMachineToken() !== '') {
                $machine = $candidate;
                break;
            }
        }
        if (!$member instanceof Utilisateur || !$admin instanceof Utilisateur || !$machine instanceof Machine) {
            $io->error('Il faut un compte actif, un administrateur actif et une machine à jeton.');

            return Command::FAILURE;
        }
        $io->writeln(sprintf('   compte #%d, machine « %s »', $member->getId(), $machine->getNom()));

        $this->db->beginTransaction();
        try {
            // Un badge et un mot de passe connus, le temps de la transaction.
            if ((string) $member->getIdentifiantRfid() === '') {
                $member->setIdentifiantRfid('SONDE-S190-' . bin2hex(random_bytes(3)));
            }
            $member->setPassword($this->hasher->hashPassword($member, self::PASSWORD));
            $admin->setPassword($this->hasher->hashPassword($admin, self::PASSWORD));
            $this->entityManager->flush();
            // Un second facteur arrêterait la connexion de sonde à la page du code.
            try {
                $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId IN (?, ?)', [$member->getId(), $admin->getId()]);
            } catch (\Throwable) {
            }
            $memberId = (int) $member->getId();
            $adminId = (int) $admin->getId();
            $adminEmail = $admin->getEmail();
            $rfid = (string) $member->getIdentifiantRfid();
            $email = $member->getEmail();

            $io->section('Avant : le compte est actif');
            $machineBefore = $this->machineAccess->authorize((string) $machine->getMachineToken(), $rfid);
            $io->writeln('   machine : ' . $machineBefore['status']);
            $this->check($io, $failures, 'la machine ne répond pas « compte désactivé »', ($machineBefore['status'] ?? '') !== 'account_inactive');
            $door = new AccessPoint();
            $doorBefore = $this->doors->decide($door, $member);
            $this->check($io, $failures, 'la porte non plus (' . $doorBefore['status'] . ')', $doorBefore['status'] !== 'account_inactive');

            $page = $this->login($email);
            $api = $this->login($email);
            $io->writeln('   connexion : ' . $this->lastLogin);
            $this->check($io, $failures, 'deux sessions ouvertes : /profil répond 200', $this->status('/profil', $page) === 200 && $this->status('/profil', $api) === 200);

            $io->section('La bascule : « inactif »');
            $this->db->executeStatement("UPDATE UTILISATEUR SET statut = 'inactif' WHERE id = ?", [$member->getId()]);
            $this->entityManager->clear();
            $member = $this->users->find($member->getId());

            $io->section('1. Le badge, à la machine');
            $machineAfter = $this->machineAccess->authorize((string) $machine->getMachineToken(), $rfid);
            $this->check($io, $failures, '🔴 refusé : 403 account_inactive', ($machineAfter['authorized'] ?? true) === false && $machineAfter['status'] === 'account_inactive' && (int) $machineAfter['httpStatus'] === 403);

            $io->section('2. Le badge, à la porte');
            $doorAfter = $this->doors->decide($door, $member);
            $this->check($io, $failures, '🔴 refusé : account_inactive', !$doorAfter['allowed'] && $doorAfter['status'] === 'account_inactive');

            $io->section('3. Les sessions DÉJÀ OUVERTES');
            $response = $this->handle(Request::create('/profil'), $page);
            $this->check($io, $failures, '🔴 la page suivante renvoie à /login (' . $response->getStatusCode() . ')', $response->isRedirect() && str_ends_with((string) $response->headers->get('Location'), '/login'));
            $login = (string) $this->handle(Request::create('/login'), $page)->getContent();
            $this->check($io, $failures, 'la connexion dit pourquoi, sans clé brute', str_contains($login, 'auth-alert-error') && !str_contains($login, 'security.account_unavailable'));
            $this->check($io, $failures, 'et la session est bien FERMÉE : /profil redemande la connexion', $this->status('/profil', $page) === 302);
            $apiResponse = $this->handle(Request::create('/api/me/favorite-machines'), $api);
            $this->check($io, $failures, '🔴 l\'API répond 401 account_inactive (' . $apiResponse->getStatusCode() . ')', $apiResponse->getStatusCode() === 401 && str_contains((string) $apiResponse->getContent(), 'account_inactive'));

            $io->section('4. Et une nouvelle connexion reste refusée');
            $again = $this->login($email);
            $this->check($io, $failures, '/profil redemande la connexion', $this->status('/profil', $again) === 302);

            $this->probeScreen($io, $failures, $memberId, $email, $adminId, $adminEmail, (int) $machine->getId());
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
        }

        $io->section('9. Rien n\'est resté');
        $after = $counts();
        foreach ($before as $table => $count) {
            $this->check($io, $failures, sprintf('%s : %d avant, %d après', $table, $count, $after[$table]), $after[$table] === $count);
        }
        $this->check($io, $failures, 'aucun statut de compte n\'a changé', $this->db->fetchAllKeyValue('SELECT id, statut FROM UTILISATEUR') === $statutBefore);
        $this->check($io, $failures, 'aucune réservation n\'a changé de statut', $this->db->fetchAllKeyValue('SELECT id, statut FROM RESERVATION') === $reservationsBefore);

        if ($failures !== []) {
            $io->error(count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S190 verte. Transaction annulée.');

        return Command::SUCCESS;
    }

    /**
     * S190d — la désactivation par l'écran, comme un administrateur la fait.
     *
     * @param list<string> $failures
     */
    private function probeScreen(SymfonyStyle $io, array &$failures, int $memberId, string $email, int $adminId, string $adminEmail, int $machineId): void
    {
        $io->section('6. S190d — désactiver depuis la fiche admin');
        $this->db->executeStatement("UPDATE UTILISATEUR SET statut = 'actif' WHERE id = ?", [$memberId]);
        $insert = "INSERT INTO RESERVATION (userId, reservableType, reservableId, reservableLabel, dateDebut, dateFin, statut, created)"
            . " VALUES (?, 'machine', ?, 'sonde S190d', ?, ?, 'confirmed', NOW())";
        $this->db->executeStatement($insert, [$memberId, $machineId, date('Y-m-d H:i:s', strtotime('+2 days')), date('Y-m-d H:i:s', strtotime('+2 days +1 hour'))]);
        $future = (int) $this->db->lastInsertId();
        $this->db->executeStatement($insert, [$memberId, $machineId, date('Y-m-d H:i:s', strtotime('-2 days')), date('Y-m-d H:i:s', strtotime('-2 days +1 hour'))]);
        $past = (int) $this->db->lastInsertId();
        $statuses = fn (): array => $this->db->fetchAllKeyValue('SELECT id, statut FROM RESERVATION WHERE userId = ?', [$memberId]);
        $before = $statuses();

        $memberSession = $this->login($email);
        $this->check($io, $failures, 'le membre est connecté : /profil 200', $this->status('/profil', $memberSession) === 200);
        $adminSession = $this->login($adminEmail);
        $page = $this->handle(Request::create('/admin/utilisateurs/' . $memberId), $adminSession);
        $html = (string) $page->getContent();
        $this->check($io, $failures, 'la fiche s\'ouvre pour l\'administrateur (' . $page->getStatusCode() . ')', $page->getStatusCode() === 200);
        $token = $this->formToken($html, '/admin/utilisateurs/' . $memberId . '/desactiver');
        $this->check($io, $failures, 'la fiche porte « Désactiver ce compte »', $token !== '');
        $this->check($io, $failures, 'et le formulaire demande confirmation', (bool) preg_match('#action="/admin/utilisateurs/' . $memberId . '/desactiver"[^>]*data-action="submit->confirm\#ask"#s', $html));

        $response = $this->post('/admin/utilisateurs/' . $memberId . '/desactiver', $token, $adminSession);
        $this->check($io, $failures, 'le POST revient à la fiche (' . $response->getStatusCode() . ')', $response->isRedirect());
        $after = $statuses();
        $changed = array_keys(array_filter($after, fn ($statut, $id) => ($before[$id] ?? null) !== $statut, ARRAY_FILTER_USE_BOTH));
        $this->check($io, $failures, '🔴 le compte est « inactif » en base', $this->db->fetchOne('SELECT statut FROM UTILISATEUR WHERE id = ?', [$memberId]) === 'inactif');
        $this->check($io, $failures, '🔴 la réservation dans deux jours est annulée', ($after[$future] ?? '') === 'cancelled');
        $this->check($io, $failures, 'celle d\'avant-hier est gardée (' . ($after[$past] ?? '?') . ')', ($after[$past] ?? '') === 'confirmed');
        $this->check($io, $failures, 'seules des réservations passées à « cancelled » ont changé', array_filter($changed, fn ($id) => $after[$id] !== 'cancelled') === []);
        $this->check($io, $failures, 'le nombre écrit À CÔTÉ du bouton était celui annulé (' . \count($changed) . ')', $this->inAnyLocale($html, 'account_status.deactivate_help', ['count' => \count($changed)]));
        $back = (string) $this->handle(Request::create('/admin/utilisateurs/' . $memberId), $adminSession)->getContent();
        $this->check($io, $failures, 'le message le redit après coup', $this->inAnyLocale($back, 'account_status.deactivated', ['count' => \count($changed)]));
        $this->check($io, $failures, '🔴 la session du membre est coupée : /profil 302', $this->status('/profil', $memberSession) === 302);

        $io->section('7. Soi-même : refusé');
        $self = (string) $this->handle(Request::create('/admin/utilisateurs/' . $adminId), $adminSession)->getContent();
        $this->check($io, $failures, 'sa propre fiche n\'a pas de bouton', $this->formToken($self, '/admin/utilisateurs/' . $adminId . '/desactiver') === '');
        $this->check($io, $failures, 'elle dit pourquoi', $this->inAnyLocale($self, 'account_status.refused_self', []));
        $adminEntity = $this->users->find($adminId);
        $this->check($io, $failures, 'et la règle du POST refuse (self)', $adminEntity instanceof Utilisateur && $this->deactivation->refusalFor($adminEntity, $adminEntity) === AccountDeactivation::REFUSED_SELF);

        $io->section('8. Réactiver');
        $token = $this->formToken($back, '/admin/utilisateurs/' . $memberId . '/reactiver');
        $this->check($io, $failures, 'la fiche porte « Réactiver ce compte »', $token !== '');
        $this->post('/admin/utilisateurs/' . $memberId . '/reactiver', $token, $adminSession);
        $this->check($io, $failures, 'le compte est « actif »', $this->db->fetchOne('SELECT statut FROM UTILISATEUR WHERE id = ?', [$memberId]) === 'actif');
        $this->check($io, $failures, 'la réservation annulée NE revient PAS', $statuses()[$future] === 'cancelled');
        $this->check($io, $failures, 'le membre peut se reconnecter : /profil 200', $this->status('/profil', $this->login($email)) === 200);
    }

    private function formToken(string $html, string $action): string
    {
        return preg_match('#action="' . preg_quote($action, '#') . '".*?name="_token" value="([^"]+)"#s', $html, $m) ? $m[1] : '';
    }

    private function post(string $path, string $token, Session $session): Response
    {
        $request = Request::create($path, 'POST', ['_token' => $token]);
        $request->headers->set('Origin', $request->getSchemeAndHttpHost());

        return $this->handle($request, $session);
    }

    /** La langue de la page suit le compte connecté : on accepte les cinq. */
    private function inAnyLocale(string $html, string $key, array $params): bool
    {
        foreach (['fr', 'en', 'de', 'es', 'it'] as $locale) {
            $text = $this->translator->trans($key, $params, null, $locale);
            if ($text !== $key && str_contains($html, htmlspecialchars($text, ENT_QUOTES))) {
                return true;
            }
        }

        return false;
    }

    private function login(string $email): Session
    {
        $session = new Session(new MockArraySessionStorage());
        $form = (string) $this->handle(Request::create('/login'), $session)->getContent();
        $token = preg_match('#name="_csrf_token"\s+value="([^"]+)"#', $form, $m) ? $m[1] : '';
        $response = $this->handle(Request::create('/login', 'POST', ['_username' => $email, '_password' => self::PASSWORD, '_csrf_token' => $token]), $session);
        $this->lastLogin = $response->getStatusCode() . ' → ' . $response->headers->get('Location') . ($token === '' ? ' (sans jeton CSRF)' : '');

        return $session;
    }

    private function status(string $path, Session $session): int
    {
        return $this->handle(Request::create($path), $session)->getStatusCode();
    }

    /**
     * ⚠️ Le conteneur survit d'une requête à l'autre dans une commande : sans
     * remise à zéro, le jeton de la session PRÉCÉDENTE resterait dans le
     * stockage et la requête suivante serait authentifiée par erreur.
     */
    private function handle(Request $request, Session $session): Response
    {
        $this->tokens->setToken(null);
        $request->setSession($session);
        // ⚠️ Le pare-feu ne relit le jeton en session QUE si la requête porte le
        // cookie de session (`hasPreviousSession()`) : sans lui, chaque requête
        // repart anonyme et une « session ouverte » n'est jamais mesurée.
        if ($session->getId() !== '') {
            $request->cookies->set($session->getName(), $session->getId());
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
