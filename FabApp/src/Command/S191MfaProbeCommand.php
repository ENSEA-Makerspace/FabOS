<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use App\Security\ConsoleRenderAuthenticator;
use App\Security\MfaService;
use App\Security\Totp;
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
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;

/**
 * S191b — la double authentification, éprouvée : l'algorithme contre la RFC, puis
 * le parcours entier comme un navigateur, dans une transaction annulée.
 */
#[AsCommand(name: 'app:s191:mfa-probe', description: 'S191b : TOTP contre les vecteurs RFC 6238 ; inscription, barrière (pages et API), rejeu refusé, codes de secours à usage unique, 5 erreurs, retrait admin. Transaction annulée.')]
final class S191MfaProbeCommand extends Command
{
    private const PASSWORD = 'sonde-S191b-motdepasse';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly MfaService $mfa,
        private readonly UserPasswordHasherInterface $hasher,
        private readonly TokenStorageInterface $tokens,
        private readonly ConsoleRenderAuthenticator $renderAs,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('save', null, InputOption::VALUE_REQUIRED, 'Dossier où enregistrer les pages rendues (inscription avec QR, codes de secours, page du code)');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];
        $save = $input->getOption('save');

        $io->section('1. 🔴 L\'algorithme contre la RFC 6238 (annexe B, SHA-1, 8 chiffres)');
        $seed = Totp::base32Encode('12345678901234567890');
        foreach ([59 => '94287082', 1111111109 => '07081804', 1111111111 => '14050471', 1234567890 => '89005924', 2000000000 => '69279037'] as $t => $expected) {
            $this->check($io, $failures, sprintf('T=%d → %s', $t, $expected), Totp::code($seed, Totp::stepAt($t), 8) === $expected);
        }
        $raw = random_bytes(20);
        $this->check($io, $failures, 'base32 : aller-retour exact', Totp::base32Decode(Totp::base32Encode($raw)) === $raw);

        if (!$this->mfa->isReady()) {
            $io->warning('Table USER_MFA absente : migration S191 en attente.');

            return Command::FAILURE;
        }

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
        $email = $member->getEmail();
        $rowsBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM USER_MFA');

        $this->db->beginTransaction();
        try {
            $this->db->executeStatement('UPDATE UTILISATEUR SET password = ? WHERE id = ?', [$this->hasher->hashPassword($member, self::PASSWORD), $member->getId()]);

            $io->section('2. Inscription');
            $secret = $this->mfa->start($member);
            $stored = (string) $this->db->fetchOne('SELECT secretEncrypted FROM USER_MFA WHERE userId = ?', [$member->getId()]);
            $this->check($io, $failures, '🔴 le secret n\'est PAS en clair en base', $stored !== '' && !str_contains($stored, $secret) && !str_contains(base64_decode($stored) ?: '', $secret));
            $this->check($io, $failures, 'état « en attente »', $this->mfa->status($member) === MfaService::PENDING);
            $s = $this->login($email);
            $this->check($io, $failures, 'en attente, la connexion n\'est PAS barrée (un QR mal scanné n\'enferme personne)', $this->status('/profil', $s) === 200);
            $page = $this->get('/profil/double-authentification', $s);
            $this->check($io, $failures, 'la page montre un QR et la clé', str_contains($page, 'class="mfa-qr"') && str_contains($page, 'data:image/svg+xml'));
            if (is_string($save)) {
                file_put_contents($save . '/mfa-inscription.html', $page);
            }
            $this->check($io, $failures, 'un code faux ne l\'active pas', $this->mfa->confirm($member, '000000') === null);
            $now = time();
            $codes = $this->mfa->confirm($member, Totp::code($secret, Totp::stepAt($now)), $now);
            $this->check($io, $failures, 'le bon code l\'active et rend 10 codes de secours', is_array($codes) && \count($codes) === 10 && $this->mfa->status($member) === MfaService::ENABLED);
            $hashes = (string) $this->db->fetchOne('SELECT recoveryCodes FROM USER_MFA WHERE userId = ?', [$member->getId()]);
            $this->check($io, $failures, '🔴 les codes de secours ne sont pas en clair en base', !str_contains($hashes, (string) ($codes[0] ?? 'x')));

            $io->section('3. La barrière');
            $g = $this->login($email);
            $r = $this->handle(Request::create('/profil'), $g);
            $this->check($io, $failures, '🔴 /profil renvoie vers la page du code', $r->isRedirect() && str_ends_with((string) $r->headers->get('Location'), '/connexion/verification'));
            $this->check($io, $failures, '🔴 une autre page non plus', str_ends_with((string) $this->handle(Request::create('/profil/sessions'), $g)->headers->get('Location'), '/connexion/verification'));
            $api = $this->handle(Request::create('/api/me/favorite-machines'), $g);
            $this->check($io, $failures, '🔴 l\'API répond 401 mfa_required', $api->getStatusCode() === 401 && str_contains((string) $api->getContent(), 'mfa_required'));
            $this->check($io, $failures, 'la page du code, elle, s\'affiche', $this->status('/connexion/verification', $g) === 200);
            if (is_string($save)) {
                file_put_contents($save . '/mfa-code.html', $this->get('/connexion/verification', $g));
            }

            $io->section('4. Le code, puis le rejeu');
            $this->challenge($g, '123456');
            $this->check($io, $failures, 'un code faux : toujours barré', $this->status('/profil', $g) === 302);
            $next = Totp::code($secret, Totp::stepAt($now) + 1);
            $this->challenge($g, $next);
            $this->check($io, $failures, 'le bon code ouvre la session', $this->status('/profil', $g) === 200);
            $h = $this->login($email);
            $this->challenge($h, $next);
            $this->check($io, $failures, '🔴 le MÊME code, rejoué ailleurs : refusé', $this->status('/profil', $h) === 302);

            $io->section('5. Codes de secours');
            $this->challenge($h, (string) $codes[0]);
            $this->check($io, $failures, 'un code de secours ouvre la session', $this->status('/profil', $h) === 200);
            $this->check($io, $failures, 'il en reste 9', $this->mfa->recoveryLeft($member) === 9);
            $k = $this->login($email);
            $this->challenge($k, strtolower(str_replace('-', ' ', (string) $codes[0])));
            $this->check($io, $failures, '🔴 le même code de secours, deux fois : refusé', $this->status('/profil', $k) === 302);

            $io->section('6. Cinq erreurs referment la session');
            $m = $this->login($email);
            for ($i = 0; $i < 5; ++$i) {
                $this->challenge($m, '111111');
            }
            $after = $this->handle(Request::create('/profil'), $m);
            $this->check($io, $failures, 'après 5 erreurs, /profil redemande la CONNEXION (plus la page du code)', str_ends_with((string) $after->headers->get('Location'), '/login'));

            $io->section('7. Le rendu console passe ; l\'équipe retire le second facteur');
            $this->renderAs->renderAs($email);
            $this->check($io, $failures, 'app:render voit le profil (pas une connexion)', $this->status('/profil', new Session(new MockArraySessionStorage())) === 200);
            $this->renderAsNobody();
            $this->mfa->reset($member);
            $n = $this->login($email);
            $this->check($io, $failures, 'retiré : la connexion n\'est plus barrée', $this->mfa->status($member) === MfaService::NONE && $this->status('/profil', $n) === 200);
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
            $this->renderAsNobody();
        }

        $this->check($io, $failures, 'USER_MFA revenue à son compte de départ', (int) $this->db->fetchOne('SELECT COUNT(*) FROM USER_MFA') === $rowsBefore);

        if ($failures !== []) {
            $io->error(count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S191b verte.');

        return Command::SUCCESS;
    }

    private function challenge(Session $session, string $code): void
    {
        $page = $this->get('/connexion/verification', $session);
        $token = preg_match('#name="_token" value="([^"]+)"#', $page, $m) ? $m[1] : '';
        $this->handle(Request::create('/connexion/verification', 'POST', ['_token' => $token, 'code' => $code]), $session);
    }

    private function login(string $email): Session
    {
        $session = new Session(new MockArraySessionStorage());
        $form = $this->get('/login', $session);
        $token = preg_match('#name="_csrf_token"\s+value="([^"]+)"#', $form, $m) ? $m[1] : '';
        $this->handle(Request::create('/login', 'POST', ['_username' => $email, '_password' => self::PASSWORD, '_csrf_token' => $token]), $session);

        return $session;
    }

    private function get(string $path, Session $session): string
    {
        return (string) $this->handle(Request::create($path), $session)->getContent();
    }

    private function status(string $path, Session $session): int
    {
        return $this->handle(Request::create($path), $session)->getStatusCode();
    }

    private function handle(Request $request, Session $session): Response
    {
        $this->tokens->setToken(null);
        $request->setSession($session);
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
