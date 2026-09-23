<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use App\Security\AccountActivation;
use App\Security\AccountVerificationTokenizer;
use App\Security\PasswordResetTokenizer;
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
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S189 — l'inscription, rejouée comme un navigateur, et TOUT annulé.
 *
 * 🔴 **La mesure centrale : deux adresses, l'une membre l'autre non, réponses
 * identiques** — statut, redirection, et la page qui suit, octet pour octet une
 * fois retirés les jetons CSRF et l'adresse tapée elle-même.
 *
 * ✅ **Aucun courrier ne part** : le journal des mails ET la file Messenger
 * vivent dans la même base, sur la même connexion ; la sonde travaille dans une
 * transaction qu'elle annule, donc le worker ne voit jamais rien. Elle compte
 * `EMAIL_LOG`, `messenger_messages` et `UTILISATEUR` avant et après.
 */
#[AsCommand(name: 'app:s189:register-probe', description: 'S189 : inscription non divulguante (adresse membre vs libre, réponses identiques), activation, correction, connexion refusée puis permise. Transaction annulée, aucun courrier envoyé.')]
final class S189RegisterProbeCommand extends Command
{
    private const PASSWORD = 'sonde-S189-motdepasse';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly AccountActivation $activation,
        private readonly AccountVerificationTokenizer $verifyTokens,
        private readonly PasswordResetTokenizer $resetTokens,
        private readonly TranslatorInterface $translator,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('save', null, InputOption::VALUE_REQUIRED, 'Dossier où enregistrer les pages rendues (vérifiez-votre-boîte, connexion refusée, lien périmé, inscription)');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $saveDir = $input->getOption('save');
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        if (!$this->activation->isRequired()) {
            $io->warning('Le courrier n\'est pas opérationnel : l\'inscription garde l\'ancien comportement, rien à éprouver.');

            return Command::SUCCESS;
        }

        $counts = fn (): array => [
            'UTILISATEUR' => (int) $this->db->fetchOne('SELECT COUNT(*) FROM UTILISATEUR'),
            'EMAIL_LOG' => (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG'),
            'messenger_messages' => (int) $this->db->fetchOne('SELECT COUNT(*) FROM messenger_messages'),
        ];
        $before = $counts();
        $member = $this->users->findOneBy(['statut' => 'actif', 'isVerified' => true]);
        if (!$member instanceof Utilisateur) {
            $io->error('Aucun compte actif pour servir d\'adresse connue.');

            return Command::FAILURE;
        }
        $memberEmail = $member->getEmail();
        $memberHash = $member->getPassword();
        $suffix = bin2hex(random_bytes(4));
        $fresh = 'sonde-s189-' . $suffix . '@example.invalid';
        $lastLog = (int) $this->db->fetchOne('SELECT COALESCE(MAX(id), 0) FROM EMAIL_LOG');

        $this->db->beginTransaction();
        try {
            $io->section('1. 🔴 Adresse MEMBRE contre adresse LIBRE : la même réponse');
            [$a, $aPage] = $this->register($memberEmail);
            [$b, $bPage] = $this->register($fresh);
            $io->writeln(sprintf('   membre : %d → %s', $a->getStatusCode(), $a->headers->get('Location')));
            $io->writeln(sprintf('   libre  : %d → %s', $b->getStatusCode(), $b->headers->get('Location')));
            $this->check($io, $failures, 'même statut et même redirection', $a->getStatusCode() === $b->getStatusCode() && $a->headers->get('Location') === $b->headers->get('Location') && $a->getStatusCode() === 303);
            $same = $this->normalise($aPage, $memberEmail) === $this->normalise($bPage, $fresh);
            $this->check($io, $failures, '🔴 la page « vérifiez votre boîte » est identique, octet pour octet', $same);
            if (!$same) {
                $io->writeln('   écart : ' . $this->firstDifference($this->normalise($aPage, $memberEmail), $this->normalise($bPage, $fresh)));
            }

            if (is_string($saveDir)) {
                file_put_contents($saveDir . '/s189-verifier.html', $bPage);
                file_put_contents($saveDir . '/s189-inscription.html', $this->get('/register', new Session(new MockArraySessionStorage())));
                file_put_contents($saveDir . '/s189-lien-perime.html', (string) $this->handle(Request::create('/inscription/activer/abc.def'), new Session(new MockArraySessionStorage()))->getContent());
            }

            $io->section('2. Ce qui est parti — dans la file, pas au serveur');
            $logs = $this->db->fetchAllAssociative('SELECT recipient, template FROM EMAIL_LOG WHERE id > ? ORDER BY id', [$lastLog]);
            $io->writeln('   ' . implode(' ; ', array_map(static fn (array $r): string => $r['template'] . ' → ' . $r['recipient'], $logs)));
            $this->check($io, $failures, 'le membre reçoit « vous avez déjà un compte », et rien d\'autre', $this->sent($logs, $memberEmail) === ['account_exists']);
            $this->check($io, $failures, 'l\'adresse libre reçoit son lien d\'activation', $this->sent($logs, $fresh) === ['account_verify']);
            $this->entityManager->clear();
            $this->check($io, $failures, 'le compte du membre n\'est PAS touché (même mot de passe)', $this->users->findOneBy(['email' => $memberEmail])?->getPassword() === $memberHash);
            $created = $this->users->findOneBy(['email' => $fresh]);
            $this->check($io, $failures, 'le nouveau compte existe, NON vérifié', $created instanceof Utilisateur && !$created->isVerified());

            $io->section('3. Connexion avant confirmation : refusée, avec une sortie');
            [$login, $after] = $this->login($fresh);
            $this->check($io, $failures, 'retour sur /login', str_ends_with((string) $login->headers->get('Location'), '/login'));
            $alert = preg_match('#<div class="auth-alert auth-alert-error">(.*?)</div>#s', $after, $m) ? trim((string) preg_replace('/\s+/', ' ', strip_tags($m[1]))) : '';
            $io->writeln('   affiché : « ' . $alert . ' »');
            if (is_string($saveDir)) {
                file_put_contents($saveDir . '/s189-connexion-refusee.html', $after);
            }
            $this->check($io, $failures, 'le message est TRADUIT, et c\'est celui de la confirmation', $alert !== '' && !str_contains($alert, 'security.') && $alert !== $this->translator->trans('security.account_unavailable') && str_contains($alert, $this->translator->trans('security.email_unconfirmed')));
            $this->check($io, $failures, 'et propose « recevoir un nouveau lien »', str_contains($after, '/inscription/verifier'));
            [, $wrongPage] = $this->login($fresh, 'mauvais-mot-de-passe');
            $this->check($io, $failures, 'un MAUVAIS mot de passe ne dit rien de la confirmation', !str_contains($wrongPage, '/inscription/verifier'));

            $io->section('4. Les liens');
            $this->check($io, $failures, 'un lien forgé → 410', $this->get('/inscription/activer/abc.def', new Session(new MockArraySessionStorage()), true) === 410);
            $reset = $this->resetTokens->create($created, new \DateTimeImmutable());
            $this->check($io, $failures, '🔴 un lien de RÉINITIALISATION ne vaut pas activation → 410', $this->get('/inscription/activer/' . $reset, new Session(new MockArraySessionStorage()), true) === 410);
            $expired = $this->verifyTokens->create($created, new \DateTimeImmutable('-3 days'));
            $this->check($io, $failures, 'un lien expiré → 410', $this->get('/inscription/activer/' . $expired, new Session(new MockArraySessionStorage()), true) === 410);
            $good = $this->verifyTokens->create($created, new \DateTimeImmutable());
            $session = new Session(new MockArraySessionStorage());
            $response = $this->handle(Request::create('/inscription/activer/' . $good), $session);
            $this->entityManager->clear();
            $this->check($io, $failures, 'le bon lien : 303 vers /login, compte vérifié', $response->getStatusCode() === 303 && str_ends_with((string) $response->headers->get('Location'), '/login') && $this->users->findOneBy(['email' => $fresh])?->isVerified() === true);
            $this->check($io, $failures, 'rejoué : 410 (usage unique)', $this->get('/inscription/activer/' . $good, new Session(new MockArraySessionStorage()), true) === 410);

            $io->section('5. Corriger une adresse mal tapée, sans recréer de compte');
            $typo = 'sonde-s189-' . $suffix . '-faute@example.invalid';
            $right = 'sonde-s189-' . $suffix . '-juste@example.invalid';
            $session = new Session(new MockArraySessionStorage());
            $this->register($typo, $session);
            $this->entityManager->clear();
            $typoUser = $this->users->findOneBy(['email' => $typo]);
            $oldLink = $typoUser instanceof Utilisateur ? $this->verifyTokens->create($typoUser, new \DateTimeImmutable()) : 'x.y';
            $usersBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM UTILISATEUR');
            $page = $this->get('/inscription/verifier', $session);
            $token = preg_match('#action="/inscription/corriger">\s*<input type="hidden" name="_token" value="([^"]+)"#', $page, $m) ? $m[1] : '';
            $this->handle(Request::create('/inscription/corriger', 'POST', ['_token' => $token, 'email' => $right]), $session);
            $this->entityManager->clear();
            $this->check($io, $failures, 'aucun compte de plus', (int) $this->db->fetchOne('SELECT COUNT(*) FROM UTILISATEUR') === $usersBefore);
            $this->check($io, $failures, 'le même compte porte la bonne adresse', $typoUser !== null && $this->users->find($typoUser->getId())?->getEmail() === $right);
            $logs = $this->db->fetchAllAssociative('SELECT recipient, template FROM EMAIL_LOG WHERE id > ? ORDER BY id', [$lastLog]);
            $this->check($io, $failures, 'un lien part à la bonne adresse', $this->sent($logs, $right) === ['account_verify']);
            $this->check($io, $failures, '🔴 le lien parti vers la faute ne marche plus', $this->get('/inscription/activer/' . $oldLink, new Session(new MockArraySessionStorage()), true) === 410);

            $io->section('6. Après confirmation, la connexion passe');
            [$ok] = $this->login($fresh);
            $this->check($io, $failures, 'redirigé hors de /login', !str_ends_with((string) $ok->headers->get('Location'), '/login') && $ok->getStatusCode() === 302);
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
        }

        $io->section('7. Rien n\'est resté, rien n\'est parti');
        $after = $counts();
        foreach ($before as $table => $count) {
            $this->check($io, $failures, sprintf('%s : %d avant, %d après', $table, $count, $after[$table]), $after[$table] === $count);
        }

        if ($failures !== []) {
            $io->error(count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S189 verte. Transaction annulée, aucun courrier envoyé.');

        return Command::SUCCESS;
    }

    /** @return array{0: Response, 1: string} la réponse au POST, et la page qui suit */
    private function register(string $email, ?Session $session = null): array
    {
        $session ??= new Session(new MockArraySessionStorage());
        $form = $this->get('/register', $session);
        $token = preg_match('#name="_token" value="([^"]+)"#', $form, $m) ? $m[1] : '';
        $response = $this->handle(Request::create('/register', 'POST', [
            '_token' => $token,
            'firstName' => 'Sonde',
            'lastName' => 'S189',
            'email' => $email,
            'password' => self::PASSWORD,
            'confirmPassword' => self::PASSWORD,
            'terms' => '1',
        ]), $session);

        return [$response, $this->get('/inscription/verifier', $session)];
    }

    /** @return array{0: Response, 1: string} */
    private function login(string $email, string $password = self::PASSWORD): array
    {
        $session = new Session(new MockArraySessionStorage());
        $form = $this->get('/login', $session);
        $token = preg_match('#name="_csrf_token"\s+value="([^"]+)"#', $form, $m) ? $m[1] : '';
        $response = $this->handle(Request::create('/login', 'POST', ['_username' => $email, '_password' => $password, '_csrf_token' => $token]), $session);

        return [$response, $this->get('/login', $session)];
    }

    private function get(string $path, Session $session, bool $statusOnly = false): string|int
    {
        $response = $this->handle(Request::create($path), $session);

        return $statusOnly ? $response->getStatusCode() : (string) $response->getContent();
    }

    private function handle(Request $request, Session $session): Response
    {
        $request->setSession($session);

        return $this->kernel->handle($request, HttpKernelInterface::MAIN_REQUEST, true);
    }

    private function normalise(string $html, string $email): string
    {
        $html = preg_replace('#name="(_token|_csrf_token)" value="[^"]+"#', 'name="$1" value="JETON"', $html) ?? $html;

        return str_replace(htmlspecialchars($email, ENT_QUOTES), 'ADRESSE', str_replace($email, 'ADRESSE', $html));
    }

    private function firstDifference(string $a, string $b): string
    {
        $i = 0;
        while ($i < min(strlen($a), strlen($b)) && $a[$i] === $b[$i]) {
            ++$i;
        }

        return json_encode([substr($a, max(0, $i - 60), 140), substr($b, max(0, $i - 60), 140)], JSON_UNESCAPED_UNICODE) ?: '';
    }

    /**
     * @param list<array{recipient: string, template: string}> $logs
     *
     * @return list<string>
     */
    private function sent(array $logs, string $to): array
    {
        return array_values(array_map(static fn (array $r): string => $r['template'], array_filter($logs, static fn (array $r): bool => $r['recipient'] === $to)));
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
