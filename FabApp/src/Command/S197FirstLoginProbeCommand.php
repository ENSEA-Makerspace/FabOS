<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Identity\AuthProvider;
use App\Identity\ExternalIdentityService;
use App\Identity\ExternalProfile;
use App\Identity\IdentityDecision;
use App\Identity\PendingExternalLogin;
use App\Repository\UtilisateurRepository;
use App\Security\AccountActivation;
use App\Security\AccountVerificationTokenizer;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Input\InputOption;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\KernelInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S197 — la première connexion par un fournisseur, sans impasse, par les
 * VRAIES pages (`$kernel->handle()`), comme un navigateur.
 *
 * Le fournisseur est simulé au seul endroit où il agit : la session reçoit le
 * profil qu'il aurait authentifié (ce que fait `OidcController` après une
 * validation que la sonde S196 prouve déjà).
 *
 *   1. Sans e-mail → la page demande l'e-mail, et RIEN d'autre ; sans nom non plus
 *      → les deux.
 *   2. Une adresse libre → compte créé NON confirmé + lien d'activation ; une
 *      adresse déjà prise → aucun compte, « vous avez déjà un compte » à sa
 *      propriétaire ; 🔴 et les deux écrans sont IDENTIQUES.
 *   3. Adresse garantie mais déjà prise → pas pré-remplie ; deux comptes
 *      distincts tant qu'on n'a pas prouvé : se connecter au compte local,
 *      confirmer sur une page qui montre l'identité en jeu → un seul compte.
 *      Annuler → rien. Sans le clic « J'ai déjà un compte », une connexion
 *      locale ne mène à AUCUNE liaison.
 *   4. Seul le nom manquait → compte ouvert et connecté.
 *   5. « Mot de passe oublié » d'un compte créé par le fournisseur → le
 *      courrier « géré par X », jamais un lien FabOS ; même écran.
 *
 * ✅ Transaction annulée ; comptes de tables comparés avant/après.
 */
#[AsCommand(name: 'app:s197:first-login-probe', description: 'S197 : « Complétez votre compte », adresse confirmée comme S189, liaison d’un compte local avec preuve et confirmation, mot de passe « géré par ». Transaction annulée.')]
final class S197FirstLoginProbeCommand extends Command
{
    use ProbeBrowser;

    private const PASSWORD = 'sonde-S197-motdepasse';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly ExternalIdentityService $identities,
        private readonly PendingExternalLogin $pending,
        private readonly AccountActivation $activation,
        private readonly AccountVerificationTokenizer $verifyTokens,
        private readonly UserPasswordHasherInterface $hasher,
        private readonly TokenStorageInterface $tokens,
        private readonly TranslatorInterface $translator,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('save', null, InputOption::VALUE_REQUIRED, 'Écrire les pages rendues dans ce dossier (pour en regarder les pixels)');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $saveDir = $input->getOption('save');
        $save = static function (string $name, string $html) use ($saveDir): void {
            if (\is_string($saveDir) && $saveDir !== '') {
                @mkdir($saveDir, 0o775, true);
                file_put_contents($saveDir . '/' . $name . '.html', $html);
            }
        };
        $io = new SymfonyStyle($input, $output);
        $failures = [];
        $tables = ['UTILISATEUR', 'EXTERNAL_IDENTITY', 'EMAIL_LOG', 'messenger_messages', 'USER_SESSION'];
        $counts = fn (): array => array_combine($tables, array_map(fn (string $t): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM ' . $t), $tables));
        $before = $counts();
        $provisionedColumn = (bool) $this->db->fetchOne("SELECT COUNT(*) FROM information_schema.COLUMNS WHERE TABLE_SCHEMA = DATABASE() AND TABLE_NAME = 'EXTERNAL_IDENTITY' AND COLUMN_NAME = 'provisioned'");
        $mail = $this->activation->isRequired();
        $io->writeln(sprintf('   courrier opérationnel : %s ; migration S197 : %s', $mail ? 'oui' : 'non', $provisionedColumn ? 'passée' : 'pas encore'));

        $member = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            if (!\in_array('ROLE_ADMIN', $candidate->getRoles(), true) && !str_ends_with($candidate->getEmail(), '.invalid')) {
                $member = $candidate;
                break;
            }
        }
        if (!$member instanceof Utilisateur) {
            $io->error('Il faut un membre actif (non administrateur).');

            return Command::FAILURE;
        }
        [$memberId, $memberEmail] = [(int) $member->getId(), $member->getEmail()];
        $lastLog = (int) $this->db->fetchOne('SELECT COALESCE(MAX(id), 0) FROM EMAIL_LOG');
        $sent = fn (string $to): array => $this->db->fetchFirstColumn('SELECT template FROM EMAIL_LOG WHERE id > ? AND recipient = ? ORDER BY id', [$lastLog, $to]);

        $this->db->beginTransaction();
        try {
            $this->db->executeStatement('UPDATE UTILISATEUR SET password = ? WHERE id = ?', [$this->hasher->hashPassword($member, self::PASSWORD), $memberId]);
            $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId = ?', [$memberId]);

            $io->section('1. Ne demander QUE ce qui manque');
            $noEmail = $this->profile('sub-1', null, false, 'Ada', 'Sonde');
            $d = $this->identities->decide($noEmail);
            $this->check($io, $failures, 'sans e-mail → « compléter », besoin : [email]', $d->outcome === IdentityDecision::COMPLETE && $d->needs === ['email']);
            $s1 = $this->pendingSession($noEmail, $d);
            $page = $this->page('/connexion/completer', $s1);
            $save('complete-email', $page);
            $this->check($io, $failures, 'la page demande l’e-mail…', str_contains($page, 'name="email"'));
            $this->check($io, $failures, '… et RIEN d’autre (ni prénom, ni nom)', !str_contains($page, 'name="firstName"') && !str_contains($page, 'name="lastName"'));
            $bare = $this->profile('sub-2', null, false, null, null);
            $d2 = $this->identities->decide($bare);
            $page2 = $this->page('/connexion/completer', $this->pendingSession($bare, $d2));
            $this->check($io, $failures, 'sans e-mail NI nom → les deux', $d2->needs === ['email', 'name'] && str_contains($page2, 'name="email"') && str_contains($page2, 'name="firstName"'));

            $io->section('2. L’adresse tapée suit S189');
            $fresh = 'sonde-s197-' . bin2hex(random_bytes(3)) . '@example.org';
            $r1 = $this->post('/connexion/completer', ['_token' => $this->formToken($page, '/connexion/completer'), 'email' => $fresh], $s1);
            $created = $this->db->fetchAssociative('SELECT id, isVerified FROM UTILISATEUR WHERE email = ?', [$fresh]);
            $this->check($io, $failures, 'adresse libre → compte créé, NON confirmé (' . $r1->getStatusCode() . ' → ' . $r1->headers->get('Location') . ')', $created !== false && (int) $created['isVerified'] === 0);
            $this->check($io, $failures, 'et lié à l’identité', $created !== false && (int) $this->db->fetchOne('SELECT userId FROM EXTERNAL_IDENTITY WHERE subject = ?', ['sub-1']) === (int) $created['id']);
            if ($mail) {
                $this->check($io, $failures, 'le lien d’activation part à cette adresse', $sent($fresh) === ['account_verify']);
            }
            $check1 = $this->normalise($this->page('/inscription/verifier', $s1), $fresh);

            $taken = $this->profile('sub-3', null, false, 'Bob', 'Sonde');
            $s3 = $this->pendingSession($taken, $this->identities->decide($taken));
            $page3 = $this->page('/connexion/completer', $s3);
            $accountsBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM UTILISATEUR');
            $r3 = $this->post('/connexion/completer', ['_token' => $this->formToken($page3, '/connexion/completer'), 'email' => $memberEmail], $s3);
            $this->check($io, $failures, 'adresse d’un membre → AUCUN compte créé, aucun lien', (int) $this->db->fetchOne('SELECT COUNT(*) FROM UTILISATEUR') === $accountsBefore && !$this->db->fetchOne('SELECT 1 FROM EXTERNAL_IDENTITY WHERE subject = ?', ['sub-3']));
            if ($mail) {
                $this->check($io, $failures, 'la propriétaire reçoit « vous avez déjà un compte », et rien d’autre', $sent($memberEmail) === ['account_exists']);
            }
            $check3 = $this->normalise($this->page('/inscription/verifier', $s3), $memberEmail);
            $this->check($io, $failures, '🔴 même redirection, et le même écran octet pour octet', $r1->headers->get('Location') === $r3->headers->get('Location') && $check1 === $check3);

            $io->section('3. Adresse garantie mais déjà prise : lier, avec preuve');
            $mine = $this->profile('sub-4', $memberEmail, true, 'Membre', 'Sonde');
            $d4 = $this->identities->decide($mine);
            $this->check($io, $failures, 'décision : compléter, adresse prise', $d4->outcome === IdentityDecision::COMPLETE && $d4->emailTaken);
            $s4 = $this->pendingSession($mine, $d4);
            $page4 = $this->page('/connexion/completer', $s4);
            $save('complete-taken', $page4);
            $this->check($io, $failures, 'l’adresse prise n’est PAS pré-remplie ; « J’ai déjà un compte » est proposé', !str_contains($page4, 'value="' . htmlspecialchars($memberEmail) . '"') && str_contains($page4, '/connexion/completer/lier'));
            $this->check($io, $failures, 'avant preuve : aucun lien vers le compte du membre', !$this->db->fetchOne('SELECT 1 FROM EXTERNAL_IDENTITY WHERE userId = ?', [$memberId]));

            $this->post('/connexion/completer/lier', ['_token' => $this->formToken($page4, '/connexion/completer/lier')], $s4);
            $this->login4($s4, $memberEmail);
            $gate = $this->handle(Request::create('/profil'), $s4);
            $this->check($io, $failures, 'connecté : toute page mène à la confirmation (' . $gate->headers->get('Location') . ')', $gate->isRedirect() && str_ends_with((string) $gate->headers->get('Location'), '/connexion/lier'));
            $confirm = $this->page('/connexion/lier', $s4);
            $save('link-confirm', $confirm);
            $this->check($io, $failures, 'la confirmation MONTRE l’identité en jeu (nom, adresse)', str_contains($confirm, 'Membre Sonde') && str_contains($confirm, htmlspecialchars($memberEmail)));
            $this->check($io, $failures, 'rien n’est lié tant qu’on n’a pas cliqué', !$this->db->fetchOne('SELECT 1 FROM EXTERNAL_IDENTITY WHERE userId = ?', [$memberId]));
            $this->post('/connexion/lier', ['_token' => $this->formToken($confirm, '/connexion/lier'), 'answer' => 'link'], $s4);
            $link = $this->db->fetchAssociative('SELECT * FROM EXTERNAL_IDENTITY WHERE subject = ?', ['sub-4']);
            $this->check($io, $failures, '🔴 lié au compte du membre, et c’est le SEUL compte', $link !== false && (int) $link['userId'] === $memberId && (int) $this->db->fetchOne('SELECT COUNT(*) FROM UTILISATEUR WHERE email = ?', [$memberEmail]) === 1);
            if ($provisionedColumn) {
                $this->check($io, $failures, 'marqué « lié », pas « créé » (son mot de passe reste le sien)', (int) $link['provisioned'] === 0);
            }
            $this->check($io, $failures, 'la connexion suivante par le fournisseur ouvre CE compte', ($l = $this->identities->decide($mine))->outcome === IdentityDecision::LINK && $l->userId === $memberId);
            $this->check($io, $failures, 'et plus aucune barrière : /profil 200', $this->status('/profil', $s4) === 200);

            $other = $this->profile('sub-5', $memberEmail, true, 'Quelquun', 'Dautre');
            $s5 = $this->pendingSession($other, $this->identities->decide($other));
            $this->post('/connexion/completer/lier', ['_token' => $this->formToken($this->page('/connexion/completer', $s5), '/connexion/completer/lier')], $s5);
            $this->login4($s5, $memberEmail);
            $c5 = $this->page('/connexion/lier', $s5);
            $this->post('/connexion/lier', ['_token' => $this->formToken($c5, '/connexion/lier'), 'answer' => 'cancel'], $s5);
            $this->check($io, $failures, '« Non, annuler » (poste partagé, identité d’un autre) → rien n’est lié', !$this->db->fetchOne('SELECT 1 FROM EXTERNAL_IDENTITY WHERE subject = ?', ['sub-5']) && $this->status('/profil', $s5) === 200);

            $s6 = $this->pendingSession($this->profile('sub-6', $memberEmail, true, 'X', 'Y'), $this->identities->decide($this->profile('sub-6', $memberEmail, true, 'X', 'Y')));
            $this->login4($s6, $memberEmail);
            $this->check($io, $failures, 'sans le clic « J’ai déjà un compte », une connexion locale ne mène à AUCUNE liaison', $this->status('/profil', $s6) === 200 && !$this->db->fetchOne('SELECT 1 FROM EXTERNAL_IDENTITY WHERE subject = ?', ['sub-6']));

            $io->section('3b. Le lien de confirmation, cliqué au mauvais endroit');
            $pendingUser = $this->identities->provision($this->profile('sub-8', null, false, 'Carol', 'Sonde'), 'sonde-s197-carol-' . bin2hex(random_bytes(3)) . '@example.org', 'Carol', 'Sonde', verified: false);
            $link = fn (): string => '/inscription/activer/' . $this->verifyTokens->create($pendingUser, new \DateTimeImmutable());
            $r = $this->handle(Request::create($link()), $s4);
            $shown = $this->page((string) $r->headers->get('Location'), $s4);
            $this->check($io, $failures, 'navigateur connecté à UN AUTRE compte : l’adresse est confirmée, on reste sur son profil, et la page dit pourquoi', (int) $this->db->fetchOne('SELECT isVerified FROM UTILISATEUR WHERE id = ?', [$pendingUser->getId()]) === 1 && str_ends_with((string) $r->headers->get('Location'), '/profil') && $this->inAnyLocale($shown, 'register_check.activated_other', ['%email%' => $pendingUser->getEmail(), '%current%' => $memberEmail]));
            $this->db->executeStatement('UPDATE UTILISATEUR SET isVerified = 0 WHERE id = ?', [$pendingUser->getId()]);
            $this->entityManager->clear();
            $pendingUser = $this->users->find($pendingUser->getId());
            $anonSession = new Session(new MockArraySessionStorage());
            $r2 = $this->handle(Request::create($link()), $anonSession);
            if ($provisionedColumn) {
                $this->check($io, $failures, 'compte créé par le fournisseur : « connectez-vous avec Sonde S197 », pas un champ mot de passe pré-rempli', str_ends_with((string) $r2->headers->get('Location'), '/login') && $this->inAnyLocale($this->page('/login', $anonSession), 'register_check.activated_provider', ['%provider%' => 'sonde_s197']));
            }

            $io->section('4. Seul le nom manquait');
            $nameless = $this->profile('sub-7', 'sonde-s197-nom-' . bin2hex(random_bytes(3)) . '@example.org', true, null, null);
            $d7 = $this->identities->decide($nameless);
            $s7 = $this->pendingSession($nameless, $d7);
            $p7 = $this->page('/connexion/completer', $s7);
            $this->check($io, $failures, 'la page demande le nom, pas l’e-mail', $d7->needs === ['name'] && str_contains($p7, 'name="firstName"') && !str_contains($p7, 'name="email"'));
            $this->post('/connexion/completer', ['_token' => $this->formToken($p7, '/connexion/completer'), 'firstName' => 'Grace', 'lastName' => ''], $s7);
            $this->check($io, $failures, 'compte ouvert, adresse confirmée, et connecté : /profil 200', (int) $this->db->fetchOne('SELECT isVerified FROM UTILISATEUR WHERE email = ?', [$nameless->email]) === 1 && $this->status('/profil', $s7) === 200);

            $io->section('5. « Mot de passe oublié » d’un compte créé par le fournisseur');
            if (!$provisionedColumn) {
                $io->writeln('   (migration S197 pas encore passée : non mesuré)');
            } else {
                $anon = new Session(new MockArraySessionStorage());
                $form = $this->page('/forgot-password', $anon);
                $token = preg_match('#name="_token"\s+value="([^"]+)"#', $form, $m) ? $m[1] : '';
                $managed = $this->post('/forgot-password', ['_token' => $token, 'email' => $nameless->email], $anon);
                $anon2 = new Session(new MockArraySessionStorage());
                $form2 = $this->page('/forgot-password', $anon2);
                $token2 = preg_match('#name="_token"\s+value="([^"]+)"#', $form2, $m) ? $m[1] : '';
                $local = $this->post('/forgot-password', ['_token' => $token2, 'email' => $memberEmail], $anon2);
                if ($mail) {
                    $this->check($io, $failures, 'le compte créé reçoit « géré par le fournisseur », pas de lien FabOS', $sent((string) $nameless->email) === ['password_managed']);
                    $this->check($io, $failures, 'le compte local LIÉ garde son lien de réinitialisation', \in_array('password_reset', $sent($memberEmail), true));
                }
                $this->check($io, $failures, 'même réponse à l’écran dans les deux cas', $managed->headers->get('Location') === $local->headers->get('Location'));
            }
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
        }

        $io->section('6. Rien n’est resté');
        $after = $counts();
        foreach ($before as $table => $count) {
            $this->check($io, $failures, sprintf('%s : %d avant, %d après', $table, $count, $after[$table]), $after[$table] === $count);
        }

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S197 verte. Transaction annulée.');

        return Command::SUCCESS;
    }

    private function profile(string $subject, ?string $email, bool $verified, ?string $first, ?string $last): ExternalProfile
    {
        return new ExternalProfile(
            providerKey: 'sonde_s197',
            issuer: 'https://idp.sonde-s197.invalid',
            subject: $subject,
            email: $email,
            emailVerified: $verified,
            firstName: $first,
            lastName: $last,
            displayName: null,
            affiliations: [],
            disabledAtSource: false,
            attributes: [],
        );
    }

    /** Ce que `OidcController` range après une connexion validée qui doit être complétée. */
    private function pendingSession(ExternalProfile $profile, IdentityDecision $decision): Session
    {
        $session = new Session(new MockArraySessionStorage());
        $session->start();
        $this->pending->store($session, new AuthProvider('sonde_s197', 'Sonde S197', AuthProvider::KIND_OIDC, $profile->issuer, 'c', 'S197_X', ['openid'], true), $profile, $decision);

        return $session;
    }

    /** Se connecter DANS la session qui porte la demande de liaison. */
    private function login4(Session $session, string $email): void
    {
        $form = $this->page('/login', $session);
        $token = preg_match('#name="_csrf_token"\s+value="([^"]+)"#', $form, $m) ? $m[1] : '';
        $this->handle(Request::create('/login', 'POST', ['_username' => $email, '_password' => self::PASSWORD, '_csrf_token' => $token]), $session);
    }

    private function normalise(string $html, string $email): string
    {
        return (string) preg_replace(['#value="[^"]*"#', '#' . preg_quote(htmlspecialchars($email), '#') . '#'], ['value=""', '@@'], $html);
    }
}
