<?php

namespace App\Command;

use App\Identity\AttributeMapping;
use App\Identity\AuthProvider;
use App\Identity\ExternalIdentityService;
use App\Identity\IdentityDecision;
use App\Identity\IdentityRefusal;
use App\Identity\OidcModule;
use App\Identity\OidcResult;
use App\Repository\UtilisateurRepository;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Firebase\JWT\JWT;
use Symfony\Component\Cache\Adapter\ArrayAdapter;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpClient\MockHttpClient;
use Symfony\Component\HttpClient\Response\MockResponse;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\KernelInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S196 — le socle des modules de connexion, éprouvé contre un FAUX fournisseur
 * OIDC tenu dans la commande : une vraie paire de clés RSA, un JWKS, un client
 * HTTP simulé. Aucun appel réseau, aucun fournisseur réel.
 *
 *   1. Le jeton valide passe ; chaque falsification est refusée, pour la BONNE
 *      raison (nonce d'une autre connexion, état rejoué, signature d'une autre
 *      clé, `alg: none`, confusion RS256→HS256, audience, émetteur, expiration,
 *      `userinfo` d'une autre personne, découverte incohérente).
 *   2. La rotation des clés : un `kid` inconnu relit le JWKS UNE fois.
 *   3. La décision : e-mail garanti repris, non garanti écarté, déjà pris →
 *      compte DISTINCT (jamais rapproché), statut local désactivé → refus,
 *      deuxième connexion → le même compte.
 *   4. L'écran : sans fournisseur activé, `/login` est identique octet pour
 *      octet ; activé, son bouton paraît (la mesure sait voir une différence).
 *      « Connexion & annuaires » et « Tester » s'ouvrent ; `?test=1` refuse un
 *      visiteur.
 *
 * ✅ Transaction annulée ; comptes de tables comparés avant/après.
 */
#[AsCommand(name: 'app:s196:identity-probe', description: 'S196 : OIDC contre un faux fournisseur (jetons falsifiés refusés, rotation de clés), décisions d’identité, écran de connexion inchangé. Transaction annulée.')]
final class S196IdentityProbeCommand extends Command
{
    use ProbeBrowser;

    private const ISSUER = 'https://idp.sonde-s196.invalid';
    private const CLIENT = 'fabos-sonde';
    private const SECRET_ENV = 'S196_PROBE_CLIENT_SECRET';
    private const REDIRECT = 'https://fabos.sonde/login/oidc/callback';
    private const PASSWORD = 'sonde-S196-motdepasse';

    /** @var array{0: \OpenSSLAsymmetricKey, 1: array<string, string>} */
    private array $k1;
    /** @var array{0: \OpenSSLAsymmetricKey, 1: array<string, string>} */
    private array $k2;
    /** @var array{0: \OpenSSLAsymmetricKey, 1: array<string, string>} */
    private array $k3;
    /** @var list<array<string, string>> */
    private array $published = [];
    private int $jwksFetches = 0;
    /** @var array<string, mixed> */
    private array $discovery = [];
    private string $idToken = '';
    private string $currentSub = '';
    /** @var array<string, mixed>|null */
    private ?array $userinfo = null;

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly ExternalIdentityService $identities,
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
        $_SERVER[self::SECRET_ENV] = 'secret-de-sonde';

        $this->k1 = self::rsa('k1');
        $this->k2 = self::rsa('k1'); // ⚠️ même `kid`, AUTRE clé : l'usurpation
        $this->k3 = self::rsa('k3');
        $this->published = [$this->k1[1]];
        $this->discovery = [
            'issuer' => self::ISSUER,
            'authorization_endpoint' => self::ISSUER . '/auth',
            'token_endpoint' => self::ISSUER . '/token',
            'userinfo_endpoint' => self::ISSUER . '/userinfo',
            'jwks_uri' => self::ISSUER . '/jwks',
        ];

        $tables = ['UTILISATEUR', 'EXTERNAL_IDENTITY', 'AUTH_PROVIDER', 'USER_SESSION'];
        $counts = fn (): array => array_combine($tables, array_map(fn (string $t): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM ' . $t), $tables));
        $before = $counts();

        $provider = $this->provider();
        $http = $this->http();
        $oidc = new OidcModule($http, new ArrayAdapter());

        $this->db->beginTransaction();
        try {
            $io->section('1. Le jeton valide, puis chaque falsification');
            $session = new Session(new MockArraySessionStorage());
            $result = $this->roundTrip($oidc, $provider, $session, []);
            $this->check($io, $failures, 'un jeton valide passe : sub « sonde-sub-1 »', $result instanceof OidcResult && $result->profile->subject === 'sonde-sub-1');
            $this->check($io, $failures, 'l’URL de départ porte state, nonce et PKCE S256', str_contains($this->lastUrl, 'code_challenge_method=S256') && str_contains($this->lastUrl, 'nonce='));

            $plain = $oidc->begin($provider, new Session(new MockArraySessionStorage()), self::REDIRECT);
            $forced = $oidc->begin($provider, new Session(new MockArraySessionStorage()), self::REDIRECT, false, true);
            $tested = $oidc->begin($provider, new Session(new MockArraySessionStorage()), self::REDIRECT, true);
            $this->check($io, $failures, 'prompt=login : absent d’ordinaire, présent si forcé et pour « Tester »', !str_contains($plain, 'prompt=') && str_contains($forced, 'prompt=login') && str_contains($tested, 'prompt=login'));

            $replayed = $this->refusal(fn () => $oidc->complete($session, $this->lastState, 'code', null, self::REDIRECT, fn () => $provider));
            $this->check($io, $failures, 'l’état rejoué → refusé (' . $replayed . ')', $replayed === 'identity.refused.state');

            foreach ([
                'nonce d’une autre connexion' => [['nonce' => 'nonce-dune-autre-connexion'], 'nonce'],
                'audience d’un autre client' => [['aud' => 'un-autre-client'], 'aud'],
                'émetteur différent' => [['iss' => 'https://pirate.invalid'], 'iss'],
                'plusieurs audiences sans azp' => [['aud' => [self::CLIENT, 'autre']], 'azp'],
                'expiré depuis une heure' => [['exp' => time() - 3600, 'iat' => time() - 7200], 'Expired'],
            ] as $what => [$override, $expect]) {
                [$key, $detail] = $this->refusalDetail(fn () => $this->roundTrip($oidc, $provider, new Session(new MockArraySessionStorage()), $override));
                $this->check($io, $failures, sprintf('%s → refusé (%s : %s)', $what, $key, $detail), $key === 'identity.refused.id_token' && str_contains($detail, $expect));
            }

            [$key, $detail] = $this->refusalDetail(fn () => $this->roundTrip($oidc, $provider, new Session(new MockArraySessionStorage()), [], signWith: $this->k2));
            $this->check($io, $failures, '🔴 signé par une AUTRE clé sous le même kid → refusé (' . $detail . ')', $key === 'identity.refused.id_token');

            [$key, $detail] = $this->refusalDetail(fn () => $this->roundTrip($oidc, $provider, new Session(new MockArraySessionStorage()), [], raw: fn (array $claims) => self::b64(['alg' => 'none', 'typ' => 'JWT', 'kid' => 'k1']) . '.' . self::b64($claims) . '.'));
            $this->check($io, $failures, '🔴 alg « none » → refusé (' . $detail . ')', $key === 'identity.refused.id_token');

            [$key, $detail] = $this->refusalDetail(fn () => $this->roundTrip($oidc, $provider, new Session(new MockArraySessionStorage()), [], raw: fn (array $claims) => JWT::encode($claims, self::publicPem($this->k1[0]), 'HS256', 'k1')));
            $this->check($io, $failures, '🔴 HS256 signé avec la clé PUBLIQUE (confusion d’algorithme) → refusé (' . $detail . ')', $key === 'identity.refused.id_token');

            $this->userinfo = ['sub' => 'quelquun-dautre', 'email' => 'x@example.org'];
            $key = $this->refusal(fn () => $this->roundTrip($oidc, $provider, new Session(new MockArraySessionStorage()), []));
            $this->userinfo = null;
            $this->check($io, $failures, 'userinfo d’une autre personne → refusé (' . $key . ')', $key === 'identity.refused.userinfo_sub');

            $badHttp = $this->http(['issuer' => 'https://autre.invalid']);
            $bad = new OidcModule($badHttp, new ArrayAdapter());
            $key = $this->refusal(fn () => $bad->begin($provider, new Session(new MockArraySessionStorage()), self::REDIRECT));
            $this->check($io, $failures, 'découverte dont l’émetteur diffère → refusé (' . $key . ')', $key === 'identity.refused.discovery');

            $io->section('2. La rotation des clés');
            $fetches = $this->jwksFetches;
            $this->published = [$this->k1[1], $this->k3[1]];
            $rotated = $this->roundTrip($oidc, $provider, new Session(new MockArraySessionStorage()), [], signWith: $this->k3);
            $this->check($io, $failures, 'un kid inconnu relit le JWKS une fois, et le jeton passe', $rotated instanceof OidcResult && $this->jwksFetches === $fetches + 1);
            $this->published = [$this->k1[1]];

            $io->section('3. Ce que FabOS décide');
            $first = $this->identities->decide($result->profile);
            $this->check($io, $failures, 'nouvelle identité, adresse garantie → création, adresse reprise', $first->outcome === IdentityDecision::CREATE && $first->email === 'sonde-s196@example.org');
            $user = $this->identities->apply($result->profile);
            $again = $this->identities->decide($result->profile);
            $this->check($io, $failures, 'deuxième connexion → le MÊME compte', $again->outcome === IdentityDecision::LINK && $again->userId === $user->getId());

            $unverified = $this->roundTrip($oidc, $provider, new Session(new MockArraySessionStorage()), ['sub' => 'sonde-sub-2', 'email' => 'sonde-s196-b@example.org', 'email_verified' => false]);
            $d = $this->identities->decide($unverified->profile);
            // S197 : plus de compte à adresse de remplacement — la personne complète.
            $this->check($io, $failures, 'adresse NON garantie → « compléter » (elle sera confirmée), rien de créé', $d->outcome === IdentityDecision::COMPLETE && $d->needs === ['email'] && self::hasNote($d, 'identity.note.email_unverified'));
            $trusting = $this->provider(['trustEmail' => true]);
            $t = $this->identities->decide(AttributeMapping::toProfile($trusting, $unverified->claims));
            $this->check($io, $failures, 'même adresse, fournisseur « de confiance » → reprise', $t->email === 'sonde-s196-b@example.org');

            $local = $this->db->fetchAssociative("SELECT id, email FROM UTILISATEUR WHERE email NOT LIKE '%.invalid' AND id <> ? ORDER BY id LIMIT 1", [$user->getId()]);
            $taken = $this->roundTrip($oidc, $provider, new Session(new MockArraySessionStorage()), ['sub' => 'sonde-sub-3', 'email' => $local['email']]);
            $d = $this->identities->decide($taken->profile);
            $this->check($io, $failures, '🔴 adresse d’un compte local existant → jamais rapprochée : « compléter » (lier avec preuve, ou autre adresse)', $d->outcome === IdentityDecision::COMPLETE && $d->emailTaken && self::hasNote($d, 'identity.note.email_taken'));
            $refused = $this->refusal(fn () => $this->identities->apply($taken->profile));
            $this->check($io, $failures, 'et `apply()` refuse d’ouvrir un compte à compléter (' . $refused . ') ; aucun lien', $refused === 'identity.refused.incomplete' && !$this->db->fetchOne('SELECT 1 FROM EXTERNAL_IDENTITY WHERE userId = ?', [$local['id']]));

            $this->db->executeStatement("UPDATE UTILISATEUR SET statut = 'inactif' WHERE id = ?", [$user->getId()]);
            $d = $this->identities->decide($result->profile);
            $this->check($io, $failures, '🔴 compte lié désactivé ICI → refusé, même si le fournisseur accepte', $d->outcome === IdentityDecision::REFUSE && $d->reason === 'identity.refused.local_inactive');

            $noSubject = $this->refusal(fn () => AttributeMapping::toProfile($provider, ['email' => 'a@b.c']));
            $this->check($io, $failures, 'sans identifiant immuable → refusé (' . $noSubject . ')', $noSubject === 'identity.refused.no_subject');

            $io->section('4. L’écran');
            $this->probeScreens($io, $failures);
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
            unset($_SERVER[self::SECRET_ENV]);
        }

        $io->section('5. Rien n’est resté');
        $after = $counts();
        foreach ($before as $table => $count) {
            $this->check($io, $failures, sprintf('%s : %d avant, %d après', $table, $count, $after[$table]), $after[$table] === $count);
        }

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S196 verte. Transaction annulée.');

        return Command::SUCCESS;
    }

    /** @param list<string> $failures */
    private function probeScreens(SymfonyStyle $io, array &$failures): void
    {
        $normalise = static fn (string $html): string => (string) preg_replace('#name="_csrf_token"\s+value="[^"]*"#', '', $html);
        $anon = new Session(new MockArraySessionStorage());
        $this->db->executeStatement('UPDATE AUTH_PROVIDER SET enabled = 0');
        $baseline = $normalise($this->page('/login', $anon));
        $this->db->executeStatement(
            "INSERT INTO AUTH_PROVIDER (providerKey,label,issuer,clientId,clientSecretEnv,scopes,enabled,createdAt) VALUES ('sonde_s196','Sonde S196',?,?,?,'openid',0,NOW())",
            [self::ISSUER, self::CLIENT, self::SECRET_ENV],
        );
        $this->check($io, $failures, '🔴 un fournisseur DÉSACTIVÉ : /login identique, octet pour octet', $normalise($this->page('/login', new Session(new MockArraySessionStorage()))) === $baseline);
        $this->db->executeStatement("UPDATE AUTH_PROVIDER SET enabled = 1 WHERE providerKey = 'sonde_s196'");
        $withButton = $this->page('/login', new Session(new MockArraySessionStorage()));
        $this->check($io, $failures, 'activé : son bouton paraît (la mesure voit une différence)', str_contains($withButton, '/login/oidc/sonde_s196') && $normalise($withButton) !== $baseline);
        $this->check($io, $failures, '« Tester » refuse un visiteur (?test=1 → connexion)', $this->handle(Request::create('/login/oidc/sonde_s196?test=1'), new Session(new MockArraySessionStorage()))->isRedirect());

        // Après « Déconnexion », le bouton du fournisseur doit redemander le mot
        // de passe (poste partagé) — mesuré sur le vrai fournisseur de test s'il existe.
        // ⚠️ La mesure de /login ci-dessus a tout désactivé : on rallume CELUI-CI (transaction annulée).
        if ($this->db->executeStatement("UPDATE AUTH_PROVIDER SET enabled = 1 WHERE providerKey = 'keycloak_test'") > 0
            || $this->db->fetchOne("SELECT 1 FROM AUTH_PROVIDER WHERE providerKey = 'keycloak_test'")) {
            $out = $this->handle(Request::create('/logout'), new Session(new MockArraySessionStorage()));
            $cookie = null;
            foreach ($out->headers->getCookies() as $c) {
                if ($c->getName() === 'fabos_reauth') {
                    $cookie = $c;
                }
            }
            $this->check($io, $failures, 'la déconnexion pose le signal « redemander le mot de passe »', $cookie !== null && $cookie->getValue() === '1' && $cookie->isHttpOnly());
            $start = Request::create('/login/oidc/keycloak_test');
            $start->cookies->set('fabos_reauth', '1');
            $withSignal = $this->handle($start, new Session(new MockArraySessionStorage()));
            $cleared = array_filter($withSignal->headers->getCookies(), static fn ($c) => $c->getName() === 'fabos_reauth' && $c->isCleared());
            $this->check($io, $failures, '🔴 le clic suivant part avec prompt=login, et consomme le signal', str_contains((string) $withSignal->headers->get('Location'), 'prompt=login') && $cleared !== []);
            $without = $this->handle(Request::create('/login/oidc/keycloak_test'), new Session(new MockArraySessionStorage()));
            $this->check($io, $failures, 'sans signal : pas de prompt (la mesure voit la différence)', $without->isRedirect() && !str_contains((string) $without->headers->get('Location'), 'prompt='));
        } else {
            $io->writeln('   (fournisseur keycloak_test absent : déconnexion → prompt=login non mesurée)');
        }

        $admin = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $admin = $candidate;
                break;
            }
        }
        if ($admin === null) {
            $this->check($io, $failures, 'un administrateur actif pour ouvrir l’écran', false);

            return;
        }
        // ⚠️ En SQL : après les requêtes simulées, le gestionnaire d'entités a été
        // remis à zéro et un `flush()` n'écrirait rien.
        $this->db->executeStatement('UPDATE UTILISATEUR SET password = ? WHERE id = ?', [$this->hasher->hashPassword($admin, self::PASSWORD), $admin->getId()]);
        $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId = ?', [$admin->getId()]);
        $session = $this->login($admin->getEmail(), self::PASSWORD);
        $list = $this->handle(Request::create('/admin/connexion'), $session);
        $this->check($io, $failures, '« Connexion & annuaires » s’ouvre et liste le fournisseur (' . $list->getStatusCode() . ')', $list->getStatusCode() === 200 && str_contains((string) $list->getContent(), 'Sonde S196'));
        $this->check($io, $failures, 'la fiche du fournisseur s’ouvre et montre l’adresse de retour', $this->status('/admin/connexion/sonde_s196', $session) === 200 && str_contains($this->page('/admin/connexion/sonde_s196', $session), '/login/oidc/callback'));
        $this->check($io, $failures, 'la page « Tester » s’ouvre', $this->status('/admin/connexion/sonde_s196/test', $session) === 200);
        $this->check($io, $failures, '« Réseau » ne porte plus de formulaire OIDC', !str_contains($this->page('/admin/network', $session), 'secretEnv'));

        // Enregistrer par le FORMULAIRE, comme un exploitant : un champ de la
        // correspondance corrigé, la confiance cochée, puis relu en base.
        $form = $this->page('/admin/connexion/nouveau', $session);
        $token = preg_match('#name="auth_provider\[_token\]"[^>]*value="([^"]+)"#', $form, $m) ? $m[1] : '';
        $saved = $this->post('/admin/connexion/nouveau', ['auth_provider' => [
            '_token' => $token, 'label' => 'Sonde formulaire', 'key' => 'sonde_form', 'issuer' => 'https://idp-form.sonde-s196.invalid',
            'clientId' => 'client-form', 'secretEnv' => 'S196_FORM_SECRET', 'scopes' => 'profile email', 'preset' => 'entra_id',
            'trustEmail' => '1', 'map_email' => 'upn',
        ]], $session);
        preg_match_all('#<div class="form-errors">(.*?)</div>#s', (string) $saved->getContent(), $errors);
        $errors = trim(strip_tags(implode(' ', $errors[1])));
        $row = $this->db->fetchAssociative("SELECT * FROM AUTH_PROVIDER WHERE providerKey = 'sonde_form'");
        $this->check($io, $failures, 'le formulaire enregistre (' . $saved->getStatusCode() . ($errors !== '' ? ' : ' . $errors : '') . '), désactivé par défaut, « openid » ajouté', $row !== false && (int) $row['enabled'] === 0 && str_starts_with((string) $row['scopes'], 'openid'));
        if ($row !== false && \array_key_exists('settingsJson', $row)) {
            $settings = json_decode((string) $row['settingsJson'], true);
            $this->check($io, $failures, 'préréglage, confiance et champ corrigé rangés (' . $row['settingsJson'] . ')', ($settings['preset'] ?? null) === 'entra_id' && ($settings['trustEmail'] ?? null) === true && ($settings['mapping'] ?? null) === ['email' => 'upn']);
            $provider = $this->registry()->find('sonde_form');
            $this->check($io, $failures, 'relu : e-mail ← « upn », identifiant ← « oid » (préréglage)', $provider !== null && $provider->mapping()['email'] === 'upn' && $provider->mapping()['subject'] === 'oid');
        } else {
            $io->writeln('   (migration S196 pas encore passée : réglages avancés non mesurés)');
        }
        $refused = $this->post('/admin/connexion/nouveau', ['auth_provider' => [
            '_token' => $token, 'label' => 'x', 'key' => 'sonde_form2', 'issuer' => 'https://idp-form2.sonde-s196.invalid',
            'clientId' => 'c', 'secretEnv' => 'le-secret-colle-ici', 'preset' => 'oidc_standard',
        ]], $session);
        $this->check($io, $failures, 'un secret COLLÉ à la place d’un nom de variable → refusé, pour CETTE raison (' . $refused->getStatusCode() . ')', $refused->getStatusCode() === 422 && $this->inAnyLocale((string) $refused->getContent(), 'identity.invalid.secret_env', []) && !$this->db->fetchOne("SELECT 1 FROM AUTH_PROVIDER WHERE providerKey = 'sonde_form2'"));
    }

    private function registry(): \App\Identity\ProviderRegistry
    {
        return new \App\Identity\ProviderRegistry($this->db);
    }

    private string $lastUrl = '';
    private string $lastState = '';

    /**
     * Un aller-retour complet : `begin()`, puis le jeton que le faux fournisseur
     * aurait émis, puis `complete()`.
     *
     * @param array<string, mixed> $override
     */
    private function roundTrip(OidcModule $oidc, AuthProvider $provider, Session $session, array $override, ?array $signWith = null, ?callable $raw = null): OidcResult
    {
        $this->lastUrl = $oidc->begin($provider, $session, self::REDIRECT);
        parse_str((string) parse_url($this->lastUrl, PHP_URL_QUERY), $query);
        $this->lastState = (string) $query['state'];
        $claims = $override + [
            'iss' => self::ISSUER, 'aud' => self::CLIENT, 'sub' => 'sonde-sub-1', 'nonce' => (string) $query['nonce'],
            'iat' => time(), 'exp' => time() + 300, 'email' => 'sonde-s196@example.org', 'email_verified' => true,
            'given_name' => 'Sonde', 'family_name' => 'S196', 'name' => 'Sonde S196',
        ];
        $this->currentSub = (string) $claims['sub'];
        $key = $signWith ?? $this->k1;
        $this->idToken = $raw !== null ? $raw($claims) : JWT::encode($claims, $key[0], 'RS256', $key[1]['kid']);

        return $oidc->complete($session, $this->lastState, 'code-de-sonde', null, self::REDIRECT, fn () => $provider);
    }

    /** @param array<string, mixed> $discovery */
    private function http(array $discovery = []): MockHttpClient
    {
        return new MockHttpClient(function (string $method, string $url, array $options) use ($discovery): MockResponse {
            $json = static fn (array $body): MockResponse => new MockResponse(json_encode($body, JSON_THROW_ON_ERROR), ['response_headers' => ['content-type' => 'application/json']]);

            return match (true) {
                str_ends_with($url, '/.well-known/openid-configuration') => $json($discovery + $this->discovery),
                str_ends_with($url, '/jwks') => (function () use ($json) { ++$this->jwksFetches; return $json(['keys' => $this->published]); })(),
                str_ends_with($url, '/token') => $json(['access_token' => 'at-de-sonde', 'token_type' => 'Bearer', 'id_token' => $this->idToken]),
                str_ends_with($url, '/userinfo') => $json($this->userinfo ?? ['sub' => $this->currentSub]),
                default => new MockResponse('', ['http_code' => 404]),
            };
        });
    }

    /** @param array<string, mixed> $settings */
    private function provider(array $settings = []): AuthProvider
    {
        return new AuthProvider('sonde_s196', 'Sonde S196', AuthProvider::KIND_OIDC, self::ISSUER, self::CLIENT, self::SECRET_ENV, ['openid', 'email', 'profile'], true, $settings);
    }

    private function refusal(callable $do): string
    {
        return $this->refusalDetail($do)[0];
    }

    /** @return array{0: string, 1: string} */
    private function refusalDetail(callable $do): array
    {
        try {
            $do();

            return ['(accepté !)', ''];
        } catch (IdentityRefusal $e) {
            return [$e->reasonKey, (string) $e->detail];
        }
    }

    private static function hasNote(IdentityDecision $d, string $key): bool
    {
        return \in_array($key, array_column($d->notes, 0), true);
    }

    /** @return array{0: \OpenSSLAsymmetricKey, 1: array<string, string>} */
    private static function rsa(string $kid): array
    {
        $key = openssl_pkey_new(['private_key_bits' => 2048, 'private_key_type' => OPENSSL_KEYTYPE_RSA]);
        $details = openssl_pkey_get_details($key);
        $b64 = static fn (string $v): string => rtrim(strtr(base64_encode($v), '+/', '-_'), '=');

        return [$key, ['kty' => 'RSA', 'kid' => $kid, 'use' => 'sig', 'alg' => 'RS256', 'n' => $b64($details['rsa']['n']), 'e' => $b64($details['rsa']['e'])]];
    }

    private static function publicPem(\OpenSSLAsymmetricKey $key): string
    {
        return (string) openssl_pkey_get_details($key)['key'];
    }

    /** @param array<string, mixed> $data */
    private static function b64(array $data): string
    {
        return rtrim(strtr(base64_encode(json_encode($data, JSON_THROW_ON_ERROR)), '+/', '-_'), '=');
    }
}
