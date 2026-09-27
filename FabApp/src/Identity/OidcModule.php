<?php

declare(strict_types=1);

namespace App\Identity;

use Firebase\JWT\JWK;
use Firebase\JWT\JWT;
use Symfony\Component\HttpFoundation\Session\SessionInterface;
use Symfony\Contracts\Cache\CacheInterface;
use Symfony\Contracts\Cache\ItemInterface;
use Symfony\Contracts\HttpClient\HttpClientInterface;

/**
 * S196 — OpenID Connect (code + PKCE), réécrit sur le contrat `ExternalProfile`.
 *
 * Les trous de la première version, bouchés un par un :
 *   - 🔴 le `nonce` était tiré puis jamais relu → il est comparé à celui de
 *     l'`id_token` : un jeton émis pour une AUTRE connexion est refusé ;
 *   - 🔴 l'`id_token` n'était pas validé (seul `userinfo` était lu) → signature
 *     vérifiée contre les clés publiées par le fournisseur (JWKS), émetteur,
 *     audience, `azp`, expiration. L'algorithme vient de la CLÉ, jamais de
 *     l'en-tête du jeton (pas de `alg: none`, pas de confusion RS/HS) ;
 *   - `email_verified` était ignoré → il décide si l'adresse est reprise ;
 *   - la découverte était refaite à chaque connexion → une heure en cache, et
 *     les clés sont relues UNE fois si le jeton cite une clé inconnue (rotation).
 *
 * Refus fermé : tout écart lève `IdentityRefusal`, jamais « on laisse passer ».
 */
final class OidcModule
{
    private const FLOW_PREFIX = 'oidc_flow_';
    private const FLOW_TTL = 600;
    private const LEEWAY = 60;
    private const CACHE_TTL = 3600;

    public function __construct(
        private readonly HttpClientInterface $http,
        private readonly CacheInterface $cache,
    ) {
    }

    /** L'adresse où envoyer le navigateur ; l'état de la connexion attend dans la session. */
    /**
     * @param bool $forceLogin `prompt=login` : le fournisseur redemande le mot de
     *                         passe même s'il a encore une session ouverte dans
     *                         ce navigateur (après une déconnexion, et pour « Tester »)
     */
    public function begin(AuthProvider $provider, SessionInterface $session, string $redirectUri, bool $test = false, bool $forceLogin = false): string
    {
        $discovery = $this->discovery($provider);
        $state = bin2hex(random_bytes(24));
        $nonce = bin2hex(random_bytes(24));
        $verifier = rtrim(strtr(base64_encode(random_bytes(48)), '+/', '-_'), '=');
        $session->set(self::FLOW_PREFIX . $state, [
            'provider' => $provider->key, 'nonce' => $nonce, 'verifier' => $verifier, 'created' => time(), 'test' => $test,
        ]);

        return $discovery['authorization_endpoint'] . (str_contains($discovery['authorization_endpoint'], '?') ? '&' : '?') . http_build_query([
            'response_type' => 'code',
            'client_id' => $provider->clientId,
            'redirect_uri' => $redirectUri,
            'scope' => implode(' ', $provider->scopes),
            'state' => $state,
            'nonce' => $nonce,
            'code_challenge' => rtrim(strtr(base64_encode(hash('sha256', $verifier, true)), '+/', '-_'), '='),
            'code_challenge_method' => 'S256',
        ] + ($forceLogin || $test ? ['prompt' => 'login'] : []));
    }

    /** La clé du fournisseur si ce retour est celui d'un « Tester », sans consommer l'état. */
    public function testFlow(SessionInterface $session, string $state): ?string
    {
        $flow = $state !== '' ? $session->get(self::FLOW_PREFIX . $state) : null;

        return \is_array($flow) && ($flow['test'] ?? false) ? (string) $flow['provider'] : null;
    }

    /**
     * Le retour du fournisseur : l'état est CONSOMMÉ (un seul usage), le code
     * échangé, l'`id_token` validé, puis les attributs traduits.
     *
     * @param callable(string): ?AuthProvider $find
     *
     * @throws IdentityRefusal
     */
    public function complete(SessionInterface $session, string $state, string $code, ?string $error, string $redirectUri, callable $find): OidcResult
    {
        $flow = $state !== '' ? $session->remove(self::FLOW_PREFIX . $state) : null;
        if (!\is_array($flow) || time() - (int) ($flow['created'] ?? 0) > self::FLOW_TTL) {
            throw new IdentityRefusal('identity.refused.state');
        }
        $test = (bool) ($flow['test'] ?? false);
        $provider = $find((string) $flow['provider']);
        // « Tester » marche sur un fournisseur pas encore activé : c'est à ça qu'il sert.
        if (!$provider instanceof AuthProvider || (!$provider->enabled && !$test)) {
            throw new IdentityRefusal('identity.refused.provider_off');
        }
        if ($error !== null && $error !== '') {
            throw new IdentityRefusal('identity.refused.provider_error', [], $error);
        }
        $secret = $_SERVER[$provider->secretEnv] ?? $_ENV[$provider->secretEnv] ?? getenv($provider->secretEnv);
        if (!\is_string($secret) || $secret === '') {
            throw new IdentityRefusal('identity.refused.secret_missing', ['%env%' => $provider->secretEnv]);
        }

        $discovery = $this->discovery($provider);
        try {
            $tokens = $this->http->request('POST', $discovery['token_endpoint'], [
                'body' => [
                    'grant_type' => 'authorization_code', 'code' => $code, 'redirect_uri' => $redirectUri,
                    'client_id' => $provider->clientId, 'client_secret' => $secret, 'code_verifier' => (string) $flow['verifier'],
                ],
                'timeout' => 8, 'max_redirects' => 0,
            ])->toArray();
        } catch (\Throwable $e) {
            throw new IdentityRefusal('identity.refused.token', [], $e->getMessage(), $e);
        }
        $idToken = $tokens['id_token'] ?? null;
        if (!\is_string($idToken) || $idToken === '') {
            throw new IdentityRefusal('identity.refused.id_token', [], 'no id_token');
        }

        $claims = $this->validateIdToken($provider, $discovery, $idToken, (string) $flow['nonce']);

        // `userinfo` complète souvent l'e-mail et le nom. 🔴 Son `sub` doit être
        // celui du jeton (OIDC Core §5.3.2), sinon on refuse.
        $access = $tokens['access_token'] ?? null;
        if (isset($discovery['userinfo_endpoint']) && \is_string($access) && $access !== '') {
            try {
                $info = $this->http->request('GET', $discovery['userinfo_endpoint'], ['auth_bearer' => $access, 'timeout' => 8, 'max_redirects' => 0])->toArray();
            } catch (\Throwable) {
                $info = null; // l'id_token validé suffit ; userinfo n'est qu'un complément
            }
            if (\is_array($info)) {
                if (($info['sub'] ?? null) !== ($claims['sub'] ?? null)) {
                    throw new IdentityRefusal('identity.refused.userinfo_sub');
                }
                $claims = $info + $claims;
            }
        }

        return new OidcResult($provider, AttributeMapping::toProfile($provider, $claims), $claims, $test);
    }

    /**
     * @param array<string, mixed> $discovery
     *
     * @return array<string, mixed> les claims du jeton, validés
     */
    private function validateIdToken(AuthProvider $provider, array $discovery, string $idToken, string $nonce): array
    {
        $decode = function (bool $fresh) use ($provider, $discovery, $idToken): array {
            $previous = JWT::$leeway;
            JWT::$leeway = self::LEEWAY;
            try {
                $keys = $this->keys($provider, (string) $discovery['jwks_uri'], $fresh);
                // Un jeton sans `kid` n'est acceptable que face à UNE seule clé :
                // il n'y a alors rien à choisir. Avec plusieurs, on refuse.
                $header = json_decode(JWT::urlsafeB64Decode(explode('.', $idToken)[0]), true);
                if (\is_array($header) && !isset($header['kid']) && \count($keys) === 1) {
                    $keys = reset($keys);
                }

                return (array) JWT::decode($idToken, $keys);
            } finally {
                JWT::$leeway = $previous;
            }
        };
        try {
            try {
                $claims = $decode(false);
            } catch (\UnexpectedValueException $e) {
                // Une clé inconnue : le fournisseur a peut-être tourné ses clés. Une relecture, pas deux.
                if (!str_contains($e->getMessage(), '"kid"')) {
                    throw $e;
                }
                $claims = $decode(true);
            }
        } catch (IdentityRefusal $e) {
            throw $e;
        } catch (\Throwable $e) {
            throw new IdentityRefusal('identity.refused.id_token', [], $e->getMessage(), $e);
        }

        $aud = $claims['aud'] ?? null;
        $audiences = \is_array($aud) ? $aud : [$aud];
        $problem = match (true) {
            ($claims['iss'] ?? null) !== $provider->issuer => 'iss',
            !\in_array($provider->clientId, $audiences, true) => 'aud',
            \count($audiences) > 1 && ($claims['azp'] ?? null) !== $provider->clientId => 'azp',
            !\is_string($claims['nonce'] ?? null) || !hash_equals($nonce, $claims['nonce']) => 'nonce',
            !isset($claims['exp'], $claims['iat']) => 'exp/iat',
            default => null,
        };
        if ($problem !== null) {
            throw new IdentityRefusal('identity.refused.id_token', [], $problem);
        }

        return json_decode(json_encode($claims, JSON_THROW_ON_ERROR), true, 512, JSON_THROW_ON_ERROR);
    }

    /** @return array<string, \Firebase\JWT\Key> */
    private function keys(AuthProvider $provider, string $jwksUri, bool $fresh): array
    {
        $key = 'oidc_jwks_' . hash('sha256', $provider->issuer . "\0" . $jwksUri);
        if ($fresh) {
            $this->cache->delete($key);
        }
        $jwks = $this->cache->get($key, function (ItemInterface $item) use ($jwksUri): array {
            $item->expiresAfter(self::CACHE_TTL);
            try {
                $set = $this->http->request('GET', $jwksUri, ['timeout' => 5, 'max_redirects' => 0])->toArray();
            } catch (\Throwable $e) {
                throw new IdentityRefusal('identity.refused.jwks', [], $e->getMessage(), $e);
            }
            // Seules les clés de SIGNATURE comptent.
            $set['keys'] = array_values(array_filter((array) ($set['keys'] ?? []), static fn ($k): bool => \is_array($k) && ($k['use'] ?? 'sig') === 'sig'));

            return $set;
        });
        if (($jwks['keys'] ?? []) === []) {
            $this->cache->delete($key);
            throw new IdentityRefusal('identity.refused.jwks', [], 'empty key set');
        }

        return JWK::parseKeySet($jwks, 'RS256');
    }

    /**
     * La découverte, gardée une heure. 🔴 Son `issuer` doit être EXACTEMENT celui
     * configuré, et chaque point d'accès en https.
     *
     * @return array<string, mixed>
     */
    public function discovery(AuthProvider $provider): array
    {
        $key = 'oidc_discovery_' . hash('sha256', $provider->issuer);
        $load = function () use ($provider): array {
            try {
                $discovery = $this->http->request('GET', $provider->issuer . '/.well-known/openid-configuration', ['timeout' => 5, 'max_redirects' => 0])->toArray();
            } catch (\Throwable $e) {
                throw new IdentityRefusal('identity.refused.discovery', [], $e->getMessage(), $e);
            }
            if (($discovery['issuer'] ?? null) !== $provider->issuer) {
                throw new IdentityRefusal('identity.refused.discovery', [], 'issuer mismatch: ' . (string) ($discovery['issuer'] ?? '—'));
            }
            foreach (['authorization_endpoint', 'token_endpoint', 'jwks_uri'] as $field) {
                if (!\is_string($discovery[$field] ?? null) || !str_starts_with($discovery[$field], 'https://')) {
                    throw new IdentityRefusal('identity.refused.discovery', [], 'missing ' . $field);
                }
            }
            if (isset($discovery['userinfo_endpoint']) && (!\is_string($discovery['userinfo_endpoint']) || !str_starts_with($discovery['userinfo_endpoint'], 'https://'))) {
                unset($discovery['userinfo_endpoint']);
            }

            return $discovery;
        };

        return $this->cache->get($key, function (ItemInterface $item) use ($load): array {
            $item->expiresAfter(self::CACHE_TTL);

            return $load();
        });
    }

    /** « Tester » repart d'une découverte fraîche : une erreur de réglage se corrige sans attendre une heure. */
    public function forget(AuthProvider $provider): void
    {
        $this->cache->delete('oidc_discovery_' . hash('sha256', $provider->issuer));
    }
}
