<?php

declare(strict_types=1);

namespace App\Identity;

use Doctrine\DBAL\Connection;

/**
 * Les fournisseurs de connexion configurés (`AUTH_PROVIDER`).
 *
 * ⚠️ Lit `kind` et `settingsJson` s'ils existent (migration S196) et retombe
 * sur « OIDC, préréglage standard » sinon : une installation qui n'a pas encore
 * migré garde ses fournisseurs tels qu'ils marchaient.
 */
final class ProviderRegistry
{
    private ?bool $extended = null;

    public function __construct(private readonly Connection $db)
    {
    }

    /** @return list<AuthProvider> */
    public function enabled(): array
    {
        return array_values(array_filter($this->all(), static fn (AuthProvider $p): bool => $p->enabled));
    }

    /** @return list<AuthProvider> */
    public function all(): array
    {
        try {
            return array_map($this->hydrate(...), $this->db->fetchAllAssociative('SELECT * FROM AUTH_PROVIDER ORDER BY label'));
        } catch (\Throwable) {
            return [];
        }
    }

    public function find(string $key): ?AuthProvider
    {
        $row = $this->db->fetchAssociative('SELECT * FROM AUTH_PROVIDER WHERE providerKey = :key', ['key' => $key]);

        return $row ? $this->hydrate($row) : null;
    }

    /** Comptes liés à chaque fournisseur (liens actifs), pour l'écran. @return array<string, int> */
    public function linkedCounts(): array
    {
        try {
            return array_map('intval', $this->db->fetchAllKeyValue('SELECT providerKey, COUNT(*) FROM EXTERNAL_IDENTITY WHERE revokedAt IS NULL GROUP BY providerKey'));
        } catch (\Throwable) {
            return [];
        }
    }

    /** La migration S196 est-elle passée ? Sans elle, on ne sait pas ranger les réglages. */
    public function isExtended(): bool
    {
        if ($this->extended === null) {
            try {
                $this->db->fetchOne('SELECT kind, settingsJson FROM AUTH_PROVIDER LIMIT 1');
                $this->extended = true;
            } catch (\Throwable) {
                $this->extended = false;
            }
        }

        return $this->extended;
    }

    /**
     * Créer ou mettre à jour un fournisseur.
     *
     * @param array<string, mixed> $settings
     *
     * @throws \InvalidArgumentException sur une configuration impossible
     */
    public function save(string $key, string $label, string $kind, string $issuer, string $clientId, string $secretEnv, string $scopes, bool $enabled, array $settings): void
    {
        $issuer = rtrim(trim($issuer), '/');
        if (!preg_match('/^[a-z][a-z0-9_]{1,79}$/', $key)) {
            throw new \InvalidArgumentException('identity.invalid.key');
        }
        if ($kind !== AuthProvider::KIND_OIDC) {
            throw new \InvalidArgumentException('identity.invalid.kind');
        }
        if (!preg_match('#^https://[^/\s]+(?:/\S*)?$#', $issuer)) {
            throw new \InvalidArgumentException('identity.invalid.issuer');
        }
        if (!preg_match('/^[A-Z][A-Z0-9_]{2,119}$/', $secretEnv)) {
            throw new \InvalidArgumentException('identity.invalid.secret_env');
        }
        $scopes = trim(preg_replace('/\s+/', ' ', $scopes) ?? '');
        if (!\in_array('openid', explode(' ', $scopes), true)) {
            $scopes = trim('openid ' . $scopes);
        }

        $params = [
            'key' => $key, 'label' => trim($label), 'issuer' => $issuer, 'client' => trim($clientId),
            'secret' => $secretEnv, 'scopes' => $scopes, 'enabled' => $enabled ? 1 : 0,
        ];
        if ($this->isExtended()) {
            $this->db->executeStatement(
                'INSERT INTO AUTH_PROVIDER (providerKey,label,kind,issuer,clientId,clientSecretEnv,scopes,enabled,settingsJson,createdAt)'
                . ' VALUES (:key,:label,:kind,:issuer,:client,:secret,:scopes,:enabled,:settings,NOW())'
                . ' ON DUPLICATE KEY UPDATE label=VALUES(label),kind=VALUES(kind),issuer=VALUES(issuer),clientId=VALUES(clientId),'
                . 'clientSecretEnv=VALUES(clientSecretEnv),scopes=VALUES(scopes),enabled=VALUES(enabled),settingsJson=VALUES(settingsJson)',
                $params + ['kind' => $kind, 'settings' => json_encode($settings, JSON_UNESCAPED_UNICODE | JSON_THROW_ON_ERROR)],
            );

            return;
        }
        $this->db->executeStatement(
            'INSERT INTO AUTH_PROVIDER (providerKey,label,issuer,clientId,clientSecretEnv,scopes,enabled,createdAt)'
            . ' VALUES (:key,:label,:issuer,:client,:secret,:scopes,:enabled,NOW())'
            . ' ON DUPLICATE KEY UPDATE label=VALUES(label),issuer=VALUES(issuer),clientId=VALUES(clientId),'
            . 'clientSecretEnv=VALUES(clientSecretEnv),scopes=VALUES(scopes),enabled=VALUES(enabled)',
            $params,
        );
    }

    public function setEnabled(string $key, bool $enabled): void
    {
        $this->db->executeStatement('UPDATE AUTH_PROVIDER SET enabled = ? WHERE providerKey = ?', [$enabled ? 1 : 0, $key]);
    }

    /** @param array<string, mixed> $row */
    private function hydrate(array $row): AuthProvider
    {
        $settings = isset($row['settingsJson']) && \is_string($row['settingsJson']) ? json_decode($row['settingsJson'], true) : null;

        return new AuthProvider(
            key: (string) $row['providerKey'],
            label: (string) $row['label'],
            kind: (string) ($row['kind'] ?? AuthProvider::KIND_OIDC),
            issuer: (string) $row['issuer'],
            clientId: (string) $row['clientId'],
            secretEnv: (string) $row['clientSecretEnv'],
            scopes: preg_split('/\s+/', trim((string) $row['scopes'])) ?: [],
            enabled: (bool) $row['enabled'],
            settings: \is_array($settings) ? $settings : [],
        );
    }
}
