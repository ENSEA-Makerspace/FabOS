<?php
namespace App\Identity;
use Doctrine\DBAL\Connection;
final class ProviderRegistry
{
    public function __construct(private readonly Connection $db) {}
    /** @return list<OidcProvider> */
    public function enabled(): array { return array_values(array_filter(array_map($this->hydrate(...), $this->db->fetchAllAssociative('SELECT * FROM AUTH_PROVIDER ORDER BY label')), static fn(OidcProvider $p) => $p->enabled)); }
    public function find(string $key): ?OidcProvider { $row = $this->db->fetchAssociative('SELECT * FROM AUTH_PROVIDER WHERE providerKey = :key', ['key' => $key]); return $row ? $this->hydrate($row) : null; }
    public function save(string $key, string $label, string $issuer, string $clientId, string $secretEnv, bool $enabled): void
    {
        $issuer = rtrim($issuer, '/');
        if (!preg_match('#^https://[^/]+(?:/.*)?$#', $issuer) || !preg_match('/^[a-z][a-z0-9_]{1,79}$/', $key) || !preg_match('/^[A-Z][A-Z0-9_]{2,119}$/', $secretEnv)) { throw new \InvalidArgumentException('Configuration OIDC invalide.'); }
        $this->db->executeStatement('INSERT INTO AUTH_PROVIDER (providerKey,label,issuer,clientId,clientSecretEnv,scopes,enabled,createdAt) VALUES (:key,:label,:issuer,:client,:secret,:scopes,:enabled,NOW()) ON DUPLICATE KEY UPDATE label=VALUES(label),issuer=VALUES(issuer),clientId=VALUES(clientId),clientSecretEnv=VALUES(clientSecretEnv),enabled=VALUES(enabled)', ['key'=>$key,'label'=>trim($label),'issuer'=>$issuer,'client'=>trim($clientId),'secret'=>$secretEnv,'scopes'=>'openid profile email','enabled'=>$enabled?1:0]);
    }
    private function hydrate(array $row): OidcProvider { return new OidcProvider((string)$row['providerKey'],(string)$row['label'],(string)$row['issuer'],(string)$row['clientId'],(string)$row['clientSecretEnv'],preg_split('/\s+/',trim((string)$row['scopes']))?:[],(bool)$row['enabled']); }
}
