<?php

declare(strict_types=1);

namespace App\Security;

use App\Entity\Utilisateur;
use Doctrine\DBAL\Connection;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Session\SessionInterface;

/**
 * S191a — les sessions d'un compte : visibles, et fermables.
 *
 * 🔴 **La clé est à FabOS, pas à PHP.** À l'ouverture, une clé aléatoire est
 * rangée dans la session et seule son empreinte SHA-256 va en base : la table ne
 * permet de reprendre la session de personne, et un identifiant de session PHP
 * régénéré (à la connexion, par exemple) ne casse pas le suivi.
 *
 * ⚠️ **Fermer une session, c'est la marquer** : la vraie coupure a lieu à SA
 * requête suivante, quand `SessionTrackingListener` lit la marque. Les fichiers
 * de session PHP ne sont pas touchés — ils ne sont pas indexés par compte.
 *
 * ⚠️ Silencieux sans sa table (migration S191) : aucune ligne, aucune erreur,
 * et l'écran « Sessions ouvertes » ne s'affiche pas.
 */
final class SessionRegistry
{
    public const SESSION_KEY = '_fabos_session_key';

    /** Une session inactive depuis plus longtemps que PHP ne la garde n'est plus « ouverte ». */
    private const ALIVE_SECONDS = 1440;
    /** On n'écrit « vue à » qu'une fois par minute : pas une écriture par requête. */
    private const TOUCH_EVERY_SECONDS = 60;
    private const KEEP_DAYS = 30;

    private ?bool $ready = null;

    public function __construct(private readonly Connection $db)
    {
    }

    public function isReady(): bool
    {
        if ($this->ready === null) {
            try {
                $this->db->fetchOne('SELECT 1 FROM USER_SESSION LIMIT 1');
                $this->ready = true;
            } catch (\Throwable) {
                $this->ready = false;
            }
        }

        return $this->ready;
    }

    /** Une connexion vient de réussir : ouvrir sa ligne et ranger sa clé. */
    public function open(Utilisateur $user, Request $request, SessionInterface $session): void
    {
        if (!$this->isReady() || $user->getId() === null) {
            return;
        }
        $key = bin2hex(random_bytes(32));
        $this->db->insert('USER_SESSION', [
            'userId' => $user->getId(),
            'keyHash' => hash('sha256', $key),
            'ipPrefix' => self::truncateIp((string) $request->getClientIp()),
            'userAgent' => mb_substr((string) $request->headers->get('User-Agent', ''), 0, 255),
        ]);
        $session->set(self::SESSION_KEY, $key);
        $session->set(self::SESSION_KEY . '_touched', time());

        // Ménage opportuniste : pas de tâche planifiée à oublier.
        if (random_int(1, 20) === 1) {
            // ⚠️ Durées calculées PAR LA BASE : les dates sont écrites par son
            // NOW(), les comparer à une heure PHP mélangerait deux fuseaux.
            $this->db->executeStatement(
                'DELETE FROM USER_SESSION WHERE (revokedAt IS NOT NULL AND revokedAt < NOW() - INTERVAL ' . self::KEEP_DAYS . ' DAY) OR lastSeenAt < NOW() - INTERVAL ' . self::KEEP_DAYS . ' DAY',
            );
        }
    }

    /**
     * @return 'unknown'|'revoked'|'alive' `unknown` : aucune ligne pour cette clé
     *         (session ouverte avant le suivi, ou table vide) — à ouvrir
     */
    public function state(SessionInterface $session): string
    {
        $key = $session->get(self::SESSION_KEY);
        if (!is_string($key) || $key === '' || !$this->isReady()) {
            return 'unknown';
        }
        $row = $this->db->fetchAssociative('SELECT id, revokedAt FROM USER_SESSION WHERE keyHash = :hash', ['hash' => hash('sha256', $key)]);
        if ($row === false) {
            return 'unknown';
        }

        return $row['revokedAt'] !== null ? 'revoked' : 'alive';
    }

    public function touch(SessionInterface $session): void
    {
        $key = $session->get(self::SESSION_KEY);
        $last = (int) $session->get(self::SESSION_KEY . '_touched', 0);
        if (!is_string($key) || !$this->isReady() || time() - $last < self::TOUCH_EVERY_SECONDS) {
            return;
        }
        $this->db->executeStatement('UPDATE USER_SESSION SET lastSeenAt = NOW() WHERE keyHash = :hash AND revokedAt IS NULL', ['hash' => hash('sha256', $key)]);
        $session->set(self::SESSION_KEY . '_touched', time());
    }

    /** La session courante se ferme (déconnexion). */
    public function close(SessionInterface $session): void
    {
        $key = $session->get(self::SESSION_KEY);
        if (is_string($key) && $this->isReady()) {
            $this->db->executeStatement('UPDATE USER_SESSION SET revokedAt = NOW() WHERE keyHash = :hash AND revokedAt IS NULL', ['hash' => hash('sha256', $key)]);
        }
        $session->remove(self::SESSION_KEY);
    }

    /**
     * Les sessions encore vivantes d'un compte, la plus récente d'abord.
     *
     * @return list<array{id: int, createdAt: string, lastSeenAt: string, ipPrefix: ?string, userAgent: ?string, device: array{browser: ?string, os: ?string}, current: bool}>
     */
    public function aliveFor(Utilisateur $user, ?SessionInterface $current = null): array
    {
        if (!$this->isReady()) {
            return [];
        }
        $currentKey = $current?->get(self::SESSION_KEY);
        $currentHash = is_string($currentKey) ? hash('sha256', $currentKey) : null;
        $rows = $this->db->fetchAllAssociative(
            'SELECT id, keyHash, createdAt, lastSeenAt, ipPrefix, userAgent FROM USER_SESSION
             WHERE userId = :user AND revokedAt IS NULL AND lastSeenAt >= NOW() - INTERVAL ' . self::ALIVE_SECONDS . ' SECOND ORDER BY lastSeenAt DESC',
            ['user' => $user->getId()],
        );

        return array_map(static fn (array $row): array => [
            'id' => (int) $row['id'],
            'createdAt' => (string) $row['createdAt'],
            'lastSeenAt' => (string) $row['lastSeenAt'],
            'ipPrefix' => $row['ipPrefix'] !== null ? (string) $row['ipPrefix'] : null,
            'userAgent' => $row['userAgent'] !== null ? (string) $row['userAgent'] : null,
            'device' => self::describeAgent($row['userAgent'] !== null ? (string) $row['userAgent'] : null),
            'current' => $currentHash !== null && hash_equals($currentHash, (string) $row['keyHash']),
        ], $rows);
    }

    /** Fermer UNE session de ce compte — jamais celle d'un autre, même avec son id. */
    public function revoke(Utilisateur $user, int $sessionId): bool
    {
        return $this->isReady() && $this->db->executeStatement(
            'UPDATE USER_SESSION SET revokedAt = NOW() WHERE id = :id AND userId = :user AND revokedAt IS NULL',
            ['id' => $sessionId, 'user' => $user->getId()],
        ) > 0;
    }

    /** @return int sessions fermées — toutes sauf la courante */
    public function revokeOthers(Utilisateur $user, SessionInterface $current): int
    {
        $key = $current->get(self::SESSION_KEY);
        if (!$this->isReady()) {
            return 0;
        }

        return (int) $this->db->executeStatement(
            'UPDATE USER_SESSION SET revokedAt = NOW() WHERE userId = :user AND revokedAt IS NULL AND keyHash <> :hash',
            ['user' => $user->getId(), 'hash' => is_string($key) ? hash('sha256', $key) : ''],
        );
    }

    /** @return int sessions fermées — toutes, y compris celle qui demande */
    public function revokeAll(Utilisateur $user): int
    {
        return $this->isReady() ? (int) $this->db->executeStatement(
            'UPDATE USER_SESSION SET revokedAt = NOW() WHERE userId = :user AND revokedAt IS NULL',
            ['user' => $user->getId()],
        ) : 0;
    }

    /** L'anonymisation efface l'historique des connexions (IP tronquées comprises). */
    public function forget(int $userId): void
    {
        if ($this->isReady()) {
            $this->db->executeStatement('DELETE FROM USER_SESSION WHERE userId = :user', ['user' => $userId]);
        }
    }

    /**
     * « Firefox · macOS » : assez pour reconnaître SON appareil. Volontairement
     * grossier — une liste de navigateurs exhaustive vieillit plus vite qu'elle ne sert.
     *
     * @return array{browser: ?string, os: ?string}
     */
    public static function describeAgent(?string $agent): array
    {
        $agent = (string) $agent;
        $browser = match (true) {
            str_contains($agent, 'Edg/') => 'Edge',
            str_contains($agent, 'OPR/') => 'Opera',
            str_contains($agent, 'Firefox/') => 'Firefox',
            str_contains($agent, 'Chrome/') => 'Chrome',
            str_contains($agent, 'Safari/') => 'Safari',
            default => null,
        };
        $os = match (true) {
            str_contains($agent, 'iPhone'), str_contains($agent, 'iPad') => 'iOS',
            str_contains($agent, 'Android') => 'Android',
            str_contains($agent, 'Mac OS X') => 'macOS',
            str_contains($agent, 'Windows') => 'Windows',
            str_contains($agent, 'Linux') => 'Linux',
            default => null,
        };

        return ['browser' => $browser, 'os' => $os];
    }

    /** 192.168.1.77 → 192.168.1.0/24 ; 2001:db8:1:2::5 → 2001:db8:1::/48. */
    public static function truncateIp(string $ip): ?string
    {
        if (filter_var($ip, FILTER_VALIDATE_IP, FILTER_FLAG_IPV4)) {
            $parts = explode('.', $ip);

            return $parts[0] . '.' . $parts[1] . '.' . $parts[2] . '.0/24';
        }
        if (filter_var($ip, FILTER_VALIDATE_IP, FILTER_FLAG_IPV6)) {
            $packed = inet_pton($ip);
            if ($packed === false) {
                return null;
            }

            return inet_ntop(substr($packed, 0, 6) . str_repeat("\0", 10)) . '/48';
        }

        return null;
    }
}
