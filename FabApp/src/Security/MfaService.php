<?php

declare(strict_types=1);

namespace App\Security;

use App\Entity\Utilisateur;
use Doctrine\DBAL\Connection;

/**
 * S191b — le second facteur d'un compte : TOTP, et des codes de secours.
 *
 * 🔴 **Le secret est CHIFFRÉ en base** (libsodium secretbox ; la clé est dérivée
 * d'APP_SECRET avec un contexte propre) : une copie de la table ne donne les
 * codes de personne. ⚠️ Faire tourner APP_SECRET rend donc tous les seconds
 * facteurs illisibles — les comptes concernés passent par la réinitialisation
 * admin (`reset()`), qui existe pour ça.
 *
 * 🔴 **Les codes de secours sont HACHÉS et à usage unique.** Ils ne s'affichent
 * qu'une fois, à la génération.
 *
 * ⚠️ Trois états : `none` ; `pending` (inscription commencée, pas encore un code
 * accepté — le second facteur n'est PAS exigé, sinon un QR mal scanné
 * enfermerait la personne dehors) ; `enabled`.
 *
 * ⚠️ Silencieux sans sa table (migration S191) : tout le monde est `none`.
 */
final class MfaService
{
    public const NONE = 'none';
    public const PENDING = 'pending';
    public const ENABLED = 'enabled';
    public const RECOVERY_COUNT = 10;

    private ?bool $ready = null;

    public function __construct(
        private readonly Connection $db,
        private readonly string $secret,
    ) {
    }

    public function isReady(): bool
    {
        if ($this->ready === null) {
            try {
                $this->db->fetchOne('SELECT 1 FROM USER_MFA LIMIT 1');
                $this->ready = true;
            } catch (\Throwable) {
                $this->ready = false;
            }
        }

        return $this->ready;
    }

    public function status(Utilisateur $user): string
    {
        $row = $this->row($user);
        if ($row === null) {
            return self::NONE;
        }

        return $row['enabledAt'] !== null ? self::ENABLED : self::PENDING;
    }

    /** Commencer (ou recommencer) une inscription : un secret neuf, pas encore exigé. */
    public function start(Utilisateur $user): string
    {
        if ($this->status($user) === self::ENABLED) {
            throw new \LogicException('Déjà activée : la désactiver d\'abord.');
        }
        $secret = Totp::newSecret();
        $this->db->executeStatement(
            'REPLACE INTO USER_MFA (userId, secretEncrypted, enabledAt, lastUsedStep, recoveryCodes) VALUES (:user, :secret, NULL, NULL, NULL)',
            ['user' => $user->getId(), 'secret' => $this->seal($secret)],
        );

        return $secret;
    }

    /** Le secret d'une inscription EN COURS, pour réafficher le QR. Jamais une fois activée. */
    public function pendingSecret(Utilisateur $user): ?string
    {
        $row = $this->row($user);

        return $row !== null && $row['enabledAt'] === null ? $this->open((string) $row['secretEncrypted']) : null;
    }

    /**
     * Le premier code accepté active le second facteur.
     *
     * @return list<string>|null les codes de secours, EN CLAIR, une seule fois ; null si le code est faux
     */
    public function confirm(Utilisateur $user, string $code, ?int $now = null): ?array
    {
        $row = $this->row($user);
        if ($row === null || $row['enabledAt'] !== null) {
            return null;
        }
        $step = Totp::verify((string) $this->open((string) $row['secretEncrypted']), $code, $now ?? time());
        if ($step === null) {
            return null;
        }
        $codes = $this->newRecoveryCodes();
        $this->db->executeStatement(
            'UPDATE USER_MFA SET enabledAt = NOW(), lastUsedStep = :step, recoveryCodes = :codes WHERE userId = :user',
            ['step' => $step, 'codes' => json_encode(array_map($this->hashCode(...), $codes)), 'user' => $user->getId()],
        );

        return $codes;
    }

    /**
     * À la connexion : un code de l'application, OU un code de secours (consommé).
     *
     * @return 'totp'|'recovery'|null
     */
    public function verify(Utilisateur $user, string $code, ?int $now = null): ?string
    {
        $row = $this->row($user);
        if ($row === null || $row['enabledAt'] === null) {
            return null;
        }
        $secret = $this->open((string) $row['secretEncrypted']);
        if ($secret === null) {
            return null;
        }
        $step = Totp::verify($secret, $code, $now ?? time(), $row['lastUsedStep'] !== null ? (int) $row['lastUsedStep'] : null);
        if ($step !== null) {
            // ⚠️ Conditionnel : deux requêtes concurrentes avec le même code ne
            // peuvent pas toutes les deux avancer le pas.
            $moved = $this->db->executeStatement(
                'UPDATE USER_MFA SET lastUsedStep = :step WHERE userId = :user AND (lastUsedStep IS NULL OR lastUsedStep < :step)',
                ['step' => $step, 'user' => $user->getId()],
            );

            return $moved > 0 ? 'totp' : null;
        }

        $hashes = json_decode((string) ($row['recoveryCodes'] ?? '[]'), true) ?: [];
        $candidate = $this->hashCode($code);
        foreach ($hashes as $i => $hash) {
            if (is_string($hash) && hash_equals($hash, $candidate)) {
                unset($hashes[$i]);
                $this->db->executeStatement('UPDATE USER_MFA SET recoveryCodes = :codes WHERE userId = :user', ['codes' => json_encode(array_values($hashes)), 'user' => $user->getId()]);

                return 'recovery';
            }
        }

        return null;
    }

    public function recoveryLeft(Utilisateur $user): int
    {
        $row = $this->row($user);

        return $row === null ? 0 : \count(json_decode((string) ($row['recoveryCodes'] ?? '[]'), true) ?: []);
    }

    /** @return list<string>|null de nouveaux codes (les anciens meurent), si le code donné est bon */
    public function regenerate(Utilisateur $user, string $code): ?array
    {
        if ($this->verify($user, $code) === null) {
            return null;
        }
        $codes = $this->newRecoveryCodes();
        $this->db->executeStatement('UPDATE USER_MFA SET recoveryCodes = :codes WHERE userId = :user', ['codes' => json_encode(array_map($this->hashCode(...), $codes)), 'user' => $user->getId()]);

        return $codes;
    }

    /** Désactiver soi-même : il faut un code valide (on ne coupe pas le second facteur avec le seul mot de passe). */
    public function disable(Utilisateur $user, string $code): bool
    {
        if ($this->verify($user, $code) === null) {
            return false;
        }
        $this->reset($user);

        return true;
    }

    /** Téléphone perdu, plus de codes de secours : l'équipe retire le second facteur. */
    public function reset(Utilisateur $user): void
    {
        if ($this->isReady()) {
            $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId = :user', ['user' => $user->getId()]);
        }
    }

    public function forget(int $userId): void
    {
        if ($this->isReady()) {
            $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId = :user', ['user' => $userId]);
        }
    }

    /** @return array<string, mixed>|null */
    private function row(Utilisateur $user): ?array
    {
        if (!$this->isReady() || $user->getId() === null) {
            return null;
        }
        $row = $this->db->fetchAssociative('SELECT * FROM USER_MFA WHERE userId = :user', ['user' => $user->getId()]);

        return $row === false ? null : $row;
    }

    /** @return list<string> « ABCD-EFGH-JK » : 50 bits chacun, lisibles, sans 0/O ni 1/I */
    private function newRecoveryCodes(): array
    {
        $alphabet = 'ABCDEFGHJKLMNPQRSTUVWXYZ23456789';
        $codes = [];
        for ($i = 0; $i < self::RECOVERY_COUNT; ++$i) {
            $raw = '';
            for ($j = 0; $j < 10; ++$j) {
                $raw .= $alphabet[random_int(0, \strlen($alphabet) - 1)];
            }
            $codes[] = substr($raw, 0, 4) . '-' . substr($raw, 4, 4) . '-' . substr($raw, 8, 2);
        }

        return $codes;
    }

    /**
     * ⚠️ HMAC et pas un simple SHA-256 : un code de secours n'a que 50 bits
     * d'aléa, qu'une copie de la table permettrait de retrouver par force brute.
     * Avec une clé dérivée d'APP_SECRET, la copie seule ne sert à rien.
     */
    private function hashCode(string $code): string
    {
        return hash_hmac('sha256', strtoupper(preg_replace('/[^A-Za-z0-9]/', '', $code) ?? ''), 'fabos-mfa-recovery-v1' . $this->secret);
    }

    private function key(): string
    {
        return sodium_crypto_generichash('fabos-mfa-v1' . $this->secret, '', SODIUM_CRYPTO_SECRETBOX_KEYBYTES);
    }

    private function seal(string $plain): string
    {
        $nonce = random_bytes(SODIUM_CRYPTO_SECRETBOX_NONCEBYTES);

        return base64_encode($nonce . sodium_crypto_secretbox($plain, $nonce, $this->key()));
    }

    private function open(string $sealed): ?string
    {
        $raw = base64_decode($sealed, true);
        if ($raw === false || \strlen($raw) <= SODIUM_CRYPTO_SECRETBOX_NONCEBYTES) {
            return null;
        }
        $plain = sodium_crypto_secretbox_open(substr($raw, SODIUM_CRYPTO_SECRETBOX_NONCEBYTES), substr($raw, 0, SODIUM_CRYPTO_SECRETBOX_NONCEBYTES), $this->key());

        return $plain === false ? null : $plain;
    }
}
