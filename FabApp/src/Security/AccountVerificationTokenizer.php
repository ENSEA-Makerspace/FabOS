<?php

declare(strict_types=1);

namespace App\Security;

use App\Entity\Utilisateur;

/**
 * S189 — le lien d'activation d'un compte, sans table.
 *
 * Même construction que `PasswordResetTokenizer` (HMAC keyé par APP_SECRET,
 * expiration DANS la charge signée, base64url), et pour la même raison : une
 * migration se lance à la main, un jeton signé part dans la même livraison que
 * les écrans qui l'utilisent.
 *
 * 🔴 **Séparation des usages.** La clé de signature est dérivée avec le préfixe
 * `verify` : un lien de réinitialisation ne vaut JAMAIS activation, ni
 * l'inverse, même si les deux charges se ressemblaient un jour.
 *
 * ✅ **Usage unique sans comptabilité** : l'empreinte porte sur l'adresse ET sur
 * l'état « vérifié ». Activer le compte, ou corriger son adresse, fait tomber
 * tous les liens déjà envoyés — dont celui parti vers l'adresse mal tapée.
 */
final class AccountVerificationTokenizer
{
    /** 48 heures : le temps d'un week-end, pas une clé qui traîne dans une boîte. */
    public const TTL_SECONDS = 172800;

    public function __construct(private readonly string $secret)
    {
    }

    public function create(Utilisateur $user, \DateTimeImmutable $now): string
    {
        $payload = \sprintf('%d.%d.%s', (int) $user->getId(), $now->getTimestamp() + self::TTL_SECONDS, $this->fingerprint($user));

        return $this->encode($payload) . '.' . $this->encode($this->sign($payload));
    }

    /** @return int|null l'id du compte, ou null si le lien est faux, altéré ou expiré */
    public function userIdIfValid(string $token, \DateTimeImmutable $now): ?int
    {
        $fields = $this->fields($token);
        if ($fields === null || (int) $fields[1] < $now->getTimestamp()) {
            return null;
        }

        return (int) $fields[0] > 0 ? (int) $fields[0] : null;
    }

    /** Le lien a-t-il été émis pour CE compte, dans son état ACTUEL ? */
    public function matchesAccount(string $token, Utilisateur $user): bool
    {
        $fields = $this->fields($token);

        return $fields !== null && hash_equals($this->fingerprint($user), $fields[2]);
    }

    /** @return array{0: string, 1: string, 2: string}|null */
    private function fields(string $token): ?array
    {
        $parts = explode('.', $token);
        if (count($parts) !== 2) {
            return null;
        }
        $payload = $this->decode($parts[0]);
        $mac = $this->decode($parts[1]);
        // ⚠️ `hash_equals`, jamais `===` : l'attaquant tient les deux côtés.
        if ($payload === null || $mac === null || !hash_equals($this->sign($payload), $mac)) {
            return null;
        }
        $fields = explode('.', $payload);

        return count($fields) === 3 ? [$fields[0], $fields[1], $fields[2]] : null;
    }

    private function fingerprint(Utilisateur $user): string
    {
        return substr(hash_hmac('sha256', $user->getEmail() . "\0" . ($user->isVerified() ? '1' : '0'), $this->key()), 0, 16);
    }

    private function sign(string $payload): string
    {
        return hash_hmac('sha256', $payload, $this->key(), true);
    }

    private function key(): string
    {
        return hash_hmac('sha256', 'verify', $this->secret, true);
    }

    private function encode(string $raw): string
    {
        return rtrim(strtr(base64_encode($raw), '+/', '-_'), '=');
    }

    private function decode(string $encoded): ?string
    {
        $decoded = base64_decode(strtr($encoded, '-_', '+/'), true);

        return $decoded === false ? null : $decoded;
    }
}
