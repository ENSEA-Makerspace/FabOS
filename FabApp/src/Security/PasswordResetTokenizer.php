<?php

declare(strict_types=1);

namespace App\Security;

use App\Entity\Utilisateur;

/**
 * Password-reset tokens, without a table.
 *
 * ⚠️ **Why stateless rather than a `password_reset_token` row.** The roadmap's
 * exit criteria for S134g are "hashed, expiring, single-use". A signed token
 * gives all three without a schema change — which matters here for a reason
 * beyond convenience: a migration on this project has to be run by the operator
 * by hand, so a design needing one cannot ship in the same pass as the screens
 * that use it. This one can.
 *
 *   hashed      the token is an HMAC-SHA256 over the payload, keyed with
 *               APP_SECRET. It carries no secret itself and cannot be forged.
 *   expiring    the expiry is IN the signed payload, so it cannot be edited.
 *   single-use  the payload commits to a fingerprint of the account's CURRENT
 *               password hash and e-mail. Using the link changes the password,
 *               which changes the hash, which makes every outstanding token for
 *               that account fail. No bookkeeping, and it also revokes older
 *               links the moment a newer reset succeeds.
 *
 * ⚠️ `hash_equals`, never `===`. Comparing MACs with a short-circuiting operator
 * leaks their contents by timing, and this is the one comparison in the file
 * that an attacker controls both sides of.
 *
 * ⚠️ The token goes in a URL, so it is base64url — no `+`, `/` or `=` to be
 * mangled by a mail client that decides to wrap or re-encode the line.
 */
final class PasswordResetTokenizer
{
    /**
     * One hour. Long enough to walk to a different machine and read the mail,
     * short enough that a link left in an inbox is not a standing key.
     */
    public const TTL_SECONDS = 3600;

    public function __construct(private readonly string $secret)
    {
    }

    public function create(Utilisateur $user, \DateTimeImmutable $now): string
    {
        $payload = \sprintf(
            '%d.%d.%s',
            (int) $user->getId(),
            $now->getTimestamp() + self::TTL_SECONDS,
            $this->fingerprint($user),
        );

        return $this->encode($payload) . '.' . $this->encode($this->sign($payload));
    }

    /**
     * @return int|null the user id the token is for, or null if it is not valid
     *                  for this user at this instant
     */
    public function userIdIfValid(string $token, \DateTimeImmutable $now): ?int
    {
        $parts = explode('.', $token);
        if (count($parts) !== 2) {
            return null;
        }

        $payload = $this->decode($parts[0]);
        $mac = $this->decode($parts[1]);
        if ($payload === null || $mac === null) {
            return null;
        }

        if (!hash_equals($this->sign($payload), $mac)) {
            return null;
        }

        $fields = explode('.', $payload);
        if (count($fields) !== 3) {
            return null;
        }

        [$userId, $expiresAt] = [(int) $fields[0], (int) $fields[1]];

        // ⚠️ Checked AFTER the signature, deliberately: an unsigned token has no
        // trustworthy expiry to check in the first place.
        if ($expiresAt < $now->getTimestamp()) {
            return null;
        }

        return $userId > 0 ? $userId : null;
    }

    /**
     * The token commits to this, and it is verified again at the moment the new
     * password is written — so a link stops working the instant the account's
     * password or e-mail changes by any other route.
     */
    public function matchesAccount(string $token, Utilisateur $user): bool
    {
        $parts = explode('.', $token);
        $payload = count($parts) === 2 ? $this->decode($parts[0]) : null;
        if ($payload === null) {
            return false;
        }

        $fields = explode('.', $payload);

        return count($fields) === 3 && hash_equals($this->fingerprint($user), $fields[2]);
    }

    private function fingerprint(Utilisateur $user): string
    {
        return substr(hash_hmac('sha256', $user->getPassword() . "\0" . $user->getEmail(), $this->secret), 0, 16);
    }

    private function sign(string $payload): string
    {
        return hash_hmac('sha256', $payload, $this->secret, true);
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
