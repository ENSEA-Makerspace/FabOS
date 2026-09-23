<?php

declare(strict_types=1);

namespace App\Security;

/**
 * S191b — TOTP (RFC 6238, sur HOTP RFC 4226) : les codes à 6 chiffres des
 * applications d'authentification.
 *
 * Écrit ici plutôt qu'importé : c'est vingt lignes de HMAC-SHA1, et la sonde les
 * éprouve contre les VECTEURS DE TEST de la RFC elle-même.
 *
 * ⚠️ `hash_equals` pour comparer un code : jamais `===`.
 * ⚠️ Une fenêtre de ±1 pas (30 s) pour les horloges de téléphone un peu à côté,
 * et `verify()` rend le PAS accepté : l'appelant le garde et refuse ensuite tout
 * pas inférieur ou égal — un code déjà servi ne se rejoue pas.
 */
final class Totp
{
    public const PERIOD = 30;
    public const DIGITS = 6;
    private const ALPHABET = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ234567';

    /** Un secret de 160 bits, en base32 (ce que les applications attendent). */
    public static function newSecret(): string
    {
        return self::base32Encode(random_bytes(20));
    }

    public static function code(string $base32Secret, int $step, int $digits = self::DIGITS, string $algo = 'sha1'): string
    {
        $key = self::base32Decode($base32Secret);
        $mac = hash_hmac($algo, pack('J', $step), $key, true);
        $offset = ord($mac[strlen($mac) - 1]) & 0x0F;
        $binary = ((ord($mac[$offset]) & 0x7F) << 24)
            | (ord($mac[$offset + 1]) << 16)
            | (ord($mac[$offset + 2]) << 8)
            | ord($mac[$offset + 3]);

        return str_pad((string) ($binary % (10 ** $digits)), $digits, '0', STR_PAD_LEFT);
    }

    public static function stepAt(int $timestamp): int
    {
        return intdiv($timestamp, self::PERIOD);
    }

    /**
     * @return int|null le pas accepté, ou null si le code ne vaut rien — faux,
     *                  hors fenêtre, ou déjà servi (pas ≤ `$lastUsedStep`)
     */
    public static function verify(string $base32Secret, string $code, int $timestamp, ?int $lastUsedStep = null): ?int
    {
        $code = preg_replace('/\s+/', '', $code) ?? '';
        if (!preg_match('/^\d{' . self::DIGITS . '}$/', $code)) {
            return null;
        }
        $now = self::stepAt($timestamp);
        foreach ([$now, $now - 1, $now + 1] as $step) {
            if ($lastUsedStep !== null && $step <= $lastUsedStep) {
                continue;
            }
            if (hash_equals(self::code($base32Secret, $step), $code)) {
                return $step;
            }
        }

        return null;
    }

    /** L'adresse que lit un QR code d'application d'authentification. */
    public static function provisioningUri(string $base32Secret, string $issuer, string $account): string
    {
        return 'otpauth://totp/' . rawurlencode($issuer) . ':' . rawurlencode($account)
            . '?secret=' . $base32Secret
            . '&issuer=' . rawurlencode($issuer)
            . '&algorithm=SHA1&digits=' . self::DIGITS . '&period=' . self::PERIOD;
    }

    public static function base32Encode(string $raw): string
    {
        $bits = '';
        foreach (str_split($raw) as $char) {
            $bits .= str_pad(decbin(ord($char)), 8, '0', STR_PAD_LEFT);
        }
        $out = '';
        foreach (str_split($bits, 5) as $chunk) {
            $out .= self::ALPHABET[bindec(str_pad($chunk, 5, '0', STR_PAD_RIGHT))];
        }

        return $out;
    }

    public static function base32Decode(string $encoded): string
    {
        $encoded = strtoupper(preg_replace('/[\s=-]+/', '', $encoded) ?? '');
        $bits = '';
        foreach (str_split($encoded) as $char) {
            $value = strpos(self::ALPHABET, $char);
            if ($value === false) {
                throw new \InvalidArgumentException('Clé base32 invalide.');
            }
            $bits .= str_pad(decbin($value), 5, '0', STR_PAD_LEFT);
        }
        $out = '';
        foreach (str_split($bits, 8) as $byte) {
            if (strlen($byte) === 8) {
                $out .= chr(bindec($byte));
            }
        }

        return $out;
    }
}
