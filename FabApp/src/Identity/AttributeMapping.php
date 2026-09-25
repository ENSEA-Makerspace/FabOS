<?php

declare(strict_types=1);

namespace App\Identity;

/**
 * S196 — traduire les attributs d'un fournisseur en `ExternalProfile`.
 *
 * 🔴 **La correspondance est une CONFIGURATION**, pas du code : chaque
 * fournisseur dit quel attribut porte l'identifiant immuable, l'e-mail, le
 * nom… Les préréglages couvrent les cas usuels ; l'exploitant corrige un
 * champ sans lire une ligne de PHP, et « Tester » lui montre le résultat.
 *
 * ⚠️ Seuls les préréglages d'OIDC existent aujourd'hui : ceux de LDAP
 * (`inetOrgPerson`, SUPANN), d'Active Directory, de CAS et de SAML/eduPerson
 * arrivent avec leurs modules (S198–S200) — proposer un préréglage pour un
 * protocole qu'on ne sait pas encore parler serait une affordance morte.
 */
final class AttributeMapping
{
    public const DEFAULT_PRESET = 'oidc_standard';

    /** Les champs du profil qu'une correspondance peut remplir, dans l'ordre de l'écran. */
    public const FIELDS = ['subject', 'email', 'emailVerified', 'firstName', 'lastName', 'displayName', 'affiliations'];

    /** @var array<string, array{kind: string, mapping: array<string, string>}> */
    private const PRESETS = [
        // Les claims standard d'OpenID Connect Core §5.1 (Keycloak, Authentik,
        // Google, Okta…). `sub` est stable par client : c'est l'identifiant.
        'oidc_standard' => ['kind' => AuthProvider::KIND_OIDC, 'mapping' => [
            'subject' => 'sub', 'email' => 'email', 'emailVerified' => 'email_verified',
            'firstName' => 'given_name', 'lastName' => 'family_name', 'displayName' => 'name',
        ]],
        // Microsoft Entra ID : `oid` est l'identifiant de l'objet dans le
        // locataire, le même pour toutes les applications ; Entra n'envoie pas
        // `email_verified` — d'où la case « faire confiance aux adresses ».
        'entra_id' => ['kind' => AuthProvider::KIND_OIDC, 'mapping' => [
            'subject' => 'oid', 'email' => 'email', 'firstName' => 'given_name',
            'lastName' => 'family_name', 'displayName' => 'name',
        ]],
    ];

    /** @return array<string, string> */
    public static function preset(string $key): array
    {
        return self::PRESETS[$key]['mapping'] ?? self::PRESETS[self::DEFAULT_PRESET]['mapping'];
    }

    /** @return list<string> */
    public static function presetsFor(string $kind): array
    {
        return array_keys(array_filter(self::PRESETS, static fn (array $p): bool => $p['kind'] === $kind));
    }

    /**
     * @param array<string, mixed> $attributes ce que le fournisseur a envoyé
     *
     * @throws IdentityRefusal sans identifiant immuable : on ne sait pas QUI c'est
     */
    public static function toProfile(AuthProvider $provider, array $attributes): ExternalProfile
    {
        $mapping = $provider->mapping();
        $read = static function (string $field) use ($mapping, $attributes): mixed {
            $name = $mapping[$field] ?? null;

            return $name !== null && \array_key_exists($name, $attributes) ? $attributes[$name] : null;
        };
        // Un annuaire rend souvent des listes (LDAP est multi-valué) : on prend la première.
        $text = static function (mixed $value): ?string {
            if (\is_array($value)) {
                $value = reset($value);
            }
            $value = \is_scalar($value) ? trim((string) $value) : '';

            return $value !== '' ? $value : null;
        };

        $subject = $text($read('subject'));
        if ($subject === null) {
            throw new IdentityRefusal('identity.refused.no_subject', ['%attribute%' => $mapping['subject'] ?? '—']);
        }
        $email = $text($read('email'));
        $email = $email !== null && filter_var($email, FILTER_VALIDATE_EMAIL) ? mb_strtolower($email) : null;
        $verified = $read('emailVerified');
        $verified = $verified === true || $verified === 'true' || $verified === 1 || $verified === '1';
        $affiliations = $read('affiliations');
        $affiliations = \is_array($affiliations) ? $affiliations : ($affiliations !== null ? [$affiliations] : []);

        return new ExternalProfile(
            providerKey: $provider->key,
            issuer: $provider->issuer,
            subject: $subject,
            email: $email,
            // ⚠️ « garantie » = le fournisseur l'affirme, OU l'exploitant a dit
            // faire confiance à CE fournisseur. Jamais par défaut.
            emailVerified: $email !== null && ($verified || $provider->trustsEmail()),
            firstName: $text($read('firstName')),
            lastName: $text($read('lastName')),
            displayName: $text($read('displayName')),
            affiliations: array_values(array_filter(array_map($text, $affiliations))),
            disabledAtSource: false,
            attributes: $attributes,
        );
    }
}
