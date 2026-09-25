<?php

declare(strict_types=1);

namespace App\Identity;

/**
 * S196 — LE contrat : ce qu'un module de connexion (OIDC, puis LDAP, AD, CAS,
 * SAML) remet au reste de FabOS, une fois SA réponse traduite.
 *
 * 🔴 Aucun protocole ne fuit au-delà : `ExternalIdentityService` ne connaît que
 * cette forme. Un module neuf n'écrit donc AUCUNE règle d'identité — il remplit
 * ceci, et c'est tout.
 *
 * ⚠️ `subject` est l'identifiant IMMUABLE chez le fournisseur (`sub`,
 * `entryUUID`, `objectGUID`, `eduPersonPrincipalName`…) : c'est lui, avec
 * `issuer`, qui désigne la personne. Jamais l'e-mail.
 */
final readonly class ExternalProfile
{
    /**
     * @param list<string>          $affiliations lues, affichables — 🔴 jamais des droits
     * @param array<string, mixed>  $attributes   tout ce que le fournisseur a envoyé (pour « Tester »)
     */
    public function __construct(
        public string $providerKey,
        public string $issuer,
        public string $subject,
        public ?string $email,
        public bool $emailVerified,
        public ?string $firstName,
        public ?string $lastName,
        public ?string $displayName,
        public array $affiliations,
        public bool $disabledAtSource,
        public array $attributes,
    ) {
    }
}
