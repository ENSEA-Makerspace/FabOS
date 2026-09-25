<?php

declare(strict_types=1);

namespace App\Identity;

/**
 * Un fournisseur de connexion configuré — une ligne de `AUTH_PROVIDER`.
 *
 * 🔴 `secretEnv` est le NOM d'une variable d'environnement, jamais le secret.
 * `settings` porte la correspondance des attributs (`mapping`), le préréglage
 * choisi (`preset`) et la confiance faite aux adresses (`trustEmail`).
 */
final readonly class AuthProvider
{
    public const KIND_OIDC = 'oidc';

    /**
     * @param list<string>         $scopes
     * @param array<string, mixed> $settings
     */
    public function __construct(
        public string $key,
        public string $label,
        public string $kind,
        public string $issuer,
        public string $clientId,
        public string $secretEnv,
        public array $scopes,
        public bool $enabled,
        public array $settings = [],
    ) {
    }

    /** @return array<string, string> champ du profil → nom d'attribut chez le fournisseur */
    public function mapping(): array
    {
        $mapping = \is_array($this->settings['mapping'] ?? null) ? $this->settings['mapping'] : [];

        return array_filter(
            array_map(static fn ($v): string => trim((string) $v), $mapping + AttributeMapping::preset((string) ($this->settings['preset'] ?? AttributeMapping::DEFAULT_PRESET))),
            static fn (string $v): bool => $v !== '',
        );
    }

    public function preset(): string
    {
        return (string) ($this->settings['preset'] ?? AttributeMapping::DEFAULT_PRESET);
    }

    /**
     * Faire confiance aux adresses de ce fournisseur même quand il ne dit pas
     * `email_verified` — une case que l'exploitant coche pour SON établissement.
     */
    public function trustsEmail(): bool
    {
        return (bool) ($this->settings['trustEmail'] ?? false);
    }
}
