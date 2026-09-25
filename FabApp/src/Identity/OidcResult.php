<?php

declare(strict_types=1);

namespace App\Identity;

/** Le retour d'un fournisseur OIDC, validé : le profil traduit, et les claims bruts pour « Tester ». */
final readonly class OidcResult
{
    /** @param array<string, mixed> $claims */
    public function __construct(
        public AuthProvider $provider,
        public ExternalProfile $profile,
        public array $claims,
        public bool $test,
    ) {
    }
}
