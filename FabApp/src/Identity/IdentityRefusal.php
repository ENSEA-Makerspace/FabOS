<?php

declare(strict_types=1);

namespace App\Identity;

/**
 * Une connexion externe REFUSÉE, avec sa raison lisible (une clé de
 * traduction `identity.refused.*`) — pour que l'écran de connexion et « Tester »
 * disent POURQUOI au lieu d'une page 500.
 *
 * ⚠️ Le détail technique (`detail`) ne va qu'à « Tester », réservé à
 * l'administration : un membre refusé lit la phrase, pas la trace.
 */
final class IdentityRefusal extends \RuntimeException
{
    /** @param array<string, string> $params */
    public function __construct(
        public readonly string $reasonKey,
        public readonly array $params = [],
        public readonly ?string $detail = null,
        ?\Throwable $previous = null,
    ) {
        parent::__construct($reasonKey . ($detail !== null ? ': ' . $detail : ''), 0, $previous);
    }
}
