<?php

declare(strict_types=1);

namespace App\Identity;

/**
 * Ce que FabOS ferait d'un `ExternalProfile` — lier, créer ou refuser — et
 * POURQUOI, en phrases (clés `identity.note.*`). « Tester » l'affiche tel quel.
 */
final readonly class IdentityDecision
{
    public const LINK = 'link';
    public const CREATE = 'create';
    public const REFUSE = 'refuse';

    /** @param list<array{0: string, 1: array<string, string>}> $notes */
    private function __construct(
        public string $outcome,
        public ?int $userId = null,
        public ?string $userLabel = null,
        public ?string $email = null,
        public ?string $firstName = null,
        public ?string $lastName = null,
        public ?string $reason = null,
        public array $notes = [],
    ) {
    }

    public static function link(int $userId, string $label): self
    {
        return new self(self::LINK, userId: $userId, userLabel: $label);
    }

    /** @param list<array{0: string, 1: array<string, string>}> $notes */
    public static function create(?string $email, ?string $firstName, ?string $lastName, array $notes): self
    {
        return new self(self::CREATE, email: $email, firstName: $firstName, lastName: $lastName, notes: $notes);
    }

    public static function refuse(string $reason): self
    {
        return new self(self::REFUSE, reason: $reason);
    }
}
