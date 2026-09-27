<?php

declare(strict_types=1);

namespace App\Identity;

/**
 * Ce que FabOS ferait d'un `ExternalProfile` — lier, créer, faire compléter
 * ou refuser — et POURQUOI, en phrases (clés `identity.note.*`). « Tester »
 * l'affiche tel quel.
 *
 * S197 — `COMPLETE` : il manque de quoi ouvrir un compte sans impasse (une
 * adresse réelle, parfois un nom). La page « Complétez votre compte » ne
 * demande QUE `needs`.
 */
final readonly class IdentityDecision
{
    public const LINK = 'link';
    public const CREATE = 'create';
    public const COMPLETE = 'complete';
    public const REFUSE = 'refuse';

    public const NEED_EMAIL = 'email';
    public const NEED_NAME = 'name';

    /**
     * @param list<array{0: string, 1: array<string, string>}> $notes
     * @param list<string>                                      $needs
     */
    private function __construct(
        public string $outcome,
        public ?int $userId = null,
        public ?string $userLabel = null,
        public ?string $email = null,
        public ?string $firstName = null,
        public ?string $lastName = null,
        public ?string $reason = null,
        public array $notes = [],
        public array $needs = [],
        public bool $emailTaken = false,
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

    /**
     * @param list<string>                                      $needs
     * @param list<array{0: string, 1: array<string, string>}> $notes
     */
    public static function complete(array $needs, ?string $firstName, ?string $lastName, bool $emailTaken, array $notes): self
    {
        return new self(self::COMPLETE, firstName: $firstName, lastName: $lastName, notes: $notes, needs: $needs, emailTaken: $emailTaken);
    }

    public static function refuse(string $reason): self
    {
        return new self(self::REFUSE, reason: $reason);
    }
}
