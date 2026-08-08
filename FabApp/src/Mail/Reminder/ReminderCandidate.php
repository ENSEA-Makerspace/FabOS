<?php

namespace App\Mail\Reminder;

use App\Entity\Utilisateur;

/**
 * One reminder a scanner believes is due: who to tell, what to say, and the key
 * that decides whether it has already been said.
 *
 * The recipient is either a known user — whose language and notification
 * preference are then honoured — or a bare address, for the borrowers a lab
 * records by hand without an account.
 */
final readonly class ReminderCandidate
{
    /** @param array<string, mixed> $context */
    private function __construct(
        public string $key,
        public string $template,
        public array $context,
        public ?Utilisateur $user,
        public ?string $email,
        public ?string $name,
    ) {
    }

    /** @param array<string, mixed> $context */
    public static function forUser(string $key, Utilisateur $user, string $template, array $context): self
    {
        return new self($key, $template, $context, $user, null, null);
    }

    /** @param array<string, mixed> $context */
    public static function forAddress(string $key, string $email, ?string $name, string $template, array $context): self
    {
        return new self($key, $template, $context, null, $email, $name);
    }

    /** Who this is going to, for the dry-run listing. */
    public function recipient(): string
    {
        return $this->user?->getEmail() ?? (string) $this->email;
    }
}
