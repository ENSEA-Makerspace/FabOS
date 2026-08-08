<?php

namespace App\Reservation\Policy;

use App\Entity\Utilisateur;

/**
 * What a person is allowed to be, for booking purposes.
 *
 * Deliberately a small ordered ladder rather than a copy of the role system:
 * roles answer "what may you administer?", tiers answer "how much may you
 * book?". Trainers sit above members because they teach — longer sessions,
 * further ahead — not because they are more trusted with the admin panel.
 *
 * The order matters: a person holding several roles is resolved to the highest
 * tier they qualify for, so promoting someone never accidentally tightens their
 * limits.
 */
enum BookingTier: string
{
    case Member = 'member';
    case Trainer = 'trainer';
    case Staff = 'staff';
    case Admin = 'admin';

    /** Highest first — resolution walks this and takes the first match. */
    public static function ordered(): array
    {
        return [self::Admin, self::Staff, self::Trainer, self::Member];
    }

    public static function forUser(Utilisateur $user): self
    {
        foreach (self::ordered() as $tier) {
            if ($tier === self::Member || $user->hasRoleNamed($tier->value)) {
                return $tier;
            }
        }

        return self::Member;
    }

    public static function tryFromValue(?string $value): ?self
    {
        return $value === null ? null : self::tryFrom($value);
    }

    /** Human label for the admin screen. */
    public function label(): string
    {
        return match ($this) {
            self::Member => 'Membre',
            self::Trainer => 'Formateur',
            self::Staff => 'Équipe',
            self::Admin => 'Administrateur',
        };
    }
}
