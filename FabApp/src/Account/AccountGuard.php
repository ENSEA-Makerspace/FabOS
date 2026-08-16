<?php

declare(strict_types=1);

namespace App\Account;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;

/**
 * Whether an account may be erased — the verdict, separate from the erasure.
 *
 * Same split as `VenueGuard` (S129): one class decides, one class acts. The
 * reason is that the decision has to be readable from a template — to hide or
 * explain a disabled control — while the act must run exactly once, and mixing
 * them produces a guard that is consulted for the button and forgotten at the
 * chokepoint.
 *
 * 🔴 **The invariant that matters: an install must never lose its last
 * administrator.** The roadmap has named "dernier Admin" as a lockout risk since
 * Phase B. Erasure is a worse version of it than archiving, because there is no
 * undo — anonymisation is irreversible by design, so a mistaken one cannot be
 * walked back by an operator who has just locked themselves out.
 */
final class AccountGuard
{
    public const REFUSED_LAST_ADMIN = 'last_admin';
    public const REFUSED_ALREADY = 'already_anonymised';

    public function __construct(private readonly UtilisateurRepository $users)
    {
    }

    /** @return string|null a `REFUSED_*` reason, or null when the erasure may proceed */
    public function refusalFor(Utilisateur $user): ?string
    {
        if (AccountAnonymiser::isAnonymised($user)) {
            return self::REFUSED_ALREADY;
        }

        if ($this->isAdmin($user) && $this->otherActiveAdminExists($user) === false) {
            return self::REFUSED_LAST_ADMIN;
        }

        return null;
    }

    public function allows(Utilisateur $user): bool
    {
        return $this->refusalFor($user) === null;
    }

    private function isAdmin(Utilisateur $user): bool
    {
        return \in_array('ROLE_ADMIN', $user->getRoles(), true);
    }

    /**
     * ⚠️ Counted over *active, non-anonymised* accounts, not over the role table.
     * An install whose only other administrator is already erased or set inactive
     * has no other administrator, and a count that says otherwise is the lockout.
     */
    private function otherActiveAdminExists(Utilisateur $user): bool
    {
        foreach ($this->users->findAll() as $other) {
            if ($other->getId() === $user->getId()) {
                continue;
            }
            if ($other->getStatut() === 'actif'
                && !AccountAnonymiser::isAnonymised($other)
                && \in_array('ROLE_ADMIN', $other->getRoles(), true)) {
                return true;
            }
        }

        return false;
    }
}
