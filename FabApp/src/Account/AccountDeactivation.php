<?php

declare(strict_types=1);

namespace App\Account;

use App\Entity\Utilisateur;
use App\Repository\ReservationRepository;
use App\Security\SessionRegistry;
use Doctrine\ORM\EntityManagerInterface;

/**
 * S190d — désactiver un compte depuis sa fiche, et le réactiver.
 *
 * S190 a rendu le statut `inactif` décisif (badge aux machines et aux portes,
 * sessions ouvertes, API) ; il manquait l'écran qui le pose. Le voici, en un
 * seul endroit, pour que le bouton, la sonde et toute future surface
 * désactivent de la même façon.
 *
 * **Décision de l'opérateur (2026-09-24)** : les réservations À VENIR sont
 * annulées, et l'écran le dit à côté du bouton. Les passées restent (elles
 * sont les statistiques). ⚠️ **Sans courrier** : une réservation annulée = un
 * courrier ferait dix messages à quelqu'un qu'on vient d'écarter.
 * ⚠️ Réactiver ne ressuscite rien : les créneaux ont pu être repris.
 */
final class AccountDeactivation
{
    public const REFUSED_SELF = 'self';
    public const REFUSED_LAST_ADMIN = AccountGuard::REFUSED_LAST_ADMIN;
    public const REFUSED_ANONYMISED = AccountGuard::REFUSED_ALREADY;

    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly ReservationRepository $reservations,
        private readonly SessionRegistry $sessions,
        private readonly AccountGuard $guard,
    ) {
    }

    public static function isInactive(Utilisateur $user): bool
    {
        return \in_array(mb_strtolower(trim($user->getStatut())), ['inactif', 'inactive'], true);
    }

    /** @return string|null un `REFUSED_*`, ou null si la désactivation peut se faire */
    public function refusalFor(Utilisateur $user, ?Utilisateur $actor): ?string
    {
        if ($actor !== null && $actor->getId() === $user->getId()) {
            return self::REFUSED_SELF;
        }

        // Le même verdict que l'anonymisation : un compte effacé n'a plus
        // rien à désactiver, et on ne retire pas le dernier administrateur.
        return $this->guard->refusalFor($user);
    }

    /** @return int réservations annulées */
    public function deactivate(Utilisateur $user): int
    {
        $user->setStatut('inactif');
        $this->em->flush();
        $cancelled = $this->reservations->cancelUpcomingForUser($user);
        $this->sessions->revokeAll($user);

        return $cancelled;
    }

    public function reactivate(Utilisateur $user): void
    {
        $user->setStatut('actif');
        $this->em->flush();
    }
}
