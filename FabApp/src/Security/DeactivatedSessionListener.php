<?php

namespace App\Security;

use App\Entity\Utilisateur;
use Symfony\Bundle\SecurityBundle\Security;
use Symfony\Component\EventDispatcher\Attribute\AsEventListener;
use Symfony\Component\HttpFoundation\JsonResponse;
use Symfony\Component\HttpFoundation\RedirectResponse;
use Symfony\Component\HttpKernel\Event\RequestEvent;
use Symfony\Component\HttpKernel\KernelEvents;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;

/**
 * S190 — désactiver un compte coupe AUSSI les sessions déjà ouvertes.
 *
 * 🔴 **Avant, « inactif » n'agissait qu'au prochain login.** `ActiveAccountChecker`
 * refuse la connexion, mais une session ouverte n'y repasse jamais : Symfony
 * relit le compte à chaque requête et ne le compare qu'au mot de passe, à
 * l'identifiant et aux rôles. Quelqu'un de désactivé pendant qu'il était
 * connecté gardait donc l'accès jusqu'à expiration de sa session — des jours.
 *
 * ✅ Le compte est relu EN BASE à chaque requête par le pare-feu ; il suffit de
 * lire son statut après lui. On ne touche pas à la comparaison de Symfony (elle
 * porte les rôles datés de S159) : on ajoute une règle, on n'en remplace aucune.
 *
 * Priorité 0 : après le pare-feu (8), avant le contrôleur.
 */
#[AsEventListener(event: KernelEvents::REQUEST, priority: 0)]
final class DeactivatedSessionListener
{
    public function __construct(
        private readonly Security $security,
        private readonly UrlGeneratorInterface $urls,
    ) {
    }

    public function __invoke(RequestEvent $event): void
    {
        if (!$event->isMainRequest()) {
            return;
        }
        $user = $this->security->getUser();
        if (!$user instanceof Utilisateur || $user->getStatut() === 'actif') {
            return;
        }

        // `false` : pas de jeton CSRF à valider — c'est le serveur qui coupe.
        $this->security->logout(false);

        $request = $event->getRequest();
        if (str_starts_with($request->getPathInfo(), '/api/')) {
            $event->setResponse(new JsonResponse(['status' => 'account_inactive'], 401));

            return;
        }
        if ($request->hasSession()) {
            // Le même message que la connexion refusée : rien de plus précis.
            $request->getSession()->getFlashBag()->add('error', 'security.account_unavailable');
        }
        $event->setResponse(new RedirectResponse($this->urls->generate('app_login')));
    }
}
