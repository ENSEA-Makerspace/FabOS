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
use Symfony\Component\Security\Http\Event\LoginSuccessEvent;

/**
 * S191b — la barrière du second facteur.
 *
 * Un mot de passe juste, sur un compte qui a activé la double authentification,
 * ouvre une session **EN ATTENTE** : tant que le code n'est pas donné, TOUTE
 * requête est renvoyée vers `/connexion/verification` (l'API répond 401).
 * Seules la page du code et la déconnexion passent.
 *
 * 🔴 **Pourquoi une barrière et pas une étape du formulaire** : Symfony ouvre la
 * session au mot de passe. Plutôt que de réécrire l'authentification (et de
 * rater un de ses chemins), on laisse faire et on ferme TOUT le reste, au même
 * endroit que les coupures de S190 et S191a — après le pare-feu, avant tout
 * contrôleur. Un chemin oublié ici est un chemin qui mène à la page du code.
 *
 * ⚠️ Vaut aussi pour une connexion OIDC : le second facteur est celui du compte.
 * ⚠️ Les rendus console (`app:render`) passent : ce ne sont pas des connexions.
 */
final class MfaGateListener
{
    public const PENDING = 'mfa.pending';
    public const FAILURES = 'mfa.failures';
    private const ALLOWED_ROUTES = ['app_mfa_challenge', 'app_logout'];

    public function __construct(
        private readonly MfaService $mfa,
        private readonly Security $security,
        private readonly UrlGeneratorInterface $urls,
        private readonly ConsoleRenderAuthenticator $console,
    ) {
    }

    #[AsEventListener(event: LoginSuccessEvent::class)]
    public function onLogin(LoginSuccessEvent $event): void
    {
        $user = $event->getUser();
        $request = $event->getRequest();
        if ($this->console->isArmed() || !$user instanceof Utilisateur || !$request->hasSession()) {
            return;
        }
        if ($this->mfa->status($user) === MfaService::ENABLED) {
            $request->getSession()->set(self::PENDING, $user->getId());
            $request->getSession()->set(self::FAILURES, 0);
        }
    }

    /** Priorité -1 : après le pare-feu (8) et la coupure des comptes désactivés (0). */
    #[AsEventListener(event: KernelEvents::REQUEST, priority: -1)]
    public function onRequest(RequestEvent $event): void
    {
        $request = $event->getRequest();
        if (!$event->isMainRequest() || !$request->hasSession() || $this->console->isArmed()) {
            return;
        }
        $session = $request->getSession();
        $pending = $session->get(self::PENDING);
        if ($pending === null) {
            return;
        }
        $user = $this->security->getUser();
        if (!$user instanceof Utilisateur || $user->getId() !== $pending) {
            // Plus personne de connecté (ou quelqu'un d'autre) : la marque n'a plus d'objet.
            $session->remove(self::PENDING);

            return;
        }
        if (in_array($request->attributes->get('_route'), self::ALLOWED_ROUTES, true)) {
            return;
        }
        if (str_starts_with($request->getPathInfo(), '/api/')) {
            $event->setResponse(new JsonResponse(['status' => 'mfa_required'], 401));

            return;
        }
        $event->setResponse(new RedirectResponse($this->urls->generate('app_mfa_challenge')));
    }
}
