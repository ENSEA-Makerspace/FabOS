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
use Symfony\Component\Security\Http\Event\LogoutEvent;

/**
 * S191a — suivre les sessions : les ouvrir, les voir vivre, couper celles qu'on a
 * fermées ailleurs.
 *
 * Même mécanique que `DeactivatedSessionListener` (S190) : le compte est relu
 * à chaque requête par le pare-feu, on lit l'état de SA session juste après. Une
 * session fermée depuis un autre appareil est coupée à sa requête suivante — la
 * page renvoie à la connexion, l'API répond 401.
 *
 * ⚠️ Une session ouverte AVANT ce suivi (ou avant la migration) n'a pas de
 * ligne : on la crée au passage, pour qu'elle apparaisse dans la liste.
 * ⚠️ Les rendus console (`app:render`) ne sont pas des connexions : ignorés.
 */
final class SessionTrackingListener
{
    public function __construct(
        private readonly SessionRegistry $registry,
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
        $this->registry->open($user, $request, $request->getSession());
    }

    #[AsEventListener(event: KernelEvents::REQUEST, priority: -2)]
    public function onRequest(RequestEvent $event): void
    {
        $request = $event->getRequest();
        if (!$event->isMainRequest() || !$request->hasSession() || $this->console->isArmed() || !$this->registry->isReady()) {
            return;
        }
        $user = $this->security->getUser();
        if (!$user instanceof Utilisateur) {
            return;
        }
        $session = $request->getSession();

        match ($this->registry->state($session)) {
            'unknown' => $this->registry->open($user, $request, $session),
            'alive' => $this->registry->touch($session),
            'revoked' => $this->cut($event),
        };
    }

    /**
     * ⚠️ Priorité 128 : `SessionLogoutListener` (0) VIDE la session ; passé après
     * lui, on ne trouvait plus la clé et la ligne restait « ouverte » — la sonde
     * S191a l'a attrapé.
     */
    #[AsEventListener(event: LogoutEvent::class, priority: 128)]
    public function onLogout(LogoutEvent $event): void
    {
        $request = $event->getRequest();
        if ($request->hasSession()) {
            $this->registry->close($request->getSession());
        }
    }

    private function cut(RequestEvent $event): void
    {
        $this->security->logout(false);
        $request = $event->getRequest();
        if (str_starts_with($request->getPathInfo(), '/api/')) {
            $event->setResponse(new JsonResponse(['status' => 'session_closed'], 401));

            return;
        }
        $request->getSession()->getFlashBag()->add('error', 'security.session_closed');
        $event->setResponse(new RedirectResponse($this->urls->generate('app_login')));
    }
}
