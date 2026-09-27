<?php

declare(strict_types=1);

namespace App\Security;

use App\Entity\Utilisateur;
use App\Identity\PendingExternalLogin;
use Symfony\Bundle\SecurityBundle\Security;
use Symfony\Component\EventDispatcher\Attribute\AsEventListener;
use Symfony\Component\HttpFoundation\RedirectResponse;
use Symfony\Component\HttpKernel\Event\RequestEvent;
use Symfony\Component\HttpKernel\KernelEvents;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;

/**
 * S197 — après « J'ai déjà un compte FabOS » puis la connexion à ce compte,
 * TOUTE page mène à la confirmation de la liaison, jusqu'à ce qu'on réponde.
 *
 * ⚠️ Priorité -2 : APRÈS la barrière du second facteur (-1). Tant que le code
 * n'est pas donné, c'est elle qui répond ; la liaison n'est proposée qu'à une
 * connexion COMPLÈTE.
 * ⚠️ Rien n'est lié ici : la page montre l'identité externe et attend un clic.
 */
final class ExternalLinkGateListener
{
    private const ALLOWED_ROUTES = ['app_external_link_confirm', 'app_logout', 'app_mfa_challenge'];

    public function __construct(
        private readonly PendingExternalLogin $pending,
        private readonly Security $security,
        private readonly UrlGeneratorInterface $urls,
        private readonly ConsoleRenderAuthenticator $console,
    ) {
    }

    #[AsEventListener(event: KernelEvents::REQUEST, priority: -2)]
    public function onRequest(RequestEvent $event): void
    {
        $request = $event->getRequest();
        if (!$event->isMainRequest() || !$request->hasSession() || $this->console->isArmed()
            || str_starts_with($request->getPathInfo(), '/api/')
            || \in_array($request->attributes->get('_route'), self::ALLOWED_ROUTES, true)) {
            return;
        }
        if (!$request->getSession()->has(PendingExternalLogin::KEY)) {
            return;
        }
        $state = $this->pending->get($request->getSession());
        if ($state === null || !$state['wantsLink'] || !$this->security->getUser() instanceof Utilisateur) {
            return;
        }
        $event->setResponse(new RedirectResponse($this->urls->generate('app_external_link_confirm')));
    }
}
