<?php

declare(strict_types=1);

namespace App\Security;

use Symfony\Component\EventDispatcher\Attribute\AsEventListener;
use Symfony\Component\HttpFoundation\Cookie;
use Symfony\Component\Security\Http\Event\LogoutEvent;

/**
 * « Déconnexion » ferme la session FabOS, pas celle du fournisseur (Keycloak,
 * CAS de l'école…) : sans ceci, le bouton du fournisseur rouvrait AUSSITÔT le
 * compte précédent — sur un poste partagé du labo, celui d'un autre membre.
 *
 * On pose un petit cookie ; le prochain départ vers un fournisseur le consomme et
 * demande `prompt=login` (le fournisseur redemande le mot de passe). Choix de
 * l'opérateur (2026-09-27) : plutôt que de fermer aussi la session chez le
 * fournisseur, ce qui déconnecterait des autres applications de l'école.
 *
 * ⚠️ Un cookie et pas la session : la session est détruite par la déconnexion.
 * ⚠️ Priorité -64 : après que Symfony a posé la réponse de redirection.
 */
final class ReauthAfterLogoutListener
{
    public const COOKIE = 'fabos_reauth';

    #[AsEventListener(event: LogoutEvent::class, priority: -64)]
    public function onLogout(LogoutEvent $event): void
    {
        $event->getResponse()?->headers->setCookie(
            Cookie::create(self::COOKIE, '1', time() + 86400, '/', null, null, true, false, Cookie::SAMESITE_LAX),
        );
    }
}
