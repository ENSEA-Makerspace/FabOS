<?php

namespace App\EventSubscriber;

use App\Entity\Utilisateur;
use App\Service\SiteSettingService;
use Symfony\Bundle\SecurityBundle\Security;
use Symfony\Component\EventDispatcher\EventSubscriberInterface;
use Symfony\Component\HttpKernel\Event\RequestEvent;
use Symfony\Component\HttpKernel\KernelEvents;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * Applies the effective language to both the request and the translator, resolved from:
 *   1. an explicit choice stored in the session (set via /locale/{locale}, currently
 *      only reachable directly, not linked from the menu per admin's choice),
 *   2. otherwise the logged-in user's stored preference (Utilisateur::getLangue()),
 *   3. otherwise the site-wide default locale, configured in the admin (Site settings)
 *      via SiteSettingService — not hardcoded, so the admin fully controls it.
 *
 * Runs at priority 7 (just after the security firewall at 8, so getUser() is populated).
 * Because Symfony's LocaleAwareListener has already pushed the *request* locale onto the
 * translator earlier in the dispatch (priority 15), we must set the translator locale
 * ourselves here — otherwise it would stay stuck on the default.
 */
final class LocaleSubscriber implements EventSubscriberInterface
{
    /** @param string[] $enabledLocales */
    public function __construct(
        private readonly Security $security,
        private readonly TranslatorInterface $translator,
        private readonly SiteSettingService $siteSettings,
        private readonly array $enabledLocales = ['fr', 'en', 'es', 'de', 'it'],
    ) {
    }

    public function onKernelRequest(RequestEvent $event): void
    {
        if (!$event->isMainRequest()) {
            return;
        }

        $request = $event->getRequest();
        $locale = $this->siteSettings->getDefaultLocale();

        // 1. Explicit choice stored in session (guard on hasPreviousSession so we only
        //    touch the session when the request actually carries a session cookie).
        if ($request->hasPreviousSession()) {
            $sessionLocale = $request->getSession()->get('_locale');
            if (is_string($sessionLocale) && in_array($sessionLocale, $this->enabledLocales, true)) {
                $locale = $sessionLocale;
            } elseif (($u = $this->security->getUser()) instanceof Utilisateur && in_array($u->getLangue(), $this->enabledLocales, true)) {
                $locale = $u->getLangue();
            }
        } elseif (($user = $this->security->getUser()) instanceof Utilisateur && in_array($user->getLangue(), $this->enabledLocales, true)) {
            // 2. Logged-in user's stored preference.
            $locale = $user->getLangue();
        }

        $request->setLocale($locale);
        $this->translator->setLocale($locale);
    }

    public static function getSubscribedEvents(): array
    {
        return [KernelEvents::REQUEST => [['onKernelRequest', 7]]];
    }
}
