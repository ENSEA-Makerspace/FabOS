<?php

namespace App\Mail;

use App\Service\SiteSettingService;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\UriSigner;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Component\Routing\RouterInterface;

/**
 * Builds and checks the one-click unsubscribe links in the footer of every
 * non-transactional mail.
 *
 * Signed rather than stored: the link carries the user id and category and an
 * HMAC over the whole URL, so there is no token column to provision, rotate or
 * leak, and a recipient cannot edit somebody else's id into their own link.
 * There is deliberately **no expiry** — an unsubscribe link found in a
 * two-year-old mail should still work, because the alternative is a person who
 * wants out being told the link is stale.
 *
 * Links are built against the admin-configured public URL, not the router's
 * request context: this runs in a worker, where the context is DEFAULT_URI and
 * points at localhost. When no public URL is configured, no link is produced —
 * mail goes out without a footer link rather than with a broken one.
 */
final class UnsubscribeLinker
{
    public const ROUTE = 'app_unsubscribe';
    public const EVENT_CANCEL_ROUTE = 'app_event_registration_cancel';

    public function __construct(
        private readonly RouterInterface $router,
        private readonly UriSigner $signer,
        private readonly SiteSettingService $settings,
    ) {
    }

    /** Null when this mail should carry no link: wrong category, or no public URL set. */
    public function urlFor(?int $userId, string $category): ?string
    {
        $baseUrl = $this->settings->getPublicBaseUrl();
        if ($userId === null || $baseUrl === '' || !NotificationCategory::isOptOutable($category)) {
            return null;
        }

        try {
            $path = $this->router->generate(
                self::ROUTE,
                ['user' => $userId, 'category' => $category],
                UrlGeneratorInterface::ABSOLUTE_PATH,
            );
        } catch (\Throwable) {
            return null;
        }

        // Sign the absolute URL, so the signature covers the host the link will
        // actually be followed on.
        return $this->signer->sign($baseUrl . $path);
    }

    /**
     * The signed link a guest uses to cancel an event registration.
     *
     * Same machinery as the unsubscribe link, for the same reason: the person
     * has no account to sign in with, so the signature *is* the authorisation,
     * and it names one registration that cannot be edited into naming another.
     */
    public function eventCancelUrl(\App\Entity\EventRegistration $registration): ?string
    {
        $baseUrl = $this->settings->getPublicBaseUrl();
        $id = $registration->getId();
        if ($id === null || $baseUrl === '') {
            return null;
        }

        try {
            $path = $this->router->generate(
                self::EVENT_CANCEL_ROUTE,
                ['registration' => $id],
                UrlGeneratorInterface::ABSOLUTE_PATH,
            );
        } catch (\Throwable) {
            return null;
        }

        return $this->signer->sign($baseUrl . $path);
    }

    /**
     * Whether an incoming request really came from a link we produced.
     *
     * The signature covers the URL as signed — including the host — so this is
     * checked against the configured public URL rather than whatever Host
     * header arrived.
     */
    public function isValid(Request $request): bool
    {
        $baseUrl = $this->settings->getPublicBaseUrl();
        if ($baseUrl === '') {
            return false;
        }

        return $this->signer->check($baseUrl . $request->getRequestUri());
    }
}
