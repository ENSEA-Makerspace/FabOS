<?php

namespace App\Event;

use App\Entity\EventRegistration;
use App\Service\SiteSettingService;
use Endroid\QrCode\Builder\Builder;
use Endroid\QrCode\ErrorCorrectionLevel;
use Endroid\QrCode\Writer\SvgWriter;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\UriSigner;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Component\Routing\RouterInterface;

/**
 * Tickets: the attendee's signed ticket page, and the QR a staff member scans
 * at the door.
 *
 * **The two URLs are deliberately different, and that difference is the whole
 * security model.** The *ticket* link is what the attendee holds — it proves
 * which registration they are, and nothing more. The *scan* link encoded in the
 * QR points at a staff-only route: signing it stops the code being forged, but
 * it is the firewall, not the signature, that stops the attendee following
 * their own QR to check themselves in. A single URL doing both jobs would hand
 * every ticket-holder a self-service check-in button.
 *
 * QR payloads are absolute URLs built from the admin-configured public base
 * URL, for the same reason mail links are: a QR is photographed off a screen or
 * a printout and followed on a different device, where a relative path is
 * meaningless and the router's DEFAULT_URI is localhost.
 */
final class TicketLinker
{
    public const TICKET_ROUTE = 'app_event_ticket';
    public const SCAN_ROUTE = 'app_staff_event_scan';

    public function __construct(
        private readonly RouterInterface $router,
        private readonly UriSigner $signer,
        private readonly SiteSettingService $settings,
    ) {
    }

    /** The attendee's own ticket page. Null when no public URL is configured. */
    public function ticketUrl(EventRegistration $registration): ?string
    {
        return $this->signedUrl(self::TICKET_ROUTE, $registration);
    }

    /** What the QR encodes: the staff check-in endpoint for this registration. */
    public function scanUrl(EventRegistration $registration): ?string
    {
        return $this->signedUrl(self::SCAN_ROUTE, $registration);
    }

    public function isValid(Request $request): bool
    {
        $baseUrl = $this->settings->getPublicBaseUrl();

        return $baseUrl !== '' && $this->signer->check($baseUrl . $request->getRequestUri());
    }

    /**
     * The QR as an inline SVG data URI.
     *
     * SVG rather than PNG so it stays sharp on a phone screen and on paper at
     * any print size, and a data URI so the ticket page and a printed copy need
     * no second request — a kiosk or a printout may have no network at all.
     *
     * High error correction because these get scanned off creased paper and
     * smudged phone screens.
     */
    public function qrSvgDataUri(EventRegistration $registration): ?string
    {
        $url = $this->scanUrl($registration);
        if ($url === null) {
            return null;
        }

        try {
            $result = (new Builder(
                writer: new SvgWriter(),
                data: $url,
                errorCorrectionLevel: ErrorCorrectionLevel::High,
                size: 320,
                margin: 8,
            ))->build();

            return $result->getDataUri();
        } catch (\Throwable) {
            // A ticket without its QR is still a usable ticket — it carries the
            // code and the name — so this must not take the page down.
            return null;
        }
    }

    /**
     * Short human-readable code, shown under the QR.
     *
     * Derived from the signature rather than stored: it needs no column, it
     * changes if the link is ever re-signed, and it gives staff something to
     * type when a camera won't cooperate — which at a real door it eventually
     * won't.
     */
    public function shortCode(EventRegistration $registration): string
    {
        $url = $this->scanUrl($registration) ?? (string) $registration->getId();
        $digest = strtoupper(substr(hash('sha256', $url), 0, 8));

        return substr($digest, 0, 4) . '-' . substr($digest, 4, 4);
    }

    private function signedUrl(string $route, EventRegistration $registration): ?string
    {
        $baseUrl = $this->settings->getPublicBaseUrl();
        $id = $registration->getId();

        if ($id === null || $baseUrl === '') {
            return null;
        }

        try {
            $path = $this->router->generate($route, ['registration' => $id], UrlGeneratorInterface::ABSOLUTE_PATH);
        } catch (\Throwable) {
            return null;
        }

        return $this->signer->sign($baseUrl . $path);
    }
}
