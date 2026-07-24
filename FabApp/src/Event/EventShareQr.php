<?php

namespace App\Event;

use App\Entity\Event;
use App\Service\SiteSettingService;
use Endroid\QrCode\Builder\Builder;
use Endroid\QrCode\ErrorCorrectionLevel;
use Endroid\QrCode\Writer\PngWriter;
use Endroid\QrCode\Writer\SvgWriter;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Component\Routing\RouterInterface;

/**
 * The QR you print on a poster: it points at the event's **public page**, so
 * anyone who scans it lands on the description and the registration form.
 *
 * Not to be confused with TicketLinker, which is the opposite direction. That
 * one is per *registration*, signed, and scanned by staff to admit a specific
 * person. This one is per *event*, unsigned, and scanned by the public to sign
 * up in the first place. A signature here would be actively wrong — the target
 * is a public page, and a signed URL on a printed poster only creates a link
 * that breaks if the secret ever rotates.
 */
final class EventShareQr
{
    public function __construct(
        private readonly RouterInterface $router,
        private readonly SiteSettingService $settings,
    ) {
    }

    /** The plain public URL the QR encodes — also worth printing as text. */
    public function publicUrl(Event $event): ?string
    {
        $baseUrl = $this->settings->getPublicBaseUrl();
        $id = $event->getId();

        if ($id === null || $baseUrl === '') {
            return null;
        }

        try {
            return $baseUrl . $this->router->generate('app_event_detail', ['id' => $id], UrlGeneratorInterface::ABSOLUTE_PATH);
        } catch (\Throwable) {
            return null;
        }
    }

    /** Inline SVG for the admin preview. */
    public function svgDataUri(Event $event): ?string
    {
        $url = $this->publicUrl($event);

        return $url === null ? null : $this->build($url, new SvgWriter(), 300)?->getDataUri();
    }

    /**
     * Print-resolution PNG bytes for download.
     *
     * 1200px because this ends up on an A3 poster at arm's length, and a QR
     * upscaled from a screen-sized image gets soft edges that phones struggle
     * with. High error correction for the same reason the ticket uses it:
     * posters get creased, rained on and photographed at an angle.
     */
    public function pngBytes(Event $event): ?string
    {
        $url = $this->publicUrl($event);

        return $url === null ? null : $this->build($url, new PngWriter(), 1200)?->getString();
    }

    private function build(string $url, object $writer, int $size): ?\Endroid\QrCode\Writer\Result\ResultInterface
    {
        try {
            return (new Builder(
                writer: $writer,
                data: $url,
                errorCorrectionLevel: ErrorCorrectionLevel::High,
                size: $size,
                margin: 16,
            ))->build();
        } catch (\Throwable) {
            return null;
        }
    }
}
