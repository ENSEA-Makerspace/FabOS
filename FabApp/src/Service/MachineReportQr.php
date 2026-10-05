<?php

declare(strict_types=1);

namespace App\Service;

use Endroid\QrCode\Builder\Builder;
use Endroid\QrCode\ErrorCorrectionLevel;
use Endroid\QrCode\Writer\SvgWriter;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Component\Routing\RouterInterface;

/**
 * S205 — le QR collé SUR la machine : il mène à `/signaler/{id}`, la page
 * publique de signalement, machine déjà choisie.
 *
 * ⚠️ Même construction que `EventShareQr` (adresse publique des réglages, niveau
 * de correction élevé : une étiquette collée se froisse, se salit, se photographie
 * de biais). Non signé : la cible est une page publique, et une signature sur une
 * étiquette imprimée ne ferait que casser le jour où le secret change.
 */
final class MachineReportQr
{
    public function __construct(
        private readonly RouterInterface $router,
        private readonly SiteSettingService $settings,
    ) {
    }

    /** L'adresse que le QR encode — à imprimer aussi en clair, sous lui. */
    public function publicUrl(int $machineId): ?string
    {
        $baseUrl = $this->settings->getPublicBaseUrl();
        if ($baseUrl === '') {
            return null;
        }

        try {
            return $baseUrl . $this->router->generate('app_report_machine', ['id' => $machineId], UrlGeneratorInterface::ABSOLUTE_PATH);
        } catch (\Throwable) {
            return null;
        }
    }

    public function svgDataUri(int $machineId): ?string
    {
        $url = $this->publicUrl($machineId);
        if ($url === null) {
            return null;
        }

        try {
            return (new Builder(
                writer: new SvgWriter(),
                data: $url,
                errorCorrectionLevel: ErrorCorrectionLevel::High,
                size: 400,
                margin: 16,
            ))->build()->getDataUri();
        } catch (\Throwable) {
            return null;
        }
    }
}
