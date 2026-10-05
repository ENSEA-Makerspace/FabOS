<?php

declare(strict_types=1);

namespace App\Twig;

use App\Feature\SiteFeatureService;
use App\Service\MachineReports;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * S205 — `machine_reports_open()` : « Signaler une panne » est-il ouvert ? Oui si la
 * fonction est allumée, les machines aussi, ET la migration passée — la MÊME condition que
 * le contrôleur public (sinon un lien mènerait à un 404). Sert aux liens vers `/signaler`
 * (fiche publique d'une machine, panneau de retour de l'en-tête).
 */
final class MachineReportExtension extends AbstractExtension
{
    public function __construct(
        private readonly SiteFeatureService $features,
        private readonly MachineReports $reports,
    ) {
    }

    public function getFunctions(): array
    {
        return [new TwigFunction('machine_reports_open', fn (): bool => $this->features->allowsSurface('machine_reports') && $this->features->allowsSurface('machines') && $this->reports->isReady())];
    }
}
