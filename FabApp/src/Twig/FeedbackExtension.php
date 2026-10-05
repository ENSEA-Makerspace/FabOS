<?php

declare(strict_types=1);

namespace App\Twig;

use App\Feature\SiteFeatureService;
use App\Service\Feedback;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * S210 — `feedback_open()` : l'entrée « Signaler un problème » de l'en-tête
 * s'affiche-t-elle ? Oui si la fonction est allumée ET la migration passée.
 * (Que le visiteur soit connecté se teste dans le gabarit : `app.user`.)
 */
final class FeedbackExtension extends AbstractExtension
{
    public function __construct(
        private readonly SiteFeatureService $features,
        private readonly Feedback $feedback,
    ) {
    }

    public function getFunctions(): array
    {
        return [new TwigFunction('feedback_open', fn (): bool => $this->features->allowsSurface('feedback') && $this->feedback->isReady())];
    }
}
