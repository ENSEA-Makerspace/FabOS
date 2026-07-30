<?php

namespace App\Twig;

use App\Feature\SiteFeatureService;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * Exposes {{ feature_enabled('badges') }} to templates so nav links / sections
 * can be hidden when an optional module is turned off in the admin.
 */
final class FeatureExtension extends AbstractExtension
{
    public function __construct(private readonly SiteFeatureService $features)
    {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('feature_enabled', $this->features->isEnabled(...)),
            // The calendar is a surface over whatever layers are drawn on it, so
            // its links ask this rather than naming machines and spaces one by
            // one — a third layer should not mean editing every template again.
            new TwigFunction('has_calendar_layer', $this->features->hasCalendarLayer(...)),
        ];
    }
}
