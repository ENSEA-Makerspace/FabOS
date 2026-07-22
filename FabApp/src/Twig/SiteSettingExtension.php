<?php

namespace App\Twig;

use App\Service\SiteSettingService;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * Exposes the admin-configurable alert banner (on/off + text) to templates,
 * so _alert_bar.html.twig can render it without every caller passing context.
 */
final class SiteSettingExtension extends AbstractExtension
{
    public function __construct(private readonly SiteSettingService $siteSettings)
    {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('alert_banner_enabled', $this->siteSettings->isAlertBannerEnabled(...)),
            new TwigFunction('alert_banner_text', $this->siteSettings->getAlertBannerText(...)),
            new TwigFunction('lab_pages_nav_label', $this->siteSettings->getLabPagesNavLabel(...)),
            new TwigFunction('ical_feed_token', $this->siteSettings->getIcalFeedToken(...)),
        ];
    }
}
