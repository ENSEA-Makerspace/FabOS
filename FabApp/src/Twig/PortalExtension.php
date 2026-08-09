<?php

namespace App\Twig;

use App\Service\SiteSettingService;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * The branding the current portal asks for, if any.
 *
 * These read through `SiteSettingService::get()`, which already resolves
 * most-specific-first, so a portal's row wins for a request served on its
 * hostname and everything else falls back to the site-wide value. Null means
 * "nothing overridden" and every caller renders exactly what it rendered before.
 */
final class PortalExtension extends AbstractExtension
{
    public function __construct(
        private readonly SiteSettingService $settings,
    ) {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('portal_name', $this->name(...)),
            new TwigFunction('portal_logo_path', $this->logoPath(...)),
            new TwigFunction('portal_primary_color', $this->primaryColor(...)),
        ];
    }

    public function name(): string
    {
        return $this->settings->getOrgName();
    }

    /**
     * A filename inside `public/images/`, or null.
     *
     * Path separators are refused rather than sanitised. The value reaches
     * `asset('images/' ~ …)`, so `../../.env` would otherwise resolve to a URL
     * pointing outside the image directory — and the admin who set it would have
     * no idea they had done that.
     */
    public function logoPath(): ?string
    {
        $value = trim((string) $this->settings->get('site_logo_path'));

        return $value !== '' && preg_match('/^[A-Za-z0-9._-]+\.(png|jpe?g|webp|svg)$/i', $value) === 1 ? $value : null;
    }

    /**
     * A hex colour, or null.
     *
     * ⚠️ **Validated on read, not only on save.** This value is interpolated into
     * a `<style>` block, where Twig's HTML escaping does nothing useful — inside
     * CSS, `}` ends the rule and anything after it is more CSS. Checking the
     * shape here means a row written by an older version, a direct SQL edit or a
     * future form that forgets to validate still cannot inject stylesheet.
     */
    public function primaryColor(): ?string
    {
        $value = trim((string) $this->settings->get('site_primary_color'));

        return preg_match('/^#(?:[0-9a-f]{3}|[0-9a-f]{6})$/i', $value) === 1 ? $value : null;
    }
}
