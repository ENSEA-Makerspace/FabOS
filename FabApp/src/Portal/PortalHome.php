<?php

namespace App\Portal;

use App\Feature\SiteFeatureRegistry;
use App\Feature\SiteFeatureService;
use App\Service\SiteSettingService;

/**
 * Which page a portal opens on.
 *
 * The driving case is a tenant who only runs events: their front door should be
 * the events page, not a block homepage advertising equipment they do not have.
 *
 * **A route name, never a path.** The stored value is matched against a list this
 * class builds from the enabled features; anything else is ignored. A free-text
 * path here would be an open redirect and, the first time somebody typed one
 * wrong, a 500 on the site's front page.
 *
 * **The list is rebuilt per request, and that is the important part.** A portal
 * pointing at events whose events feature is later switched off would otherwise
 * 302 its own front page onto a 404 — the gate does its job and the site becomes
 * unreachable at the root. Validating against what is enabled *now* means the
 * worst case is the ordinary homepage coming back, which is a fallback rather
 * than an outage. Same reasoning as validating the accent colour on read.
 */
final class PortalHome
{
    public const SETTING = 'portal_home_route';

    public function __construct(
        private readonly SiteSettingService $settings,
        private readonly SiteFeatureService $features,
        private readonly SiteFeatureRegistry $registry,
    ) {
    }

    /**
     * The pages this scope could open on: every enabled feature with a public
     * landing page, plus the calendar when anything is drawn on it.
     *
     * `app_home` is deliberately absent — choosing it would redirect the
     * homepage to itself forever.
     *
     * @return array<string, string> route name => label
     */
    public function options(): array
    {
        $options = [];

        // The calendar belongs to no single feature; it is the shared grid the
        // resource layers project onto, so it follows the same rule the nav uses.
        if ($this->features->hasCalendarLayer()) {
            $options['app_calendar'] = 'Calendrier';
        }

        foreach ($this->registry->all() as $feature) {
            if ($feature->landingRoute !== null && $this->features->isEnabled($feature->key)) {
                $options[$feature->landingRoute] = $feature->label;
            }
        }

        return $options;
    }

    /**
     * The route to send the homepage to, or null to render the homepage itself.
     *
     * Null covers every uninteresting case — nothing configured, a value from an
     * older release, a feature since switched off — because they all have the
     * same right answer: show the ordinary homepage.
     */
    public function redirectRoute(): ?string
    {
        $configured = trim((string) $this->settings->get(self::SETTING));

        return $configured !== '' && isset($this->options()[$configured]) ? $configured : null;
    }
}
