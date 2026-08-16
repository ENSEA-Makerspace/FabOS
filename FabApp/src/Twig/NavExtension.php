<?php

namespace App\Twig;

use App\Nav\NavBuilder;
use Symfony\Component\HttpFoundation\RequestStack;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * Exposes the assembled navigation so the header and footer render a list rather
 * than carrying one hand-written condition per feature.
 */
final class NavExtension extends AbstractExtension
{
    public function __construct(
        private readonly NavBuilder $nav,
        private readonly RequestStack $requests,
        private readonly UrlGeneratorInterface $urls,
    ) {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('nav_header', $this->nav->header(...)),
            new TwigFunction('nav_footer', $this->nav->footer(...)),
            // The admin sidebar, grouped by feature — one builder, not a second
            // navigation model living in a template (S82).
            new TwigFunction('nav_admin', $this->nav->admin(...)),
            new TwigFunction('nav_admin_section', $this->nav->adminCurrentSection(...)),
            // The heading of an admin list is the name of the menu entry that
            // leads to it — see `NavBuilder::adminCurrentTitle()` for why the
            // page no longer carries a second, separately-translated copy.
            new TwigFunction('nav_admin_title', $this->nav->adminCurrentTitle(...)),
            // For the error pages: somewhere to go that is guaranteed to answer.
            new TwigFunction('nav_safe_destinations', $this->nav->safeDestinations(...)),
            new TwigFunction('nav_is_current', $this->isCurrent(...)),
        ];
    }

    /**
     * Is this nav entry the section the reader is currently in?
     *
     * The menu had **no active state at all** before this — grepped, the only
     * `is-active` rule in `style.css` was on the language switcher — so it never
     * told anyone where they were, on any page. That is most of what "the menu
     * feels inconsistent" turns out to mean.
     *
     * ⚠️ **Matched by path prefix, not by route name**, and that is the whole
     * design. Route equality would light "Machines" on `/machines` and go dark
     * again on `/machines/12` — the detail page, where knowing which section you
     * are in matters *more*, not less. There is no route-to-section map to keep
     * in sync either; the URL already encodes the hierarchy, and the day it
     * stops doing so the breadcrumbs are wrong too.
     *
     * ⚠️ The prefix test requires a following `/` so `/machines-hs` cannot light
     * up `/machines`, and the root is exact-match only — every path on the site
     * starts with `/`, so a prefix test would leave "Accueil" permanently lit.
     *
     * @param array<string, mixed> $params
     */
    public function isCurrent(string $route, array $params = []): bool
    {
        $request = $this->requests->getCurrentRequest();
        if ($request === null) {
            return false;
        }

        try {
            $target = $this->urls->generate($route, $params);
        } catch (\Throwable) {
            // An entry pointing at a route that no longer exists is a nav bug,
            // not a reason to 500 the header on every page of the site.
            return false;
        }

        // Reconstructed the same way `path()` builds its output, so the two are
        // comparable when the app is served from a sub-directory.
        $here = $request->getBaseUrl() . $request->getPathInfo();
        $target = rtrim($target, '/');

        if ($target === '' || $target === $request->getBaseUrl()) {
            return $here === $target || $here === $target . '/';
        }

        return $here === $target || str_starts_with($here, $target . '/');
    }
}
