<?php

namespace App\Service;

use App\Portal\PortalContext;
use Doctrine\DBAL\Connection;

/**
 * Small generic key/value store for site-wide settings (currently just the default
 * locale). Mirrors SiteFeatureService's fail-safe pattern: a missing table/row falls back
 * to a sane default instead of breaking the site.
 *
 * Rows are scoped by portal: portalId 0 holds the site-wide value, a portal's own
 * row overrides it for that portal only (see PortalContext). Reads take the most
 * specific row available; writes land in the scope the request is being served at,
 * which is the global one unless a hostname resolves to a portal.
 */
final class SiteSettingService
{
    private const DEFAULT_LOCALE_KEY = 'default_locale';
    private const FALLBACK_LOCALE = 'fr';
    private const ALERT_BANNER_ENABLED_KEY = 'alert_banner_enabled';
    private const ALERT_BANNER_TEXT_KEY = 'alert_banner_text';
    private const LAB_PAGES_NAV_LABEL_KEY = 'lab_pages_nav_label';
    private const FALLBACK_LAB_PAGES_NAV_LABEL = 'Our Lab';
    private const LAB_RULES_HTML_KEY = 'lab_rules_html';
    private const LAB_RULES_PDF_URL_KEY = 'lab_rules_pdf_url';
    private const ICAL_FEED_TOKEN_KEY = 'ical_feed_token';
    private const PUBLIC_BASE_URL_KEY = 'public_base_url';
    private const LAB_ADDRESS_KEY = 'lab_address';
    private const ORG_NAME_KEY = 'org_name';
    private const VENUE_LABEL_KEY = 'venue_label';

    /**
     * The defaults are **this install's current wording**, not neutral words.
     *
     * FabOS is not a fablab-only product, but this deployment is one, and the
     * point of S31 is to make the vocabulary editable — not to rename anybody's
     * site out from under them. So an install that never opens the settings
     * screen renders exactly the strings it rendered before.
     */
    private const FALLBACK_ORG_NAME = 'ENSEA';
    private const FALLBACK_VENUE_LABEL = 'FabLab';

    public function __construct(
        private readonly Connection $db,
        private readonly PortalContext $portals,
    ) {
    }

    public function getDefaultLocale(): string
    {
        $value = $this->get(self::DEFAULT_LOCALE_KEY);

        return $value !== null && $value !== '' ? $value : self::FALLBACK_LOCALE;
    }

    public function setDefaultLocale(string $locale): void
    {
        $this->set(self::DEFAULT_LOCALE_KEY, $locale);
    }

    public function isAlertBannerEnabled(): bool
    {
        return $this->get(self::ALERT_BANNER_ENABLED_KEY) === '1';
    }

    public function getAlertBannerText(): string
    {
        return $this->get(self::ALERT_BANNER_TEXT_KEY) ?? '';
    }

    public function setAlertBanner(bool $enabled, string $text): void
    {
        $this->set(self::ALERT_BANNER_ENABLED_KEY, $enabled ? '1' : '0');
        $this->set(self::ALERT_BANNER_TEXT_KEY, $text);
    }

    public function getLabPagesNavLabel(): string
    {
        $value = $this->get(self::LAB_PAGES_NAV_LABEL_KEY);

        return $value !== null && trim($value) !== '' ? $value : self::FALLBACK_LAB_PAGES_NAV_LABEL;
    }

    public function setLabPagesNavLabel(string $label): void
    {
        $this->set(self::LAB_PAGES_NAV_LABEL_KEY, trim($label));
    }

    /**
     * Admin-authored lab rules, stored as HTML (trusted, admin-only input). Empty
     * string when unset — the public page renders a placeholder in that case.
     */
    public function getLabRulesHtml(): string
    {
        return $this->get(self::LAB_RULES_HTML_KEY) ?? '';
    }

    /**
     * Optional URL to a downloadable PDF of the lab rules. Empty when unset —
     * the public page hides the download button in that case.
     */
    public function getLabRulesPdfUrl(): string
    {
        return $this->get(self::LAB_RULES_PDF_URL_KEY) ?? '';
    }

    public function setLabRules(string $html, string $pdfUrl): void
    {
        $this->set(self::LAB_RULES_HTML_KEY, $html);
        $this->set(self::LAB_RULES_PDF_URL_KEY, trim($pdfUrl));
    }

    /**
     * Shared secret gating the read-only iCal resource feeds. Lazily generated
     * and persisted on first use so feeds work out of the box; rotate it with
     * regenerateIcalFeedToken(). Returns '' only if the store is unavailable.
     */
    public function getIcalFeedToken(): string
    {
        $value = $this->get(self::ICAL_FEED_TOKEN_KEY);

        if ($value !== null && $value !== '') {
            return $value;
        }

        return $this->regenerateIcalFeedToken();
    }

    public function regenerateIcalFeedToken(): string
    {
        $token = bin2hex(random_bytes(20));

        try {
            $this->set(self::ICAL_FEED_TOKEN_KEY, $token);
        } catch (\Throwable) {
            return '';
        }

        return $token;
    }

    /**
     * The address this site is reachable at from outside, without a trailing
     * slash — e.g. https://fabos.example.org.
     *
     * Mail rendered by the worker has no request to infer a host from, and
     * Symfony's router falls back to DEFAULT_URI, which ships as
     * http://localhost. A link built from that is worse than no link. So the
     * public URL is a setting an admin owns, like the sender account: a lab can
     * fix its own links without a deploy, and mail features that need an
     * absolute URL can ask whether one is actually usable first.
     */
    public function getPublicBaseUrl(): string
    {
        $url = trim($this->get(self::PUBLIC_BASE_URL_KEY) ?? '');
        if ($url === '') {
            return '';
        }

        $url = rtrim($url, '/');

        // Anything that isn't an absolute http(s) URL would produce a broken
        // link in somebody's inbox; treat it as unset instead.
        return preg_match('#^https?://[^/\s]+#i', $url) === 1 ? $url : '';
    }

    public function setPublicBaseUrl(string $url): void
    {
        $this->set(self::PUBLIC_BASE_URL_KEY, rtrim(trim($url), '/'));
    }

    /**
     * The lab's own postal address, used for every on-site event.
     *
     * Stored once here rather than per event: a lab that moves premises edits
     * one field instead of every event it has ever created, and events created
     * before the move keep pointing at wherever the lab actually is.
     */
    public function getLabAddress(): string
    {
        return trim($this->get(self::LAB_ADDRESS_KEY) ?? '');
    }

    public function setLabAddress(string $address): void
    {
        $this->set(self::LAB_ADDRESS_KEY, trim($address));
    }

    /** The organisation running this install — a school, an association, a company. */
    public function getOrgName(): string
    {
        $value = trim($this->get(self::ORG_NAME_KEY) ?? '');

        return $value !== '' ? $value : self::FALLBACK_ORG_NAME;
    }

    /**
     * What this place is called: "FabLab", "atelier", "makerspace", "studio".
     *
     * Distinct from the organisation on purpose — "the ENSEA FabLab" is two
     * different nouns, and a deployment can need to change either one without
     * the other. Every catalog string that used to hardcode one of them now
     * carries `%org%` or `%venue%`.
     */
    public function getVenueLabel(): string
    {
        $value = trim($this->get(self::VENUE_LABEL_KEY) ?? '');

        return $value !== '' ? $value : self::FALLBACK_VENUE_LABEL;
    }

    public function setVocabulary(string $orgName, string $venueLabel): void
    {
        $this->set(self::ORG_NAME_KEY, trim($orgName));
        $this->set(self::VENUE_LABEL_KEY, trim($venueLabel));
    }

    /**
     * Most specific value for the key: the current portal's row if it has one,
     * otherwise the global row. Null when unset — or when the store is missing
     * entirely, so a setting never takes the site down.
     *
     * Public so feature-owned settings groups (App\Mail\MailSettings) can share the
     * portal-scoped store without re-implementing the fallback; the typed accessors
     * above stay the right way in for anything site-wide.
     */
    public function get(string $key): ?string
    {
        try {
            $value = $this->db->fetchOne(
                'SELECT settingValue FROM SITE_SETTING WHERE settingKey = :k AND portalId IN (:g, :p) ORDER BY portalId DESC LIMIT 1',
                ['k' => $key, 'g' => PortalContext::GLOBAL_SCOPE, 'p' => $this->portals->scopeId()],
            );
        } catch (\Throwable) {
            return null;
        }

        return is_string($value) ? $value : null;
    }

    /**
     * One portal's own row for a key, with **no fallback to global** — null means
     * "this portal overrides nothing here", which the portal editor has to be able
     * to tell apart from "overrides it with the same value the site has".
     */
    public function getForScope(int $portalId, string $key): ?string
    {
        try {
            $value = $this->db->fetchOne(
                'SELECT settingValue FROM SITE_SETTING WHERE settingKey = :k AND portalId = :p',
                ['k' => $key, 'p' => $portalId],
            );
        } catch (\Throwable) {
            return null;
        }

        return is_string($value) ? $value : null;
    }

    /** Writes one portal's override; **null removes the row**, so the key inherits global again. */
    public function setForScope(int $portalId, string $key, ?string $value): void
    {
        if ($value === null) {
            $this->db->executeStatement(
                'DELETE FROM SITE_SETTING WHERE settingKey = :k AND portalId = :p',
                ['k' => $key, 'p' => $portalId],
            );

            return;
        }

        $this->db->executeStatement(
            'INSERT INTO SITE_SETTING (settingKey, portalId, settingValue) VALUES (:k, :p, :v) ON DUPLICATE KEY UPDATE settingValue = :v',
            ['k' => $key, 'p' => $portalId, 'v' => $value],
        );
    }

    /** Writes at the scope the request is being served at — global unless a portal is resolved. */
    public function set(string $key, string $value): void
    {
        $this->db->executeStatement(
            'INSERT INTO SITE_SETTING (settingKey, portalId, settingValue) VALUES (:k, :p, :v) ON DUPLICATE KEY UPDATE settingValue = :v',
            ['k' => $key, 'p' => $this->portals->scopeId(), 'v' => $value],
        );
    }
}
