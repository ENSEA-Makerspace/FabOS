<?php

namespace App\Service;

use App\Portal\PortalContext;
use Doctrine\DBAL\Connection;

/**
 * Small generic key/value store for site-wide settings (currently just the default
 * locale). Mirrors ModuleService's fail-safe pattern: a missing table/row falls back
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
     * Most specific value for the key: the current portal's row if it has one,
     * otherwise the global row. Null when unset — or when the store is missing
     * entirely, so a setting never takes the site down.
     */
    private function get(string $key): ?string
    {
        try {
            $value = $this->db->fetchOne(
                'SELECT settingValue FROM SITE_SETTING WHERE settingKey = :k AND portalId IN (:g, :p) ORDER BY portalId DESC LIMIT 1',
                ['k' => $key, 'g' => PortalContext::GLOBAL_SCOPE, 'p' => $this->portals->scopeId()],
            );
        } catch (\Throwable) {
            // Unscoped retry: keeps the configured values readable if this code lands
            // before migration Version20260726100000 has run. Drop once it's everywhere.
            try {
                $value = $this->db->fetchOne('SELECT settingValue FROM SITE_SETTING WHERE settingKey = :k', ['k' => $key]);
            } catch (\Throwable) {
                return null;
            }
        }

        return is_string($value) ? $value : null;
    }

    private function set(string $key, string $value): void
    {
        $this->db->executeStatement(
            'INSERT INTO SITE_SETTING (settingKey, portalId, settingValue) VALUES (:k, :p, :v) ON DUPLICATE KEY UPDATE settingValue = :v',
            ['k' => $key, 'p' => $this->portals->scopeId(), 'v' => $value],
        );
    }
}
