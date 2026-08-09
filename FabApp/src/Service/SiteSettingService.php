<?php

namespace App\Service;

use Doctrine\DBAL\Connection;

/**
 * Small generic key/value store for site-wide settings (currently just the default
 * locale). Mirrors SiteFeatureService's fail-safe pattern: a missing table/row falls back
 * to a sane default instead of breaking the site.
 *
 * Each key has one instance-wide value.
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
    private const BOOKING_IDENTITY_ROLES_KEY = 'booking_identity_roles';
    private const TIMEZONE_KEY = 'timezone';
    private const DEVELOPMENT_MODE_KEY = 'development_mode';
    private const USAGE_RIGHTS_ENFORCED_KEY = 'usage_rights_enforced';

    /**
     * The zone the lab lives in — the wall-clock a displayed time means.
     *
     * ⚠️ Read the audit in `docs/HISTORY.md` (S38b) before touching anything that
     * uses this. It is **not** a value to hand to every `|date()` call: the database
     * holds two conventions, and only one of them needs converting.
     *
     *  - Machine timestamps (`createdAt`, `LogUtilisation`, `Progression`, and every
     *    `CURRENT_TIMESTAMP` default — MariaDB is on UTC too) are stored **UTC** and
     *    must be converted for display. That is what `|lab_date()` is for.
     *  - Human-entered wall-clock (`Reservation`, `Event`, `OpeningHours`, access
     *    passes) is stored **already in lab time** and must be rendered raw with
     *    plain `|date()`. Converting it shifts it twice — opening hours would go
     *    from 08:00 to 10:00.
     *
     * ⚠️ And do not apply it with `date_default_timezone_set()`. Tried and reverted
     * 2026-08-01: it moves the read and the hydration together, so nothing appears
     * to change while every stored date silently changes meaning.
     */
    private const FALLBACK_TIMEZONE = 'Europe/Paris';

    /**
     * Who may see *who* booked a slot, as a list of security roles.
     *
     * Not a boolean and not a hardcoded "staff only", because the answer differs by
     * institution: a school may want trainers to see the names while students see
     * only that a slot is taken, an association may want every member to see them.
     * So it is a list the operator ticks, drawn from the `ROLE` table they already
     * edit.
     *
     * ⚠️ The default is deliberately the **restrictive** one. Before this setting
     * existed `/calendrier` published every booker's real name and free-text motif
     * to anonymous visitors from the internet (S38); an install that never opens the
     * screen must land on the safe side of that, not inherit the leak.
     */
    private const FALLBACK_BOOKING_IDENTITY_ROLES = ['ROLE_STAFF', 'ROLE_ADMIN'];

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

    /**
     * Always a zone PHP knows: a stored value that is no longer a valid identifier
     * (a typo, or one dropped by a tzdata update) falls back rather than throwing on
     * every page that shows a date.
     */
    public function getTimezone(): string
    {
        $value = trim($this->get(self::TIMEZONE_KEY) ?? '');

        return self::isValidTimezone($value) ? $value : self::FALLBACK_TIMEZONE;
    }

    public function setTimezone(string $timezone): void
    {
        $timezone = trim($timezone);

        if (!self::isValidTimezone($timezone)) {
            throw new \InvalidArgumentException(sprintf('Unknown timezone "%s".', $timezone));
        }

        $this->set(self::TIMEZONE_KEY, $timezone);
    }


    public static function isValidTimezone(string $timezone): bool
    {
        return $timezone !== '' && in_array($timezone, \DateTimeZone::listIdentifiers(), true);
    }

    /**
     * @return string[] security roles, never empty
     *
     * An empty stored value means "nobody but the owner of the booking", which is a
     * legitimate choice — but it is stored as the explicit marker '-' rather than an
     * empty string, because an empty string is also what a missing row and an
     * unreachable database look like, and those two must fall back to the default
     * instead of silently hiding names from the staff who need them.
     */
    public function getBookingIdentityRoles(): array
    {
        $raw = trim($this->get(self::BOOKING_IDENTITY_ROLES_KEY) ?? '');

        if ($raw === '') {
            return self::FALLBACK_BOOKING_IDENTITY_ROLES;
        }

        if ($raw === '-') {
            return [];
        }

        $roles = array_values(array_filter(array_map(
            static fn (string $role): string => strtoupper(trim($role)),
            explode(',', $raw),
        )));

        return $roles !== [] ? $roles : self::FALLBACK_BOOKING_IDENTITY_ROLES;
    }

    /** @param string[] $roles */
    public function setBookingIdentityRoles(array $roles): void
    {
        $roles = array_values(array_unique(array_filter(array_map(
            static fn (string $role): string => strtoupper(trim($role)),
            $roles,
        ))));

        $this->set(self::BOOKING_IDENTITY_ROLES_KEY, $roles === [] ? '-' : implode(',', $roles));
    }

    public function setVocabulary(string $orgName, string $venueLabel): void
    {
        $this->set(self::ORG_NAME_KEY, trim($orgName));
        $this->set(self::VENUE_LABEL_KEY, trim($venueLabel));
    }

    /**
     * Shows the admin-only development navigation. This is intentionally a
     * presentation switch: it never relaxes a route, firewall, or permission.
     * It defaults to off so a production install does not advertise internal
     * tools when the setting has never been configured.
     */
    public function isDevelopmentMode(): bool
    {
        return $this->get(self::DEVELOPMENT_MODE_KEY) === '1';
    }

    public function setDevelopmentMode(bool $enabled): void
    {
        $this->set(self::DEVELOPMENT_MODE_KEY, $enabled ? '1' : '0');
    }

    /**
     * Kept opt-in: enabling it changes the default from current site access to
     * explicit package grants. Administrators retain operational recovery.
     */
    public function isUsageRightsEnforced(): bool
    {
        return $this->get(self::USAGE_RIGHTS_ENFORCED_KEY) === '1';
    }

    public function setUsageRightsEnforced(bool $enabled): void
    {
        $this->set(self::USAGE_RIGHTS_ENFORCED_KEY, $enabled ? '1' : '0');
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
                'SELECT settingValue FROM SITE_SETTING WHERE settingKey = :k',
                ['k' => $key],
            );
        } catch (\Throwable) {
            return null;
        }

        return is_string($value) ? $value : null;
    }

    public function set(string $key, string $value): void
    {
        $this->db->executeStatement(
            'INSERT INTO SITE_SETTING (settingKey, settingValue) VALUES (:k, :v) ON DUPLICATE KEY UPDATE settingValue = :v',
            ['k' => $key, 'v' => $value],
        );
    }
}
