<?php

namespace App\Service;

use Doctrine\DBAL\Connection;

/**
 * Small generic key/value store for site-wide settings (currently just the default
 * locale). Mirrors ModuleService's fail-safe pattern: a missing table/row falls back
 * to a sane default instead of breaking the site.
 */
final class SiteSettingService
{
    private const DEFAULT_LOCALE_KEY = 'default_locale';
    private const FALLBACK_LOCALE = 'fr';
    private const ALERT_BANNER_ENABLED_KEY = 'alert_banner_enabled';
    private const ALERT_BANNER_TEXT_KEY = 'alert_banner_text';
    private const LAB_PAGES_NAV_LABEL_KEY = 'lab_pages_nav_label';
    private const FALLBACK_LAB_PAGES_NAV_LABEL = 'Our Lab';

    public function __construct(private readonly Connection $db)
    {
    }

    public function getDefaultLocale(): string
    {
        try {
            $value = $this->db->fetchOne(
                'SELECT settingValue FROM SITE_SETTING WHERE settingKey = :k',
                ['k' => self::DEFAULT_LOCALE_KEY],
            );
        } catch (\Throwable) {
            return self::FALLBACK_LOCALE;
        }

        return is_string($value) && $value !== '' ? $value : self::FALLBACK_LOCALE;
    }

    public function setDefaultLocale(string $locale): void
    {
        $this->db->executeStatement(
            'INSERT INTO SITE_SETTING (settingKey, settingValue) VALUES (:k, :v) ON DUPLICATE KEY UPDATE settingValue = :v',
            ['k' => self::DEFAULT_LOCALE_KEY, 'v' => $locale],
        );
    }

    public function isAlertBannerEnabled(): bool
    {
        try {
            $value = $this->db->fetchOne(
                'SELECT settingValue FROM SITE_SETTING WHERE settingKey = :k',
                ['k' => self::ALERT_BANNER_ENABLED_KEY],
            );
        } catch (\Throwable) {
            return false;
        }

        return $value === '1';
    }

    public function getAlertBannerText(): string
    {
        try {
            $value = $this->db->fetchOne(
                'SELECT settingValue FROM SITE_SETTING WHERE settingKey = :k',
                ['k' => self::ALERT_BANNER_TEXT_KEY],
            );
        } catch (\Throwable) {
            return '';
        }

        return is_string($value) ? $value : '';
    }

    public function setAlertBanner(bool $enabled, string $text): void
    {
        $this->db->executeStatement(
            'INSERT INTO SITE_SETTING (settingKey, settingValue) VALUES (:k, :v) ON DUPLICATE KEY UPDATE settingValue = :v',
            ['k' => self::ALERT_BANNER_ENABLED_KEY, 'v' => $enabled ? '1' : '0'],
        );
        $this->db->executeStatement(
            'INSERT INTO SITE_SETTING (settingKey, settingValue) VALUES (:k, :v) ON DUPLICATE KEY UPDATE settingValue = :v',
            ['k' => self::ALERT_BANNER_TEXT_KEY, 'v' => $text],
        );
    }

    public function getLabPagesNavLabel(): string
    {
        try {
            $value = $this->db->fetchOne(
                'SELECT settingValue FROM SITE_SETTING WHERE settingKey = :k',
                ['k' => self::LAB_PAGES_NAV_LABEL_KEY],
            );
        } catch (\Throwable) {
            return self::FALLBACK_LAB_PAGES_NAV_LABEL;
        }

        return is_string($value) && trim($value) !== '' ? $value : self::FALLBACK_LAB_PAGES_NAV_LABEL;
    }

    public function setLabPagesNavLabel(string $label): void
    {
        $this->db->executeStatement(
            'INSERT INTO SITE_SETTING (settingKey, settingValue) VALUES (:k, :v) ON DUPLICATE KEY UPDATE settingValue = :v',
            ['k' => self::LAB_PAGES_NAV_LABEL_KEY, 'v' => trim($label)],
        );
    }
}
