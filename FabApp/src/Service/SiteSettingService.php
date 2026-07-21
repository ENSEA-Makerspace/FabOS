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
}
