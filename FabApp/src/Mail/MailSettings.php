<?php

namespace App\Mail;

use App\Service\SiteSettingService;

/**
 * The sender account an admin configures before FabOS can send anything: a
 * Symfony transport DSN plus the identity mail goes out as.
 *
 * Kept in SITE_SETTING, so it inherits the portal scoping for free — a portal
 * with its own rows sends as itself, everyone else falls back to the site-wide
 * account. Nothing is sent while isConfigured() is false.
 */
final class MailSettings
{
    private const DSN_KEY = 'mail_transport_dsn';
    private const FROM_ADDRESS_KEY = 'mail_from_address';
    private const FROM_NAME_KEY = 'mail_from_name';
    private const REPLY_TO_KEY = 'mail_reply_to';

    public function __construct(private readonly SiteSettingService $settings)
    {
    }

    public function getTransportDsn(): string
    {
        return trim($this->settings->get(self::DSN_KEY) ?? '');
    }

    public function getFromAddress(): string
    {
        return trim($this->settings->get(self::FROM_ADDRESS_KEY) ?? '');
    }

    public function getFromName(): string
    {
        $name = trim($this->settings->get(self::FROM_NAME_KEY) ?? '');

        return $name !== '' ? $name : 'FabOS';
    }

    public function getReplyTo(): string
    {
        return trim($this->settings->get(self::REPLY_TO_KEY) ?? '');
    }

    /** A transport and a sender address are the minimum for any mail to go out. */
    public function isConfigured(): bool
    {
        return $this->getTransportDsn() !== '' && $this->getFromAddress() !== '';
    }

    public function save(string $dsn, string $fromAddress, string $fromName, string $replyTo): void
    {
        $this->settings->set(self::DSN_KEY, trim($dsn));
        $this->settings->set(self::FROM_ADDRESS_KEY, trim($fromAddress));
        $this->settings->set(self::FROM_NAME_KEY, trim($fromName));
        $this->settings->set(self::REPLY_TO_KEY, trim($replyTo));
    }

    /**
     * The DSN with its password replaced by dots, for display. The admin form posts
     * this masked value back untouched when it wasn't edited (see AdminController),
     * so the real password never has to round-trip through the browser.
     */
    public function getMaskedTransportDsn(): string
    {
        return self::mask($this->getTransportDsn());
    }

    public static function mask(string $dsn): string
    {
        return (string) preg_replace('#^([a-z0-9+.-]+://[^:/@]+):[^@]*@#i', '$1:••••••@', $dsn);
    }

    public static function isMasked(string $dsn): bool
    {
        return str_contains($dsn, '••••••@');
    }
}
