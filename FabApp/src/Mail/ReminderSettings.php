<?php

namespace App\Mail;

use App\Service\SiteSettingService;

/**
 * What the admin gets to decide about scheduled reminders: which ones go out at
 * all, and how far ahead.
 *
 * Lives in SITE_SETTING for the same reason the sender account does — it
 * stays in the shared setting store, and a lab can retune its lead times without
 * a deploy. Every reminder ships **off by default**: a lab that upgrades into
 * this version should not discover it by having FabOS mail its whole member
 * list about bookings they already know about.
 */
final class ReminderSettings
{
    public const BOOKING = 'booking';
    public const LOAN_DUE = 'loan_due';
    public const LOAN_OVERDUE = 'loan_overdue';
    public const MAINTENANCE_OVERDUE = 'maintenance_overdue';
    public const EVENT = 'event';

    /** Every reminder the scanner knows how to send, in the order the admin sees them. */
    public const KINDS = [self::BOOKING, self::EVENT, self::LOAN_DUE, self::LOAN_OVERDUE, self::MAINTENANCE_OVERDUE];

    private const ENABLED_PREFIX = 'reminder_enabled_';
    private const BOOKING_LEAD_KEY = 'reminder_booking_lead_hours';
    private const LOAN_LEAD_KEY = 'reminder_loan_lead_days';
    private const EVENT_LEAD_KEY = 'reminder_event_lead_hours';

    public const DEFAULT_BOOKING_LEAD_HOURS = 24;
    public const DEFAULT_LOAN_LEAD_DAYS = 2;
    public const DEFAULT_EVENT_LEAD_HOURS = 24;

    public function __construct(private readonly SiteSettingService $settings)
    {
    }

    public function isEnabled(string $kind): bool
    {
        return in_array($kind, self::KINDS, true)
            && ($this->settings->get(self::ENABLED_PREFIX . $kind) ?? '0') === '1';
    }

    /** @return array<string, bool> kind => enabled, for the admin form. */
    public function all(): array
    {
        $state = [];
        foreach (self::KINDS as $kind) {
            $state[$kind] = $this->isEnabled($kind);
        }

        return $state;
    }

    public function anyEnabled(): bool
    {
        return in_array(true, $this->all(), true);
    }

    /**
     * How long before a booking starts its reminder goes out. Clamped to a
     * window the scanner can actually serve: below an hour a timer that ticks
     * hourly would miss bookings entirely, and beyond a week a "reminder" is
     * just noise.
     */
    public function getBookingLeadHours(): int
    {
        return $this->clamp($this->settings->get(self::BOOKING_LEAD_KEY), self::DEFAULT_BOOKING_LEAD_HOURS, 1, 168);
    }

    /** How many days before its expected return a loan is flagged as coming due. */
    public function getLoanLeadDays(): int
    {
        return $this->clamp($this->settings->get(self::LOAN_LEAD_KEY), self::DEFAULT_LOAN_LEAD_DAYS, 0, 30);
    }

    /** How long before an event its registrants are reminded. Same window rules as bookings. */
    public function getEventLeadHours(): int
    {
        return $this->clamp($this->settings->get(self::EVENT_LEAD_KEY), self::DEFAULT_EVENT_LEAD_HOURS, 1, 168);
    }

    /** @param array<string, bool> $enabled kind => enabled */
    public function save(array $enabled, int $bookingLeadHours, int $loanLeadDays, int $eventLeadHours = self::DEFAULT_EVENT_LEAD_HOURS): void
    {
        foreach (self::KINDS as $kind) {
            $this->settings->set(self::ENABLED_PREFIX . $kind, !empty($enabled[$kind]) ? '1' : '0');
        }

        $this->settings->set(self::BOOKING_LEAD_KEY, (string) $this->clamp((string) $bookingLeadHours, self::DEFAULT_BOOKING_LEAD_HOURS, 1, 168));
        $this->settings->set(self::LOAN_LEAD_KEY, (string) $this->clamp((string) $loanLeadDays, self::DEFAULT_LOAN_LEAD_DAYS, 0, 30));
        $this->settings->set(self::EVENT_LEAD_KEY, (string) $this->clamp((string) $eventLeadHours, self::DEFAULT_EVENT_LEAD_HOURS, 1, 168));
    }

    private function clamp(?string $raw, int $default, int $min, int $max): int
    {
        if ($raw === null || trim($raw) === '' || !is_numeric(trim($raw))) {
            return $default;
        }

        return max($min, min($max, (int) trim($raw)));
    }
}
