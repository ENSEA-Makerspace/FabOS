<?php

namespace App\Mail;

use App\Entity\Utilisateur;
use App\Mail\Message\SendMailMessage;
use App\Service\SiteSettingService;
use Symfony\Component\Messenger\MessageBusInterface;

/**
 * The one way anything in FabOS sends mail. Callers hand over a recipient, a
 * template name and a context; this decides whether the mail may go out at all,
 * writes it to the audit log and queues it.
 *
 * Nothing is sent when no sender account is configured, or while sending is
 * paused — a lab that hasn't set up SMTP silently sends nothing rather than
 * throwing from the middle of a booking.
 */
final class Mailer
{
    public function __construct(
        private readonly MailSettings $settings,
        private readonly MailLog $log,
        private readonly MailSender $sender,
        private readonly MessageBusInterface $bus,
        private readonly SiteSettingService $siteSettings,
        private readonly NotificationPreferences $preferences,
    ) {
    }

    /**
     * Whether mail can actually go out: a sender account is set up and sending
     * has not been paused.
     *
     * Mail is **kernel**, not a module. Booking confirmations, event
     * registrations and password-shaped mail are transactional — an install
     * should not be able to stop them by flipping a feature toggle, which is
     * what the old `emails` module allowed. "This install doesn't send mail" is
     * expressed by having no sender account; "stop sending for now" is the
     * pause switch.
     */
    public function isOperational(): bool
    {
        return $this->settings->isConfigured() && !$this->settings->isPaused();
    }

    /**
     * Mail to a known user, in their own language.
     *
     * $transactional marks mail the user cannot opt out of (password reset, a
     * booking they just made). Everything else — reminders, digests, announcements
     * — respects their notification preferences: first the master switch on the
     * account, then the per-category opt-out.
     */
    public function queueToUser(Utilisateur $user, string $template, array $context = [], string $category = NotificationCategory::GENERAL, bool $transactional = true): bool
    {
        $userId = $user->getId();

        if (!$transactional) {
            if (!$user->isNotificationEmail()) {
                return false;
            }

            if ($userId !== null && !$this->preferences->accepts($userId, $category)) {
                return false;
            }
        }

        $name = trim(($user->getFirstName() ?? '') . ' ' . ($user->getLastName() ?? ''));

        return $this->queue($user->getEmail(), $name !== '' ? $name : null, $template, $context, $category, $user->getLangue(), $userId);
    }

    /** @param array<string, mixed> $context */
    public function queue(string $to, ?string $toName, string $template, array $context = [], string $category = NotificationCategory::GENERAL, ?string $locale = null, ?int $userId = null): bool
    {
        $logId = $this->record($to, $toName, $template, $context, $category, $locale, $userId);
        if ($logId === null) {
            return false;
        }

        $this->bus->dispatch(new SendMailMessage($logId));

        return true;
    }

    /**
     * Sends without going through the queue and reports what happened — for the
     * admin's "send a test" button, where the whole point is to see the transport
     * error rather than to have it retried out of sight.
     *
     * @param array<string, mixed> $context
     *
     * @return string|null null on success, otherwise the failure to show the admin
     */
    public function sendNow(string $to, ?string $toName, string $template, array $context = [], string $category = NotificationCategory::TEST, ?string $locale = null): ?string
    {
        $logId = $this->record($to, $toName, $template, $context, $category, $locale);
        if ($logId === null) {
            return 'Mail is not available: check that a sender account is configured and that sending is not paused.';
        }

        try {
            $this->sender->send($logId);
        } catch (\Throwable $e) {
            return $e->getMessage();
        }

        $row = $this->log->find($logId);

        return ($row['status'] ?? null) === MailLog::STATUS_SENT ? null : (string) ($row['error'] ?? 'Unknown error.');
    }

    /**
     * @param array<string, mixed> $context
     *
     * @return int|null the log id, or null when this mail must not be sent
     */
    private function record(string $to, ?string $toName, string $template, array $context, string $category, ?string $locale, ?int $userId = null): ?int
    {
        if (!$this->isOperational() || filter_var($to, FILTER_VALIDATE_EMAIL) === false) {
            return null;
        }

        try {
            return $this->log->queue($to, $toName, $template, $context, $locale ?? $this->siteSettings->getDefaultLocale(), $category, $userId);
        } catch (\Throwable) {
            // The log is the queue: if it can't be written, there is nothing to send.
            return null;
        }
    }
}
