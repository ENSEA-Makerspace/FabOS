<?php

namespace App\Mail;

/**
 * The kinds of mail FabOS sends, as far as a recipient is concerned.
 *
 * This is the vocabulary of the preference screen, so the split is by "would
 * someone want to mute this separately?" rather than by which module emits it.
 * Staff who are tired of maintenance chasers should not have to give up their
 * own booking reminders to get peace.
 *
 * The important line here is OPTOUTABLE. Everything else is transactional: mail
 * that answers something the person just did, and that they would be alarmed
 * not to receive. Silently dropping a booking confirmation because someone
 * unticked a box a year ago is worse than sending one mail too many, so those
 * categories deliberately have no off switch — and, being unsubscribable-from,
 * carry no unsubscribe link either.
 */
final class NotificationCategory
{
    /** Confirmations and answers for something the recipient just did. */
    public const BOOKING = 'booking';

    /** Scheduled nudges before a booking, or about a loan coming due. */
    public const REMINDER = 'reminder';

    /** Overdue-maintenance chasers, sent to staff at large. */
    public const MAINTENANCE = 'maintenance';

    /**
     * Event registration: confirmations, waitlist, "a seat opened".
     *
     * Transactional, so it is absent from OPTOUTABLE below. Someone who joined a
     * waitlist did so precisely to be told when a place frees up; letting a
     * preference suppress that would break the feature rather than honour a
     * choice.
     */
    public const EVENT = 'event';

    /** Digests and announcements. Nothing emits this yet; the switch exists first. */
    public const NEWS = 'news';

    /** Fallback for callers that don't say. */
    public const GENERAL = 'general';

    /** The admin's own test send — never subject to anybody's preferences. */
    public const TEST = 'test';

    /**
     * Categories a recipient may switch off, in the order the profile screen
     * shows them. Anything not listed here is transactional.
     */
    public const OPTOUTABLE = [self::REMINDER, self::MAINTENANCE, self::NEWS, self::GENERAL];

    public static function isOptOutable(string $category): bool
    {
        return in_array($category, self::OPTOUTABLE, true);
    }

    /** Translation key for the category's name on the preference screen. */
    public static function label(string $category): string
    {
        return 'notification.category.' . $category . '.label';
    }

    /** Translation key for the one-line explanation under it. */
    public static function description(string $category): string
    {
        return 'notification.category.' . $category . '.description';
    }
}
