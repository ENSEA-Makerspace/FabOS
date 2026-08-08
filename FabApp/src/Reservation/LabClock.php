<?php

namespace App\Reservation;

use App\Service\SiteSettingService;

/**
 * "Is this booking in the past?" — the one question S38b's audit did not answer.
 *
 * S38b classified every date column into two storage conventions and fixed how
 * they are **displayed**. It never looked at how they are **compared**, and that
 * is where the remaining skew lives.
 *
 * A `Reservation` is convention B: the wall-clock the member typed, stored
 * verbatim. Doctrine writes it with `format()` (no conversion) and hydrates it
 * with `createFromFormat()` and **no zone**, so it comes back carrying lab
 * digits under PHP's default label — UTC on this box. The digits are right,
 * which is why plain `|date()` renders it correctly; the *instant* is wrong by
 * the lab's UTC offset.
 *
 * ⚠️ So `$reservation->getDateDebut() <= new \DateTimeImmutable('now', $lab)` —
 * the shape `ApiController::cancelReservation` and `SiteController::myReservations`
 * both used — compares a fake instant against a real one and is **out by the
 * offset**. In Paris in August that is two hours, always in the permissive
 * direction: a booking looks two hours further in the future than it is, so a
 * finished slot sits in "À venir" and stays cancellable after it has started.
 * Nothing is wrongly refused, which is why it went unnoticed.
 *
 * ⚠️ **A freshly parsed date is not affected and must not be passed through
 * here.** `parseReservationDate()` builds the lab zone in, so its result is a
 * true instant already; `ReservationService::book()` comparing it against a real
 * `now` is correct as written. The skew is created by the round trip through the
 * database, so only *hydrated* values need `instantOf()`. Running a parsed date
 * through it would double-shift it — the same trap that stopped the mass-pinning
 * session in S38b.
 *
 * Scope: this fixes the booking verbs and the /mes-reservations grouping, which
 * is what S77 touches. `Event`, `OpeningHours` and access-pass validity are
 * convention B too and are compared the same skewed way elsewhere; that sweep is
 * recorded, not done here.
 */
final class LabClock
{
    public function __construct(private readonly SiteSettingService $siteSettings)
    {
    }

    public function zone(): \DateTimeZone
    {
        return new \DateTimeZone($this->siteSettings->getTimezone());
    }

    /** The current instant. Correct to compare against anything `instantOf()` returned. */
    public function now(): \DateTimeImmutable
    {
        return new \DateTimeImmutable('now', $this->zone());
    }

    /**
     * A convention-B value straight out of Doctrine, re-labelled as the instant
     * it actually means: keep the digits, replace the zone.
     */
    public function instantOf(\DateTimeInterface $stored): \DateTimeImmutable
    {
        return new \DateTimeImmutable($stored->format('Y-m-d H:i:s'), $this->zone());
    }

    /**
     * The inverse, for writing: a real instant expressed in the digits Doctrine
     * expects to store. Used by "terminer maintenant", which is the only verb
     * that writes `now` into a booking column.
     */
    public function storedFormOf(\DateTimeInterface $instant): \DateTimeImmutable
    {
        return new \DateTimeImmutable(
            \DateTimeImmutable::createFromInterface($instant)->setTimezone($this->zone())->format('Y-m-d H:i:s'),
        );
    }
}
