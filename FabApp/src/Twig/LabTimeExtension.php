<?php

namespace App\Twig;

use App\Service\SiteSettingService;
use Twig\Extension\AbstractExtension;
use Twig\TwigFilter;
use Twig\TwigFunction;

/**
 * `|lab_date()` — render a **machine timestamp** in the lab's configured timezone.
 *
 * **Why a named filter instead of `|date('H:i', tz)` everywhere.** This database
 * holds two conventions (the audit is S38b in `docs/HISTORY.md`), and the correct
 * treatment is opposite for each:
 *
 *   - **Machine timestamps** — `createdAt`, `created`, `updated`, `LogUtilisation`,
 *     `Progression`, `derniereConnexion`, and every `CURRENT_TIMESTAMP` default
 *     (MariaDB runs UTC too) — are stored as **UTC** wall-clock. They must be
 *     converted before display. **That is this filter.**
 *   - **Human-entered wall-clock** — `Reservation.dateDebut/dateFin`, `Event`,
 *     `OpeningHours`, access-pass validity — is stored **already in lab time**,
 *     because the parser pinned the lab zone on the way in so the naive string kept
 *     the time the human typed. It must be rendered by plain `|date()`.
 *
 * ⚠️ **Using the wrong one is silent and shifts by hours in the wrong direction.**
 * `|lab_date()` on a booking would show a 10:00 slot as 12:00; plain `|date()` on a
 * log shows a 10:41 scan as 08:41 — which is the bug this filter exists to fix. The
 * filter is named rather than parameterised precisely so the call site *states*
 * which convention it belongs to instead of leaving the next reader to trace the
 * value back to the code that wrote it.
 *
 * ⚠️ There is deliberately no global default-timezone switch. Setting
 * `date_default_timezone_set()` moves the read and Doctrine's hydration together —
 * display does not change, every stored date silently changes meaning, and the
 * Symfony form model timezone moves too, so new rows start being written under a
 * different convention from the old ones. Tried and reverted 2026-08-01.
 */
final class LabTimeExtension extends AbstractExtension
{
    public function __construct(private readonly SiteSettingService $siteSettings)
    {
    }

    public function getFilters(): array
    {
        return [
            new TwigFilter('lab_date', $this->labDate(...)),
        ];
    }

    public function getFunctions(): array
    {
        return [
            // The zone's own name, for the few places that need to *say* it rather
            // than format with it — the iCal VTIMEZONE component, and the settings
            // screen showing what is currently configured.
            new TwigFunction('lab_timezone', $this->siteSettings->getTimezone(...)),
        ];
    }

    /**
     * Mirrors Twig's own `|date` signature, so switching a call site is only the
     * filter name: `|date('d/m H:i')` becomes `|lab_date('d/m H:i')`.
     *
     * Null passes through as an empty string rather than "now" — Twig's `|date`
     * renders the current time for null, which on a nullable column like
     * `dateFin` would silently invent a value.
     */
    public function labDate(\DateTimeInterface|string|null $date, string $format = 'd/m/Y H:i'): string
    {
        if ($date === null || $date === '') {
            return '';
        }

        if (!$date instanceof \DateTimeInterface) {
            try {
                $date = new \DateTimeImmutable($date);
            } catch (\Throwable) {
                return '';
            }
        }

        return \DateTimeImmutable::createFromInterface($date)
            ->setTimezone(new \DateTimeZone($this->siteSettings->getTimezone()))
            ->format($format);
    }
}
