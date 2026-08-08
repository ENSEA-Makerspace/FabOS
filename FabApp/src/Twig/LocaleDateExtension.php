<?php

namespace App\Twig;

use Symfony\Component\HttpFoundation\RequestStack;
use Twig\Extension\AbstractExtension;
use Twig\TwigFilter;

/**
 * `|loc_date()` — render a date with its **month and day names in the reader's
 * language**.
 *
 * **Why this exists.** Twig's `|date()` is PHP's `date()`, and PHP's `date()`
 * has exactly one language: English. So `|date('M')` on the event badges printed
 * `AUG` to a French reader looking at a French page, on a site whose default
 * locale is `fr` and which ships five translation catalogues. Nothing in the
 * translation layer could fix it — the string never went through the translator.
 *
 * **The pattern is ICU, not PHP.** `MMM` not `M`, `EEEE` not `l`, `d` not `j`.
 * That is deliberate and it is the same vocabulary `twig/intl-extra`'s
 * `format_datetime` uses, so a call site here reads the same as one anywhere else
 * in the Symfony world — and this file can be deleted the day that package is
 * installed. The tokens actually used on this site:
 *
 *     d      day of month, no padding        2
 *     dd     day of month, padded            02
 *     MMM    month, abbreviated              août
 *     MMMM   month, full                     août
 *     EEE    weekday, abbreviated            sam.
 *     EEEE   weekday, full                   samedi
 *     yyyy   year                            2026
 *     HH:mm  24-hour clock                   14:30
 *
 * ⚠️ **This filter does NOT convert timezones**, and that is not an oversight.
 * Everything it currently formats — `Event`, `OpeningHours`, `Reservation` — is
 * *human-entered wall-clock* already stored in lab time (convention B in
 * `LabTimeExtension`'s header). Converting would shift a 14:00 workshop to 16:00.
 * A machine timestamp that needs localised month names wants `|lab_date()` first;
 * add `|lab_loc_date` the day one turns up rather than adding a zone argument
 * here, so the call site keeps *stating* which convention it belongs to.
 *
 * ⚠️ The locale comes from the request, which is what the language switcher
 * writes (`app_switch_locale` → session `_locale`), so the filter follows the
 * reader and not the server.
 */
final class LocaleDateExtension extends AbstractExtension
{
    /**
     * Fallback token map, used only if ext-intl is missing. It buys English
     * output instead of a fatal error — the box has intl, but a fresh clone of
     * this repo somewhere else might not, and a homepage that 500s on a missing
     * PHP extension is a worse first impression than an English month name.
     */
    private const ICU_TO_PHP = [
        'EEEE' => 'l', 'EEE' => 'D', 'MMMM' => 'F', 'MMM' => 'M',
        'yyyy' => 'Y', 'yy' => 'y', 'dd' => 'd', 'd' => 'j',
        'MM' => 'm', 'HH' => 'H', 'mm' => 'i', 'ss' => 's',
    ];

    public function __construct(private readonly RequestStack $requestStack)
    {
    }

    public function getFilters(): array
    {
        return [
            new TwigFilter('loc_date', $this->locDate(...)),
        ];
    }

    /**
     * Null passes through as an empty string rather than "now" — same rule as
     * `|lab_date()`, and for the same reason: a nullable column like `dateFin`
     * must not silently invent a value.
     */
    public function locDate(\DateTimeInterface|string|null $date, string $pattern = 'd MMMM yyyy', ?string $locale = null): string
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

        $locale ??= $this->requestStack->getCurrentRequest()?->getLocale() ?? \Locale::getDefault();

        if (!class_exists(\IntlDateFormatter::class)) {
            return $date->format(strtr($pattern, self::ICU_TO_PHP));
        }

        $formatter = new \IntlDateFormatter(
            $locale,
            \IntlDateFormatter::NONE,
            \IntlDateFormatter::NONE,
            // The value's OWN zone — see the no-conversion warning above.
            $date->getTimezone(),
            \IntlDateFormatter::GREGORIAN,
            $pattern,
        );

        return $formatter->format($date) ?: $date->format(strtr($pattern, self::ICU_TO_PHP));
    }
}
