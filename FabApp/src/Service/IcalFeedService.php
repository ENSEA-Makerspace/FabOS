<?php

namespace App\Service;

use App\Entity\Reservation;

/**
 * Builds a read-only RFC 5545 iCalendar (.ics) feed for a single reservable
 * resource (machine or space). Feeds are privacy-safe: they expose only the
 * busy time slots, never who booked them.
 *
 * Times are emitted as Europe/Paris local time with an embedded VTIMEZONE,
 * matching how the app stores and displays reservations everywhere else.
 */
final class IcalFeedService
{
    private const TZID = 'Europe/Paris';
    private const PRODID = '-//FabOS//Resource calendar//EN';

    /**
     * @param string          $resourceName Human label of the machine/space (e.g. "Découpe laser")
     * @param string          $busyLabel    Localised word shown as each event's title suffix (e.g. "Réservé")
     * @param Reservation[]   $reservations Active (non-cancelled) reservations for the resource
     */
    public function build(string $resourceName, string $busyLabel, array $reservations): string
    {
        $now = gmdate('Ymd\THis\Z');

        $lines = [
            'BEGIN:VCALENDAR',
            'VERSION:2.0',
            'PRODID:' . self::PRODID,
            'CALSCALE:GREGORIAN',
            'METHOD:PUBLISH',
            'X-WR-CALNAME:' . $this->escapeText($resourceName . ' — FabOS'),
            'X-WR-TIMEZONE:' . self::TZID,
        ];

        array_push($lines, ...$this->timezoneComponent());

        foreach ($reservations as $reservation) {
            $id = $reservation->getId();
            if ($id === null) {
                continue;
            }

            $summary = $resourceName !== '' ? $resourceName . ' — ' . $busyLabel : $busyLabel;

            $lines[] = 'BEGIN:VEVENT';
            $lines[] = 'UID:reservation-' . $id . '@fabos';
            $lines[] = 'DTSTAMP:' . $now;
            $lines[] = 'DTSTART;TZID=' . self::TZID . ':' . $this->localStamp($reservation->getDateDebut());
            $lines[] = 'DTEND;TZID=' . self::TZID . ':' . $this->localStamp($reservation->getDateFin());
            $lines[] = 'SUMMARY:' . $this->escapeText($summary);
            $lines[] = 'STATUS:' . ($reservation->getStatut() === 'pending' ? 'TENTATIVE' : 'CONFIRMED');
            $lines[] = 'TRANSP:OPAQUE';
            $lines[] = 'END:VEVENT';
        }

        $lines[] = 'END:VCALENDAR';

        // RFC 5545 requires CRLF line endings and 75-octet folding.
        return implode("\r\n", array_map($this->foldLine(...), $lines)) . "\r\n";
    }

    /**
     * Wall-clock components as stored (no timezone conversion): the value is
     * already the intended Europe/Paris local time, which we label via TZID.
     */
    private function localStamp(\DateTimeInterface $dt): string
    {
        return $dt->format('Ymd\THis');
    }

    /** @return string[] */
    private function timezoneComponent(): array
    {
        return [
            'BEGIN:VTIMEZONE',
            'TZID:' . self::TZID,
            'BEGIN:DAYLIGHT',
            'TZOFFSETFROM:+0100',
            'TZOFFSETTO:+0200',
            'TZNAME:CEST',
            'DTSTART:19700329T020000',
            'RRULE:FREQ=YEARLY;BYMONTH=3;BYDAY=-1SU',
            'END:DAYLIGHT',
            'BEGIN:STANDARD',
            'TZOFFSETFROM:+0200',
            'TZOFFSETTO:+0100',
            'TZNAME:CET',
            'DTSTART:19701025T030000',
            'RRULE:FREQ=YEARLY;BYMONTH=10;BYDAY=-1SU',
            'END:STANDARD',
            'END:VTIMEZONE',
        ];
    }

    private function escapeText(string $text): string
    {
        return str_replace(
            ["\\", "\n", "\r", ';', ','],
            ['\\\\', '\\n', '', '\\;', '\\,'],
            $text,
        );
    }

    /**
     * Fold a content line to <=75 octets per line, continuation lines start
     * with a space (RFC 5545 §3.1). Folds on byte boundaries safely for the
     * ASCII produced here; multibyte-aware to avoid splitting a UTF-8 sequence.
     */
    private function foldLine(string $line): string
    {
        if (strlen($line) <= 75) {
            return $line;
        }

        $chunks = [];
        $current = '';
        // First line may hold 75 octets; continuation lines carry a leading
        // space, so their content budget is 74.
        $limit = 75;
        foreach (mb_str_split($line) as $char) {
            if (strlen($current) + strlen($char) > $limit) {
                $chunks[] = $current;
                $current = '';
                $limit = 74;
            }
            $current .= $char;
        }
        $chunks[] = $current;

        return implode("\r\n ", $chunks);
    }
}
