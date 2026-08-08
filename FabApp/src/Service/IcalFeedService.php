<?php

namespace App\Service;


use App\Entity\Reservation;

/**
 * Builds a read-only RFC 5545 iCalendar (.ics) feed for a single reservable
 * resource (machine or space). Feeds are privacy-safe: they expose only the
 * busy time slots, never who booked them.
 *
 * Times are emitted as UTC, so the feed carries no VTIMEZONE and stays correct
 * whatever zone the operator configures,
 * matching how the app stores and displays reservations everywhere else.
 */
final class IcalFeedService
{
    public function __construct(private readonly SiteSettingService $siteSettings)
    {
    }

    /**
     * The zone label the feed advertises. iCal needs the zone *named*, not just
     * applied: the stored strings are already lab wall-clock, so the feed emits them
     * verbatim and tells the calendar client which zone to read them in via TZID.
     */
    private function tzid(): string
    {
        return $this->siteSettings->getTimezone();
    }
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
            'X-WR-TIMEZONE:' . $this->tzid(),
        ];

        foreach ($reservations as $reservation) {
            $id = $reservation->getId();
            if ($id === null) {
                continue;
            }

            $summary = $resourceName !== '' ? $resourceName . ' — ' . $busyLabel : $busyLabel;

            $lines[] = 'BEGIN:VEVENT';
            $lines[] = 'UID:reservation-' . $id . '@fabos';
            $lines[] = 'DTSTAMP:' . $now;
            $lines[] = 'DTSTART:' . $this->utcStamp($reservation->getDateDebut());
            $lines[] = 'DTEND:' . $this->utcStamp($reservation->getDateFin());
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
     * The stored value is lab wall-clock, emitted as UTC.
     *
     * ⚠️ Previously the feed emitted the wall-clock verbatim under a `TZID`, with a
     * hand-written `VTIMEZONE` carrying CET/CEST offsets and the EU daylight-saving
     * rules. That was correct only for Europe/Paris: once the zone became an
     * operator setting, a lab in another country would have advertised its own TZID
     * against Paris's offsets. Emitting UTC needs no VTIMEZONE, is unambiguous for
     * every client, and cannot drift when a country changes its DST rules.
     *
     * The reinterpretation is deliberate: the object carries the server's zone but
     * its *value* is lab wall-clock, so the naive string is re-read in the lab zone
     * before converting.
     */
    private function utcStamp(\DateTimeInterface $dt): string
    {
        return (new \DateTimeImmutable(
            $dt->format('Y-m-d H:i:s'),
            new \DateTimeZone($this->tzid()),
        ))->setTimezone(new \DateTimeZone('UTC'))->format('Ymd\THis\Z');
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
