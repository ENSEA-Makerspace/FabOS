<?php

declare(strict_types=1);

namespace App\Page;

use App\Entity\Machine;
use App\Feature\SiteFeatureService;
use App\Repository\EventRepository;
use App\Repository\MachineRepository;
use App\Schedule\ScheduleResolver;
use App\Service\SiteSettingService;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * L'accueil d'une borne tactile (proposition `kiosque-accueil`, 2026-10-01,
 * d'après `productwide/07-kiosque.png` et `espaces/07-kiosque-entree.png`).
 *
 * Lecture seule, aucune donnée personnelle : l'état du lieu (ScheduleResolver),
 * la semaine d'ouverture, le nombre d'événements à venir et de machines — ce que
 * lisent déjà `/kiosk/events` et `/kiosk/machine/{id}`. Rien du journal RFID.
 */
final class KioskHome
{
    public function __construct(
        private readonly ScheduleResolver $schedule,
        private readonly EventRepository $events,
        private readonly MachineRepository $machines,
        private readonly SiteFeatureService $features,
        private readonly SiteSettingService $siteSettings,
        private readonly TranslatorInterface $translator,
        private readonly \App\Service\Checkins $checkins,
    ) {
    }

    /**
     * @return array{
     *   status: array{open: bool, signal: string, label: string, detail: ?string},
     *   week: list<array{day: string, hours: ?string, today: bool}>,
     *   events: ?array{count: int, next: ?string},
     *   machines: ?array{count: int, down: int, firstId: ?int}
     * }
     */
    public function build(): array
    {
        $now = new \DateTimeImmutable('now', new \DateTimeZone($this->siteSettings->getTimezone()));

        $events = null;
        if ($this->features->allowsSurface('events')) {
            $rows = $this->events->findUpcoming(8);
            $events = ['count' => \count($rows), 'next' => $rows === [] ? null : $rows[0]->getTitre()];
        }

        $machines = null;
        if ($this->features->allowsSurface('machines')) {
            $rows = $this->machines->findLive([], ['nom' => 'ASC']);
            $down = 0;
            foreach ($rows as $machine) {
                /** @var Machine $machine */
                if (\in_array(strtolower($machine->getStatut()), ['maintenance', 'panne'], true)) {
                    ++$down;
                }
            }
            $machines = ['count' => \count($rows), 'down' => $down, 'firstId' => $rows === [] ? null : $rows[0]->getId()];
        }

        // S207 — la tuile « Je suis là » : seulement si le Check-in est allumé ET sa table présente.
        $checkin = $this->features->allowsSurface('checkin') && $this->checkins->isReady();

        return ['status' => $this->status($now), 'week' => $this->week($now), 'events' => $events, 'machines' => $machines, 'checkin' => $checkin];
    }

    /** @return array{open: bool, signal: string, label: string, detail: ?string} */
    private function status(\DateTimeImmutable $now): array
    {
        $minuteNow = (int) $now->format('H') * 60 + (int) $now->format('i');
        $intervals = $this->schedule->openIntervalsFor(null, $now);
        foreach ($intervals as $interval) {
            if ($minuteNow >= $interval['start'] && $minuteNow < $interval['end']) {
                return ['open' => true, 'signal' => 'go', 'label' => $this->translator->trans('kiosk_home.open_until', ['%time%' => self::clock($interval['end'])]), 'detail' => null];
            }
        }

        // Fermé : dire pourquoi s'il y a une raison, et QUAND on rouvre.
        $reason = $intervals === [] ? $this->schedule->closureReasonFor(null, $now) : null;
        foreach ($intervals as $interval) {
            if ($interval['start'] > $minuteNow) {
                return ['open' => false, 'signal' => 'wait', 'label' => $this->translator->trans('kiosk_home.closed_opens_at', ['%time%' => self::clock($interval['start'])]), 'detail' => $reason];
            }
        }
        $formatter = new \IntlDateFormatter($this->translator->getLocale(), \IntlDateFormatter::NONE, \IntlDateFormatter::NONE, $now->getTimezone(), null, 'EEEE');
        for ($i = 1; $i <= 14; ++$i) {
            $day = $now->modify('+' . $i . ' days');
            $span = $this->schedule->openMinutesFor(null, $day);
            if ($span !== null) {
                $label = $i === 1
                    ? $this->translator->trans('kiosk_home.closed_reopens_tomorrow', ['%time%' => self::clock($span['start'])])
                    : $this->translator->trans('kiosk_home.closed_reopens_day', ['%day%' => (string) $formatter->format($day), '%time%' => self::clock($span['start'])]);

                return ['open' => false, 'signal' => 'stop', 'label' => $label, 'detail' => $reason];
            }
        }

        return ['open' => false, 'signal' => 'stop', 'label' => $this->translator->trans('kiosk_home.closed'), 'detail' => $reason];
    }

    /** @return list<array{day: string, hours: ?string, today: bool}> */
    private function week(\DateTimeImmutable $now): array
    {
        $formatter = new \IntlDateFormatter($this->translator->getLocale(), \IntlDateFormatter::NONE, \IntlDateFormatter::NONE, $now->getTimezone(), null, 'EEEE');
        $week = [];
        for ($i = 0; $i < 7; ++$i) {
            $day = $now->modify('+' . $i . ' days');
            $parts = [];
            foreach ($this->schedule->openIntervalsFor(null, $day) as $interval) {
                $parts[] = self::clock($interval['start']) . '–' . self::clock($interval['end']);
            }
            $week[] = ['day' => ucfirst((string) $formatter->format($day)), 'hours' => $parts === [] ? null : implode(' · ', $parts), 'today' => $i === 0];
        }

        return $week;
    }

    private static function clock(int $minutes): string
    {
        return sprintf('%d:%02d', intdiv($minutes, 60), $minutes % 60);
    }
}
