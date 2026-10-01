<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\Machine;
use App\Entity\Place;
use App\Event\EventArtwork;
use App\Feature\SiteFeatureService;
use App\Repository\EventRepository;
use App\Repository\MachineRepository;
use App\Repository\PlaceRepository;
use App\Reservation\NextFreeSlotService;
use App\Reservation\ReservableType;
use App\Schedule\ScheduleResolver;
use App\Service\SiteSettingService;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * « Accueil public » (proposition du 2026-10-01, planche `01-accueil-configurable.png`) :
 * ce qu'un VISITEUR anonyme lit en arrivant — l'état du jour, les espaces et les
 * équipements avec leur état, les prochains événements.
 *
 * Aucun calcul neuf : les mêmes services que `/places` et `/machines`
 * (`ScheduleResolver::isOpenAt`, `NextFreeSlotService::find` avec user = null,
 * mêmes mots `state.*`). Lecture seule.
 */
final class PublicHome
{
    private const PER_ROW = 4;

    public function __construct(
        private readonly ScheduleResolver $schedule,
        private readonly NextFreeSlotService $nextFreeSlot,
        private readonly PlaceRepository $places,
        private readonly MachineRepository $machines,
        private readonly EventRepository $events,
        private readonly EventArtwork $artwork,
        private readonly SiteFeatureService $features,
        private readonly SiteSettingService $siteSettings,
        private readonly TranslatorInterface $translator,
    ) {
    }

    /**
     * @return array{today: array{open: bool, openNow: bool, label: string, hours: ?string, reason: ?string}, places: ?array{cards: list<array<string, mixed>>, total: int}, machines: ?array{cards: list<array<string, mixed>>, total: int}, events: list<array{event: \App\Entity\Event, thumb: ?string}>}
     */
    public function build(): array
    {
        $zone = new \DateTimeZone($this->siteSettings->getTimezone());
        $now = new \DateTimeImmutable('now', $zone);
        $openNow = $this->schedule->isOpenAt(null, $now);

        $placeCards = null;
        if ($this->features->allowsSurface('places')) {
            $rows = $this->places->findLive([], ['nom' => 'ASC']);
            $placeCards = ['total' => \count($rows), 'cards' => []];
            foreach (\array_slice($rows, 0, self::PER_ROW) as $place) {
                /** @var Place $place */
                $slot = $this->nextFreeSlot->find(null, ReservableType::Place, (int) $place->getId());
                $placeCards['cards'][] = ['place' => $place] + $this->state($slot, $openNow, false, $now);
            }
        }

        $machineCards = null;
        if ($this->features->allowsSurface('machines')) {
            $rows = $this->machines->findLive([], ['nom' => 'ASC']);
            $machineCards = ['total' => \count($rows), 'cards' => []];
            foreach (\array_slice($rows, 0, self::PER_ROW) as $machine) {
                /** @var Machine $machine */
                $down = \in_array(strtolower($machine->getStatut()), ['maintenance', 'panne'], true);
                $slot = $down ? null : $this->nextFreeSlot->find(null, ReservableType::Machine, (int) $machine->getId());
                $machineCards['cards'][] = ['machine' => $machine] + $this->state($slot, $openNow, $down, $now);
            }
        }

        $events = [];
        foreach ($this->events->findUpcoming(3) as $event) {
            $events[] = ['event' => $event, 'thumb' => $this->artwork->describe($event)['thumb']];
        }

        return [
            'today' => $this->today($now, $openNow),
            'places' => $placeCards,
            'machines' => $machineCards,
            'events' => $events,
        ];
    }

    /**
     * L'état d'une chose, avec les mots de `/places` et `/machines` — mais en
     * disant QUAND quand ce n'est pas maintenant (« Libre demain 09:00 »).
     *
     * @param array{start: \DateTimeImmutable, end: \DateTimeImmutable}|null $slot
     * @return array{state: string, tone: string, freeNow: bool}
     */
    private function state(?array $slot, bool $openNow, bool $down, \DateTimeImmutable $now): array
    {
        if ($down) {
            return ['state' => $this->translator->trans('machines.state_down'), 'tone' => 'stop', 'freeNow' => false];
        }
        // Même règle que les deux pages : « libre » = libre MAINTENANT (dans l'heure).
        if ($openNow && $slot !== null && $slot['start'] <= $now->modify('+60 minutes')) {
            return ['state' => $this->translator->trans('state.free'), 'tone' => 'go', 'freeNow' => true];
        }
        if ($slot === null) {
            return ['state' => $this->translator->trans('state.full'), 'tone' => 'wait', 'freeNow' => false];
        }
        $start = $slot['start']->setTimezone($now->getTimezone());
        $time = $start->format('H:i');
        $dayDiff = (int) $now->setTime(0, 0)->diff($start->setTime(0, 0))->format('%r%a');
        $label = match (true) {
            $dayDiff <= 0 => $this->translator->trans('state.free_at', ['%time%' => $time]),
            $dayDiff === 1 => 'Libre demain ' . $time,
            default => $this->translator->trans('state.free_on', ['%day%' => $start->format('d/m'), '%time%' => $time]),
        };

        return ['state' => $label, 'tone' => 'wait', 'freeNow' => false];
    }

    /** @return array{open: bool, openNow: bool, label: string, hours: ?string, reason: ?string} */
    private function today(\DateTimeImmutable $now, bool $openNow): array
    {
        $minutes = $this->schedule->openMinutesFor(null, $now);
        $fmt = static fn (int $m): string => intdiv($m, 60) . ':' . str_pad((string) ($m % 60), 2, '0', STR_PAD_LEFT);
        $minuteNow = (int) $now->format('H') * 60 + (int) $now->format('i');

        // Ouvert aujourd'hui : il reste (ou il y a eu) des heures ce jour — tant
        // que la fermeture n'est pas passée.
        if ($minutes !== null && $minuteNow < $minutes['end']) {
            return [
                'open' => true,
                'openNow' => $openNow,
                'label' => 'Ouvert aujourd’hui',
                'hours' => $fmt($minutes['start']) . '–' . $fmt($minutes['end']),
                'reason' => null,
            ];
        }

        // Fermé : dire pourquoi s'il y a une raison, et QUAND on rouvre.
        $reason = $minutes === null ? $this->schedule->closureReasonFor(null, $now) : null;
        $formatter = new \IntlDateFormatter('fr_FR', \IntlDateFormatter::NONE, \IntlDateFormatter::NONE, $now->getTimezone(), null, 'EEEE');
        $reopens = null;
        for ($i = 1; $i <= 14 && $reopens === null; $i++) {
            $day = $now->modify('+' . $i . ' days');
            $next = $this->schedule->openMinutesFor(null, $day);
            if ($next !== null) {
                $name = $i === 1 ? 'demain' : (string) $formatter->format($day);
                $reopens = 'rouvre ' . $name . ' ' . $fmt($next['start']);
            }
        }

        return [
            'open' => false,
            'openNow' => false,
            'label' => 'Fermé',
            'hours' => $reopens,
            'reason' => $reason,
        ];
    }
}
