<?php

declare(strict_types=1);

namespace App\Page;

use App\Entity\Machine;
use App\Entity\Place;
use App\Feature\SiteFeatureService;
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
        private readonly SiteFeatureService $features,
        private readonly SiteSettingService $siteSettings,
        private readonly TranslatorInterface $translator,
    ) {
    }

    /**
     * Les rangées « Espaces » et « Équipements » de l'accueil : ce qui est libre
     * maintenant d'abord, puis ce qui se libère le plus tôt (2026-10-02, accueil
     * de la 0.5). Vu comme un visiteur : le droit de chacun se lit sur la fiche.
     *
     * @return array{places: ?array{cards: list<array<string, mixed>>, total: int}, machines: ?array{cards: list<array<string, mixed>>, total: int}}
     */
    public function freeNow(): array
    {
        $now = new \DateTimeImmutable('now', new \DateTimeZone($this->siteSettings->getTimezone()));
        $openNow = $this->schedule->isOpenAt(null, $now);
        $soonestFirst = static fn (array $a, array $b): int => [!$a['freeNow'], $a['at'] ?? PHP_INT_MAX] <=> [!$b['freeNow'], $b['at'] ?? PHP_INT_MAX];

        $placeCards = null;
        if ($this->features->allowsSurface('places')) {
            $cards = [];
            foreach ($this->places->findLive([], ['nom' => 'ASC']) as $place) {
                /** @var Place $place */
                $slot = $this->nextFreeSlot->find(null, ReservableType::Place, (int) $place->getId());
                $cards[] = ['place' => $place, 'at' => $slot ? $slot['start']->getTimestamp() : null] + $this->state($slot, $openNow, false, $now);
            }
            usort($cards, $soonestFirst);
            $placeCards = ['total' => \count($cards), 'cards' => \array_slice($cards, 0, self::PER_ROW)];
        }

        $machineCards = null;
        if ($this->features->allowsSurface('machines')) {
            $cards = [];
            foreach ($this->machines->findLive([], ['nom' => 'ASC']) as $machine) {
                /** @var Machine $machine */
                $down = \in_array(strtolower($machine->getStatut()), ['maintenance', 'panne'], true);
                $slot = $down ? null : $this->nextFreeSlot->find(null, ReservableType::Machine, (int) $machine->getId());
                $cards[] = ['machine' => $machine, 'at' => $slot ? $slot['start']->getTimestamp() : null] + $this->state($slot, $openNow, $down, $now);
            }
            usort($cards, $soonestFirst);
            // Une machine par catégorie d'abord : quatre imprimantes identiques
            // ne disent rien de plus qu'une.
            $picked = [];
            $seen = [];
            foreach ($cards as $i => $card) {
                $category = (string) $card['machine']->getCategorySlug();
                if (!isset($seen[$category]) && \count($picked) < self::PER_ROW) {
                    $seen[$category] = true;
                    $picked[$i] = $card;
                }
            }
            foreach ($cards as $i => $card) {
                if (\count($picked) >= self::PER_ROW) {
                    break;
                }
                $picked[$i] ??= $card;
            }
            ksort($picked);
            $machineCards = ['total' => \count($cards), 'cards' => array_values($picked)];
        }

        return ['places' => $placeCards, 'machines' => $machineCards];
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
            $dayDiff === 1 => $this->translator->trans('state.free_tomorrow', ['%time%' => $time]),
            default => $this->translator->trans('state.free_on', ['%day%' => $start->format('d/m'), '%time%' => $time]),
        };

        return ['state' => $label, 'tone' => 'wait', 'freeNow' => false];
    }
}
