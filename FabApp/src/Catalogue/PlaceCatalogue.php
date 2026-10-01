<?php

declare(strict_types=1);

namespace App\Catalogue;

use App\Entity\Utilisateur;
use App\Repository\PlaceRepository;
use App\Reservation\NextFreeSlotService;
use App\Reservation\ReservableType;
use App\Schedule\ScheduleResolver;
use App\Service\SiteSettingService;
use App\UsageRights\UsageRightsService;
use App\Venue\VenueContext;
use Symfony\Component\HttpFoundation\Request;

/**
 * Ce que montre le catalogue des espaces (`/places`) : les cartes, leur créneau
 * libre, l'état du lieu. Sorti du contrôleur le 2026-10-01 pour que la page et
 * sa proposition de refonte lisent le MÊME calcul.
 */
final class PlaceCatalogue
{
    public function __construct(
        private readonly PlaceRepository $places,
        private readonly NextFreeSlotService $nextFreeSlot,
        private readonly ScheduleResolver $schedule,
        private readonly SiteSettingService $siteSettings,
        private readonly UsageRightsService $usageRights,
        private readonly VenueContext $venues,
    ) {
    }

    /** @return array<string, mixed> les variables du gabarit */
    public function build(Request $request, ?Utilisateur $user): array
    {
        $usageVerdict = $this->usageRights->verdict($user, 'places');
        $search = trim((string) $request->query->get('q', ''));

        $now = new \DateTimeImmutable('now', new \DateTimeZone($this->siteSettings->getTimezone()));

        // ⚠️ S138. The PUBLIC catalogue had no location filter, on an install with
        // more than one location since S129 — a member was shown every row in the
        // organisation with no way to narrow it. Same gap /machines had until S137.
        $venueContext = $this->venues->forRequest($request, $user);

        // ⚠️ When the catalogue is filtered to one location, "closed" is that
        // location's fact; aggregated across all of them there is no single
        // answer, so it keeps the default venue's.
        // 🔴 `isOpenAt()` rather than a comparison against the envelope (S134d):
        // at 12:30 in a lab that shuts for lunch, the envelope says open and the
        // door is locked.
        // 🔴 **And it must come AFTER `$venueContext` exists.** S134d put this
        // block above the assignment on this page, so `$venueContext['selected']`
        // was an undefined variable and the location filter was ignored — the
        // page silently answered for the DEFAULT venue. Prod runs without
        // `strict_variables`, so nothing said a word; it took a warning in a
        // self-test to surface it.
        $venueOpenNow = $this->schedule->isOpenAt($venueContext['selected']?->getId(), $now);
        // 🔴 **The reason, not just the fact** (S134e). "Closed" leaves a member
        // wondering whether the lab is shut, broken, or whether they misread the
        // page; "closed — public holiday" ends the question.
        $venueClosureReason = $venueOpenNow ? null : $this->schedule->closureReasonFor($venueContext['selected']?->getId(), $now);
        // ⚠️ S147, J-2 — le catalogue PROPOSE, donc les espaces archivés en sortent.
        $rows = $this->places->findLive(
            $venueContext['selected'] === null ? [] : ['venue' => $venueContext['selected']],
            ['nom' => 'ASC'],
        );
        $cards = [];
        foreach ($rows as $place) {
            if ($search !== '' && stripos($place->getNom(), $search) === false) {
                continue;
            }
            $slot = $this->nextFreeSlot->find($usageVerdict->allowed ? $user : null, ReservableType::Place, (int) $place->getId());
            $cards[] = [
                'place' => $place,
                'slot' => $slot,
                'freeNow' => $venueOpenNow && $slot !== null && $slot['start'] <= $now->modify('+60 minutes'),
                'usageRight' => $usageVerdict,
            ];
        }

        return [
            'venueContext' => $venueContext,
            'cards' => $cards,
            'search' => $search,
            'venueOpenNow' => $venueOpenNow,
            'venueClosureReason' => $venueClosureReason,
            'totalCount' => \count($cards),
            'allCount' => \count($rows),
            'freeCount' => \count(array_filter($cards, static fn (array $c): bool => $c['freeNow'])),
        ];
    }
}
