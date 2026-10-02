<?php

declare(strict_types=1);

namespace App\Page;

use App\Entity\Event;
use App\Entity\EventRegistration;
use App\Entity\Utilisateur;
use App\Event\EventArtwork;
use App\Event\TicketLinker;
use App\Repository\EventCategoryRepository;
use App\Repository\EventRegistrationRepository;
use App\Repository\EventRepository;
use App\UsageRights\UsageRightsService;
use App\Venue\VenueContext;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * « Événements » (proposition du 2026-10-01, planche `01-evenements.png`) : les
 * MÊMES données que `SiteController::events()` (mêmes dépôts, mêmes filtres, mêmes
 * cartes), plus « Mes inscriptions » pour un membre connecté. Lecture seule.
 *
 * ⚠️ Le calcul des cartes est reproduit ici et non extrait : la page actuelle
 * n'est pas touchée tant que la proposition n'est pas retenue.
 */
final class EventsHub
{
    public function __construct(
        private readonly EventRepository $events,
        private readonly EventRegistrationRepository $registrations,
        private readonly EventArtwork $artwork,
        private readonly UsageRightsService $usageRights,
        private readonly VenueContext $venues,
        private readonly EventCategoryRepository $eventCategories,
        private readonly TicketLinker $tickets,
        private readonly TranslatorInterface $translator,
    ) {
    }

    /** @return array<string, mixed> */
    public function build(Request $request, ?Utilisateur $member): array
    {
        $search = trim((string) $request->query->get('q', ''));
        $when = (string) $request->query->get('when', 'all');
        if (!in_array($when, [EventRepository::WHEN_UPCOMING, EventRepository::WHEN_PAST, 'all'], true)) {
            $when = EventRepository::WHEN_UPCOMING;
        }

        $venueContext = $this->venues->forRequest($request, $member);
        $rows = $this->events->findForCatalogue($when === 'all' ? null : $when, $search);
        if ($venueContext['selected'] !== null) {
            $rows = array_values(array_filter(
                $rows,
                static fn (Event $event): bool => $event->getVenue()?->getId() === $venueContext['selected']->getId(),
            ));
        }
        $categorySlug = trim((string) $request->query->get('category', ''));
        $selectedCategory = $categorySlug !== '' ? $this->eventCategories->findOneBySlug($categorySlug) : null;
        if ($selectedCategory !== null) {
            $rows = array_values(array_filter(
                $rows,
                static fn (Event $event): bool => $event->getCategory()?->getId() === $selectedCategory->getId(),
            ));
        }
        $categoryOptions = [['value' => '', 'label' => $this->translator->trans('event_categories.menu_all')]];
        foreach ($this->eventCategories->findSelectable() as $category) {
            $categoryOptions[] = ['value' => $category->getSlug(), 'label' => $category->getLabel()];
        }

        $now = $this->events->storedNow();
        $seatsTaken = $this->registrations->countSeatsTakenFor($rows);
        $cards = [];
        foreach ($rows as $event) {
            $cards[] = $this->card($event, $member, $seatsTaken[(int) $event->getId()] ?? 0, $now);
        }

        return [
            'venueContext' => $venueContext,
            'categoryOptions' => $categoryOptions,
            'category' => $selectedCategory?->getSlug() ?? '',
            'cards' => $cards,
            'search' => $search,
            'when' => $when,
            'total' => count($cards),
            'all' => $this->events->countWhen(null),
            'countUpcoming' => $this->events->countWhen(EventRepository::WHEN_UPCOMING),
            'countPast' => $this->events->countWhen(EventRepository::WHEN_PAST),
            'mine' => $member === null ? null : $this->mine($member, $now),
        ];
    }

    /**
     * Inscriptions (places tenues ou liste d'attente) à des événements non
     * terminés ni annulés : la prochaine en grand, les autres en lignes.
     *
     * @return array{next: ?array<string, mixed>, others: list<array<string, mixed>>}
     */
    private function mine(Utilisateur $member, \DateTimeImmutable $now): array
    {
        $rows = [];
        foreach ($this->registrations->findForUser($member) as $registration) {
            $event = $registration->getEvent();
            $end = $event?->getDateFin() ?? $event?->getDateDebut();
            if ($event === null || $event->isCancelled() || $end === null || $end < $now) {
                continue;
            }
            $rows[] = $registration;
        }
        $events = array_map(static fn (EventRegistration $r): Event => $r->getEvent(), $rows);
        $seatsTaken = $this->registrations->countSeatsTakenFor($events);

        $items = array_map(fn (EventRegistration $r): array => [
            'registration' => $r,
            'card' => $this->card($r->getEvent(), $r->getUtilisateur(), $seatsTaken[(int) $r->getEvent()->getId()] ?? 0, $now),
            // Le billet n'existe que pour une place tenue (lien signé ; null sans URL publique).
            'ticketUrl' => $r->holdsSeat() ? $this->tickets->ticketUrl($r) : null,
        ], $rows);

        return ['next' => array_shift($items), 'others' => $items];
    }

    /** @return array<string, mixed> */
    private function card(Event $event, ?Utilisateur $member, int $taken, \DateTimeImmutable $now): array
    {
        $capacity = $event->getCapacite();

        return [
            'event' => $event,
            'photo' => $this->artwork->describe($event)['thumb'],
            'past' => !$event->isCancelled() && $event->getDateDebut() !== null && $event->getDateDebut() <= $now,
            'seatsTaken' => $taken,
            'full' => $capacity !== null && $capacity > 0 && $taken >= $capacity,
            'seatsLeft' => $capacity !== null ? max(0, $capacity - $taken) : null,
            'usageRight' => $member instanceof Utilisateur && !$event->isGuestsAllowed()
                ? $this->usageRights->verdict($member, 'events', $event->getDateDebut(), $event->getDateFin())
                : null,
        ];
    }
}
