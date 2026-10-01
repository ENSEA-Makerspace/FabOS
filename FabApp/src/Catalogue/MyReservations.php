<?php

declare(strict_types=1);

namespace App\Catalogue;

use App\Entity\Utilisateur;
use App\Repository\ReservationRepository;
use App\Reservation\Verb\BookingVerb;
use App\Reservation\Verb\BookingVerbService;
use App\Reservation\LabClock;
use App\Reservation\ReservableResolver;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * Ce que montre « Mes réservations » : les quatre groupes, les verbes de chaque
 * réservation, l'offre d'annulation. Sorti du contrôleur le 2026-10-01 pour que
 * la page et sa proposition de refonte lisent le MÊME calcul.
 */
final class MyReservations
{
    public function __construct(
        private readonly ReservationRepository $reservations,
        private readonly ReservableResolver $reservables,
        private readonly TranslatorInterface $translator,
        private readonly LabClock $clock,
        private readonly BookingVerbService $bookingVerbs,
    ) {
    }

    /** @return array<string, mixed> les variables du gabarit */
    public function build(Request $request, Utilisateur $user): array
    {
        $items = $this->reservations->findForUser($user, ['dateDebut' => 'DESC']);
        $this->reservables->warm($items);

        // ⚠️ `$now` is a real instant, and every stored booking date is put on the
        // same footing with `instantOf()` before being compared to it. The old
        // shape — a lab-zoned `now` against a raw hydrated column — was out by the
        // lab's UTC offset, always in the permissive direction: a finished
        // booking sat in "À venir" for two more hours and stayed cancellable
        // after it had started. See LabClock for why the digits look right anyway.
        $now = $this->clock->now();
        $current = [];
        $upcoming = [];
        $past = [];
        $cancelled = [];
        $nextReservation = null;

        foreach ($items as $reservation) {
            // Declined requests group with cancellations — both are bookings the
            // user no longer has, and neither holds its slot any more.
            if (!$reservation->isActive()) {
                $cancelled[] = $reservation;
                continue;
            }

            $start = $this->clock->instantOf($reservation->getDateDebut());
            $end = $this->clock->instantOf($reservation->getDateFin());

            if ($end < $now) {
                $past[] = $reservation;
                continue;
            }

            if ($start <= $now && $end >= $now) {
                $current[] = $reservation;
                continue;
            }

            $upcoming[] = $reservation;
            if ($nextReservation === null || $reservation->getDateDebut() < $nextReservation->getDateDebut()) {
                $nextReservation = $reservation;
            }
        }

        usort($current, static fn ($a, $b): int => $a->getDateDebut() <=> $b->getDateDebut());
        usort($upcoming, static fn ($a, $b): int => $a->getDateDebut() <=> $b->getDateDebut());
        usort($past, static fn ($a, $b): int => $b->getDateDebut() <=> $a->getDateDebut());
        usort($cancelled, static fn ($a, $b): int => $b->getDateDebut() <=> $a->getDateDebut());


        $groups = [
            'current' => $current,
            'upcoming' => $upcoming,
            'past' => $past,
            'cancelled' => $cancelled,
        ];

        // Tiles are the four states, and their counts are computed over the whole
        // set — picking one state must not blank the others, which is the shell's
        // stated expectation.
        //
        // ⚠️ Translated HERE. The shell prints `tile.label` raw, because every other
        // catalogue hands it a finished word; passing a message key instead puts
        // "resv.f_current" on the screen, which is exactly what happened.
        $labels = [
            'current' => $this->translator->trans('resv.f_current'),
            'upcoming' => $this->translator->trans('resv.f_upcoming'),
            'past' => $this->translator->trans('resv.f_past'),
            'cancelled' => $this->translator->trans('resv.f_cancelled'),
        ];

        $state = (string) $request->query->get('etat', '');
        $search = trim((string) $request->query->get('q', ''));

        $visible = array_key_exists($state, $groups)
            ? [$state => $groups[$state]]
            : $groups;

        if ($search !== '') {
            $needle = mb_strtolower($search);
            foreach ($visible as $key => $rows) {
                $visible[$key] = array_values(array_filter($rows, function ($r) use ($needle): bool {
                    $name = (string) ($this->reservables->resolve($r)->name ?? '');

                    return $name !== '' && str_contains(mb_strtolower($name), $needle);
                }));
            }
        }

        // S77's verbs, resolved once per visible card by the same service the
        // endpoints ask. ⚠️ The template must never re-derive one of these: the
        // page hiding a control the endpoint would have honoured (or drawing one
        // it refuses) is precisely the drift this replaces. Cheap today — no
        // lock window is configured, so none of it queries.
        $verbs = [];
        foreach ($visible as $rows) {
            foreach ($rows as $reservation) {
                $verbs[$reservation->getId()] = $this->bookingVerbs->verdicts($reservation, $user, $now);
            }
        }

        // The undo offer, arriving as `?undo=<id>` from the cancel redirect. It is
        // re-checked here rather than trusted: the parameter is in the member's
        // own URL bar, so it has to prove itself like any other input, and the
        // slot may well have been taken in the seconds since.
        $undo = null;
        $undoId = (int) $request->query->get('undo', 0);
        if ($undoId > 0) {
            $candidate = $this->reservations->find($undoId);

            // ⚠️ Ownership is checked here and not left to the verb alone. The
            // verb lets an admin act on anyone's booking, which is right for the
            // endpoint and wrong for this bar: the id comes from the URL, and an
            // admin pasting `?undo=1234` would be shown a stranger's resource
            // label on their own page. That is the class of leak S38 spent a
            // session closing. This page only ever offers you your own bookings.
            $mine = $candidate?->getUtilisateur()?->getId() === $user->getId();

            if ($candidate !== null && $mine && $this->bookingVerbs->verdict(BookingVerb::Restore, $candidate, $user, $now)->allowed) {
                $undo = $candidate;
            }
        }

        return [
            'reservations' => $items,
            'groupsInOrder' => $visible,
            'groupLabels' => $labels,
            'verbs' => $verbs,
            'undo' => $undo,
            'tiles' => array_map(
                static fn (string $key): array => ['slug' => $key, 'label' => $labels[$key], 'total' => count($groups[$key])],
                array_keys($groups),
            ),
            'activeState' => array_key_exists($state, $groups) ? $state : '',
            'search' => $search,
            'totalShown' => array_sum(array_map('count', $visible)),
            'totalAll' => array_sum(array_map('count', $groups)),
            'nextReservation' => $nextReservation,
            'now' => $now,
        ];
    }
}
