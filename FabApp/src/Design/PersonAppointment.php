<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use App\Reservation\PersonAvailabilityService;

/**
 * « Rendez-vous avec une personne » (proposition du 2026-10-01, planche
 * `02-rendez-vous-personne.png`). Lecture seule : les créneaux viennent de
 * `PersonAvailabilityService::dailySlots()` — le MÊME appel que
 * `PersonBookingController::booking()` ; la règle (disponibilités × ouverture
 * du lieu × réservations × passé) n'est pas réécrite ici, seulement REGROUPÉE
 * en bande de jours + créneaux du jour choisi.
 */
final class PersonAppointment
{
    /** Jours montrés dans la bande. */
    private const STRIP_DAYS = 14;

    public function __construct(
        private readonly UtilisateurRepository $people,
        private readonly PersonAvailabilityService $availability,
    ) {
    }

    /**
     * @return array{person: ?Utilisateur, bookable: list<Utilisateur>, duration: int, durations: list<int>, strip: list<array<string, mixed>>, day: ?array<string, mixed>, slot: ?array<string, mixed>, next: ?array<string, mixed>}
     */
    public function build(?int $personId, ?string $jour, ?string $creneau, ?int $duree): array
    {
        $bookable = $this->people->findBy(['bookable' => true], ['id' => 'ASC']);
        $person = null;
        foreach ($bookable as $candidate) {
            if ($personId !== null && $candidate->getId() === $personId) {
                $person = $candidate;
                break;
            }
        }
        // Défaut : la première personne qui accepte des rendez-vous ET a un créneau.
        if ($person === null && $personId === null) {
            foreach ($bookable as $candidate) {
                $durations = $candidate->getBookingDurationsMinutes();
                if ($this->availability->dailySlots($candidate, $durations[0]) !== []) {
                    $person = $candidate;
                    break;
                }
            }
            $person ??= $bookable[0] ?? null;
        }
        if ($person === null) {
            return ['person' => null, 'bookable' => $bookable, 'duration' => 0, 'durations' => [], 'strip' => [], 'day' => null, 'slot' => null, 'next' => null];
        }

        $durations = $person->getBookingDurationsMinutes();
        $duration = $duree !== null && \in_array($duree, $durations, true) ? $duree : $durations[0];
        $days = $this->availability->dailySlots($person, $duration);

        $byDate = [];
        foreach ($days as $d) {
            $byDate[$d['date']->format('Y-m-d')] = $d;
        }
        $first = $days[0] ?? null;
        $next = $first === null ? null : ['date' => $first['date'], 'slot' => $first['slots'][0]];

        // Bande : 14 jours calendaires à partir d'aujourd'hui (fuseau du labo, repris des créneaux).
        $today = ($first !== null ? $first['date'] : new \DateTimeImmutable('today'))->setTime(0, 0);
        $today = $first !== null ? (new \DateTimeImmutable('today', $first['date']->getTimezone())) : $today;
        $strip = [];
        for ($i = 0; $i < self::STRIP_DAYS; ++$i) {
            $date = $today->modify(sprintf('+%d days', $i));
            $key = $date->format('Y-m-d');
            $strip[] = ['date' => $date, 'key' => $key, 'dow' => $date->format('N'), 'free' => isset($byDate[$key]) ? \count($byDate[$key]['slots']) : 0];
        }

        $dayKey = $jour !== null && isset($byDate[$jour]) ? $jour : ($first !== null ? $first['date']->format('Y-m-d') : null);
        $day = $dayKey === null ? null : $byDate[$dayKey];

        $slot = null;
        if ($day !== null && $creneau !== null) {
            foreach ($day['slots'] as $candidate) {
                if ($candidate['start']->format('H:i') === $creneau) {
                    $slot = $candidate;
                    break;
                }
            }
        }

        return [
            'person' => $person,
            'bookable' => $bookable,
            'duration' => $duration,
            'durations' => $durations,
            'strip' => $strip,
            'day' => $day,
            'slot' => $slot,
            'next' => $next,
        ];
    }
}
