<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\AccessPoint;
use App\Entity\RfidReader;
use App\Repository\AccessPointRepository;
use App\Repository\AccessRfidLogRepository;
use App\Repository\RfidReaderRepository;
use App\Rfid\DoorAccessDecision;
use App\Rfid\ReaderHealth;
use App\Schedule\ScheduleResolver;
use App\Service\PlaceBadges;

/**
 * La fiche d'UN point d'accès, avec sa mise en service (proposition
 * `point-acces`, 2026-10-01, d'après `espaces/05-mise-en-service-point-acces.png`).
 *
 * ⚠️ LECTURE SEULE et rien de neuf : la pièce vient du point, le lecteur de
 * `RfidReaderRepository` + `ReaderHealth`, les badges exigés de `PlaceBadges`,
 * les horaires de `ScheduleResolver`, les passages du journal RFID. La règle
 * dite en clair est celle de `DoorAccessDecision` (réservation ±15 min, ou badge
 * seul pendant les heures). Aucun secret ni jeton n'est exposé.
 *
 * Étape : `key`, `label`, `note`, `done`, `blocking`, `optional`. Une étape
 * `optional` ne compte pas pour dire « en service » (ex. les horaires quand la
 * pièce n'exige aucun badge : ils ne changent rien à la porte).
 */
final class DoorSheet
{
    private const EVENTS = 5;

    private const STATUS = [
        'authorized' => 'Autorisé',
        'booking_in_progress' => 'Réservation en cours',
        'badge_open_hours' => 'Badge seul, heures d’ouverture',
        'no_booking_now' => 'Pas de réservation à cette heure',
        'missing_badge' => 'Badge manquant',
        'no_place_bound' => 'Aucune pièce reliée',
        'access_point_archived' => 'Porte archivée',
        'account_inactive' => 'Compte désactivé',
        'unknown_rfid' => 'Badge inconnu',
        'reader_inactive' => 'Lecteur désactivé',
    ];

    private const HEALTH = [
        ReaderHealth::READY => 'en ligne',
        ReaderHealth::OFFLINE => 'hors ligne',
        ReaderHealth::NEVER_SEEN => 'jamais connecté',
        ReaderHealth::UNPAIRED => 'non associé',
        ReaderHealth::INVALID_PAIRING => 'association invalide',
        ReaderHealth::DISABLED => 'désactivé',
        ReaderHealth::ARCHIVED => 'archivé',
    ];

    public function __construct(
        private readonly AccessPointRepository $points,
        private readonly RfidReaderRepository $readers,
        private readonly ReaderHealth $health,
        private readonly PlaceBadges $placeBadges,
        private readonly ScheduleResolver $schedule,
        private readonly AccessRfidLogRepository $logs,
    ) {
    }

    /**
     * @return array{
     *   point: ?AccessPoint, inService: bool, headline: string, signal: string,
     *   others: list<array{point: AccessPoint, inService: bool, signal: string, current: bool}>,
     *   steps: list<array<string, mixed>>, blocking: ?array<string, mixed>,
     *   rule: string, hoursToday: ?string, badges: list<string>,
     *   readers: list<array{reader: RfidReader, health: array{state: string, signal: string, label: string}, stateLabel: string}>,
     *   events: list<array{who: string, outcome: string, authorized: bool, detail: string, at: \DateTimeImmutable}>
     * }
     */
    public function build(?int $id): array
    {
        $live = $this->points->findLive();
        $point = null;
        foreach ($live as $candidate) {
            if ($id !== null && $candidate->getId() === $id) {
                $point = $candidate;
            }
        }
        $point ??= $live[0] ?? null;

        $others = [];
        foreach ($live as $candidate) {
            $eval = $this->evaluate($candidate);
            $others[] = ['point' => $candidate, 'inService' => $eval['inService'], 'signal' => $eval['signal'], 'current' => $candidate === $point];
        }
        if ($point === null) {
            return ['point' => null, 'inService' => false, 'headline' => '', 'signal' => 'muted', 'others' => [], 'steps' => [], 'blocking' => null, 'rule' => '', 'hoursToday' => null, 'badges' => [], 'readers' => [], 'events' => []];
        }

        $eval = $this->evaluate($point);

        $events = [];
        foreach ($eval['readers'] as $row) {
            foreach ($this->logs->search(0, $row['reader']->getId(), null, null, self::EVENTS) as $log) {
                $events[] = [
                    'who' => $log->getUtilisateur()?->getDisplayName() ?? 'Badge inconnu',
                    'outcome' => $log->isAuthorized() ? 'Porte ouverte' : 'Refusé',
                    'authorized' => $log->isAuthorized(),
                    'detail' => self::STATUS[$log->getStatus()] ?? strtolower(str_replace('_', ' ', $log->getStatus())),
                    'at' => $log->getCreatedAt(),
                ];
            }
        }
        usort($events, static fn (array $a, array $b): int => $b['at'] <=> $a['at']);

        return [
            'point' => $point,
            'inService' => $eval['inService'],
            'headline' => $eval['inService'] ? 'En service' : 'À mettre en service',
            'signal' => $eval['signal'],
            'others' => $others,
            'steps' => $eval['steps'],
            'blocking' => $eval['blocking'],
            'rule' => $this->rule($point, $eval['badges']),
            'hoursToday' => $eval['hoursToday'],
            'badges' => $eval['badges'],
            'readers' => $eval['readers'],
            'events' => \array_slice($events, 0, self::EVENTS),
        ];
    }

    /** @return array<string, mixed> */
    private function evaluate(AccessPoint $point): array
    {
        $place = $point->getPlace();
        $readers = [];
        foreach ($this->readers->findForAdmin() as $reader) {
            if (!$reader->isArchived() && $reader->getAccessPoint()?->getId() === $point->getId()) {
                $health = $this->health->of($reader);
                $readers[] = ['reader' => $reader, 'health' => $health, 'stateLabel' => self::HEALTH[$health['state']] ?? $health['state']];
            }
        }
        $badges = $place === null ? [] : $this->placeBadges->requiredNames($place);

        // L'état du lecteur : le meilleur des lecteurs reliés (une porte peut en avoir deux).
        $best = null;
        foreach ($readers as $row) {
            if ($best === null || ($row['health']['signal'] === 'go' && $best['health']['signal'] !== 'go')) {
                $best = $row;
            }
        }
        $seen = false;
        foreach ($readers as $row) {
            $seen = $seen || $row['reader']->getLastSeenAt() !== null;
        }
        $passed = false;
        foreach ($readers as $row) {
            $passed = $passed || $this->logs->search(0, $row['reader']->getId(), null, null, 1) !== [];
        }

        $hoursToday = null;
        $hoursWeek = false;
        if ($place !== null) {
            $venueId = $place->getVenue()?->getId();
            $today = new \DateTimeImmutable('today');
            for ($i = 0; $i < 7; ++$i) {
                $day = $today->modify('+' . $i . ' days');
                $span = $this->schedule->openMinutesFor($venueId, $day, 'place', (int) $place->getId());
                if ($span !== null) {
                    $hoursWeek = true;
                    if ($i === 0) {
                        $hoursToday = sprintf('%02d:%02d–%02d:%02d', intdiv($span['start'], 60), $span['start'] % 60, intdiv($span['end'], 60), $span['end'] % 60);
                    }
                }
            }
        }

        $steps = [
            [
                'key' => 'place', 'label' => 'Pièce reliée', 'done' => $place !== null, 'blocking' => true, 'optional' => false,
                'note' => $place !== null ? $place->getNom() : 'Sans pièce, la porte ne sait pas pour quelle réservation s’ouvrir : elle refuse tout le monde.',
            ],
            [
                'key' => 'reader', 'label' => 'Lecteur relié et en ligne', 'blocking' => true, 'optional' => false,
                'done' => $best !== null && $best['health']['signal'] === 'go',
                'note' => $best === null
                    ? 'Aucun lecteur ne commande cette porte : le membre badge devant le mur et rien ne se passe.'
                    : $best['reader']->getName() . ' : ' . $best['stateLabel'] . '.',
            ],
            [
                'key' => 'badges', 'label' => 'Badge exigé par la pièce', 'blocking' => false, 'optional' => true,
                'done' => $badges !== [],
                'note' => $badges !== []
                    ? implode(', ', $badges) . ' : ce badge ouvre la porte aux heures d’ouverture, sans réservation.'
                    : 'Facultatif. Sans badge exigé, la porte s’ouvre seulement sur réservation.',
            ],
            [
                'key' => 'hours', 'label' => 'Horaires de la pièce', 'blocking' => false, 'optional' => $badges === [],
                'done' => $hoursWeek,
                'note' => $badges === []
                    ? 'Sans badge exigé, les horaires ne changent rien à la porte.'
                    : ($hoursWeek ? 'Le badge seul ouvre pendant ces heures.' : 'Aucune heure d’ouverture cette semaine : le badge seul n’ouvrirait jamais, seule une réservation le ferait.'),
            ],
            [
                'key' => 'first', 'label' => 'Premier passage enregistré', 'blocking' => false, 'optional' => false,
                'done' => $passed,
                'note' => $passed ? 'Le journal RFID a déjà vu un passage sur ce lecteur.' : ($seen ? 'Le lecteur répond ; il attend un premier badge.' : 'En attente d’un premier badge sur le lecteur.'),
            ],
        ];

        $blocking = null;
        foreach ($steps as $step) {
            if (!$step['done'] && !$step['optional']) {
                $blocking = $step;
                break;
            }
        }
        $inService = $blocking === null;

        return [
            'inService' => $inService,
            'signal' => $inService ? 'go' : ($blocking['blocking'] ? 'stop' : 'caution'),
            'steps' => $steps,
            'blocking' => $blocking,
            'badges' => $badges,
            'readers' => $readers,
            'hoursToday' => $hoursToday,
        ];
    }

    /** @param list<string> $badges */
    private function rule(AccessPoint $point, array $badges): string
    {
        $place = $point->getPlace();
        if ($place === null) {
            return 'Aucune pièce n’est reliée : cette porte n’ouvre pour personne.';
        }
        $window = DoorAccessDecision::MARGIN_MINUTES;
        $booking = sprintf('sur réservation de « %s », de %d min avant à %d min après le créneau', $place->getNom(), $window, $window);
        if ($badges === []) {
            return 'Cette porte s’ouvre ' . $booking . '. Aucun badge n’est exigé par la pièce : sans réservation, elle reste fermée.';
        }

        return sprintf(
            'Cette porte s’ouvre au badge seul pendant les heures d’ouverture, pour qui détient %s ; en dehors des heures, ou sans ce badge, elle s’ouvre %s.',
            implode(' ou ', array_map(static fn (string $b): string => '« ' . $b . ' »', $badges)),
            $booking,
        );
    }
}
