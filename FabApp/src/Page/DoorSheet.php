<?php

declare(strict_types=1);

namespace App\Page;

use App\Entity\AccessPoint;
use App\Entity\RfidReader;
use App\Repository\AccessPointRepository;
use App\Repository\AccessRfidLogRepository;
use App\Repository\RfidReaderRepository;
use App\Rfid\DoorAccessDecision;
use App\Rfid\ReaderHealth;
use App\Schedule\ScheduleResolver;
use App\Service\PlaceBadges;
use Symfony\Contracts\Translation\TranslatorInterface;

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

    /** Statut du journal → clé de traduction. */
    private const STATUS = [
        'authorized' => 'door_sheet.status_authorized',
        'booking_in_progress' => 'door_sheet.status_booking_in_progress',
        'badge_open_hours' => 'door_sheet.status_badge_open_hours',
        'no_booking_now' => 'door_sheet.status_no_booking_now',
        'missing_badge' => 'door_sheet.status_missing_badge',
        'no_place_bound' => 'door_sheet.status_no_place_bound',
        'access_point_archived' => 'door_sheet.status_access_point_archived',
        'account_inactive' => 'door_sheet.status_account_inactive',
        'unknown_rfid' => 'door_sheet.status_unknown_rfid',
        'reader_inactive' => 'door_sheet.status_reader_inactive',
    ];

    /** État du lecteur → clé de traduction (en minuscules dans la phrase). */
    private const HEALTH = [
        ReaderHealth::READY => 'door_sheet.health_ready',
        ReaderHealth::OFFLINE => 'door_sheet.health_offline',
        ReaderHealth::NEVER_SEEN => 'door_sheet.health_never_seen',
        ReaderHealth::UNPAIRED => 'door_sheet.health_unpaired',
        ReaderHealth::INVALID_PAIRING => 'door_sheet.health_invalid_pairing',
        ReaderHealth::DISABLED => 'door_sheet.health_disabled',
        ReaderHealth::ARCHIVED => 'door_sheet.health_archived',
    ];

    public function __construct(
        private readonly AccessPointRepository $points,
        private readonly RfidReaderRepository $readers,
        private readonly ReaderHealth $health,
        private readonly PlaceBadges $placeBadges,
        private readonly ScheduleResolver $schedule,
        private readonly AccessRfidLogRepository $logs,
        private readonly TranslatorInterface $translator,
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
        // La fiche d'une porte archivée reste lisible (la liste l'affiche).
        foreach ([...$live, ...$this->points->findForAdmin()] as $candidate) {
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
                    'who' => $log->getUtilisateur()?->getDisplayName() ?? $this->translator->trans('door_sheet.unknown_badge'),
                    'outcome' => $this->translator->trans($log->isAuthorized() ? 'door_sheet.outcome_ok' : 'door_sheet.outcome_refused'),
                    'authorized' => $log->isAuthorized(),
                    'detail' => isset(self::STATUS[$log->getStatus()]) ? $this->translator->trans(self::STATUS[$log->getStatus()]) : strtolower(str_replace('_', ' ', $log->getStatus())),
                    'at' => $log->getCreatedAt(),
                ];
            }
        }
        usort($events, static fn (array $a, array $b): int => $b['at'] <=> $a['at']);

        return [
            'point' => $point,
            'inService' => $eval['inService'],
            'headline' => $this->translator->trans($eval['inService'] ? 'door_sheet.headline_live' : 'door_sheet.headline_setup'),
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
                $readers[] = ['reader' => $reader, 'health' => $health, 'stateLabel' => isset(self::HEALTH[$health['state']]) ? $this->translator->trans(self::HEALTH[$health['state']]) : $health['state']];
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

        $t = fn (string $key, array $params = []): string => $this->translator->trans($key, $params);
        $steps = [
            [
                'key' => 'place', 'label' => $t('door_sheet.step_place'), 'done' => $place !== null, 'blocking' => true, 'optional' => false,
                'note' => $place !== null ? $place->getNom() : $t('door_sheet.step_place_missing'),
            ],
            [
                'key' => 'reader', 'label' => $t('door_sheet.step_reader'), 'blocking' => true, 'optional' => false,
                'done' => $best !== null && $best['health']['signal'] === 'go',
                'note' => $best === null
                    ? $t('door_sheet.step_reader_missing')
                    : $t('door_sheet.step_reader_state', ['%reader%' => $best['reader']->getName(), '%state%' => $best['stateLabel']]),
            ],
            [
                'key' => 'badges', 'label' => $t('door_sheet.step_badges'), 'blocking' => false, 'optional' => true,
                'done' => $badges !== [],
                'note' => $badges !== []
                    ? $t('door_sheet.step_badges_set', ['%badges%' => implode(', ', $badges)])
                    : $t('door_sheet.step_badges_none'),
            ],
            [
                'key' => 'hours', 'label' => $t('door_sheet.step_hours'), 'blocking' => false, 'optional' => $badges === [],
                'done' => $hoursWeek,
                'note' => $badges === []
                    ? $t('door_sheet.step_hours_irrelevant')
                    : ($hoursWeek ? $t('door_sheet.step_hours_ok') : $t('door_sheet.step_hours_none')),
            ],
            [
                'key' => 'first', 'label' => $t('door_sheet.step_first'), 'blocking' => false, 'optional' => false,
                'done' => $passed,
                'note' => $passed ? $t('door_sheet.step_first_done') : ($seen ? $t('door_sheet.step_first_seen') : $t('door_sheet.step_first_wait')),
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
            return $this->translator->trans('door_sheet.rule_no_place');
        }
        $window = DoorAccessDecision::MARGIN_MINUTES;
        $booking = $this->translator->trans('door_sheet.rule_booking', ['%place%' => $place->getNom(), '%n%' => $window]);
        if ($badges === []) {
            return $this->translator->trans('door_sheet.rule_booking_only', ['%booking%' => $booking]);
        }

        return $this->translator->trans('door_sheet.rule_badge', [
            '%badges%' => implode($this->translator->trans('door_sheet.or'), array_map(static fn (string $b): string => '« ' . $b . ' »', $badges)),
            '%booking%' => $booking,
        ]);
    }
}
