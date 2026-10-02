<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\AccessRfidLog;
use App\Repository\AccessRfidLogRepository;
use App\Repository\MachineRepository;
use App\Repository\RfidReaderRepository;
use App\Rfid\AccessIncident;
use App\Rfid\ReaderHealth;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * Données de la proposition `incidents-acces` (2026-10-01, d'après les planches
 * `espaces/06-incidents-acces.png` et `equipement/equipment-access-incidents.png`).
 *
 * ⚠️ LECTURE SEULE et rien de neuf : les lignes sortent de
 * `AccessRfidLogRepository::search()` (la même requête que
 * `/admin/access-rfid-logs`), le verbe correctif de `AccessIncident::of()`, et
 * l'état des lecteurs de `ReaderHealth::of()`. Seul le REGROUPEMENT par cause est
 * propre à la proposition : il range les statuts réels du journal (les deux
 * générations de vocabulaire) en six causes lisibles.
 *
 * ⚠️ Il n'existe PAS de statut « hors plage horaire » dans le journal : la
 * planche en montre un, la donnée n'en écrit pas. Pas de tuile inventée.
 */
final class AccessIncidentBoard
{
    /** @var array<string, array{label: string, statuses: list<string>}> `label` est une clé `access_incidents.*` */
    private const CAUSES = [
        'badge' => ['label' => 'cause_badge', 'statuses' => ['missing_badge', 'REQUIRED_BADGE_MISSING']],
        'formation' => ['label' => 'cause_formation', 'statuses' => ['NO_TRAINING', 'TRAINING_REQUIRED']],
        'inconnu' => ['label' => 'cause_unknown', 'statuses' => ['unknown_rfid']],
        'lecteur' => ['label' => 'cause_device', 'statuses' => ['reader_inactive', 'unknown_machine', 'unauthorized_device', 'invalid_payload', 'device_api_not_configured']],
        'compte' => ['label' => 'cause_account', 'statuses' => ['account_inactive']],
        'serveur' => ['label' => 'cause_server', 'statuses' => ['server_error']],
    ];

    /** Lignes montrées ; le total réel est rendu à côté. */
    private const LIMIT = 50;

    public function __construct(
        private readonly AccessRfidLogRepository $logs,
        private readonly AccessIncident $incidents,
        private readonly RfidReaderRepository $readers,
        private readonly ReaderHealth $readerHealth,
        private readonly MachineRepository $machines,
        private readonly TranslatorInterface $translator,
    ) {
    }

    /**
     * @param string $cause `todo` (défaut : tous les refus), `all` (journal entier)
     *                      ou une clé de cause ; toute autre valeur retombe sur `todo`.
     *
     * @return array{
     *     cause: string, days: int, readerId: ?int, machineId: ?int, refined: bool,
     *     tiles: list<array{key: string, label: string, count: int}>,
     *     rows: list<array{log: AccessRfidLog, fix: ?array<string, mixed>, cause: string}>,
     *     shown: int, matching: int,
     *     health: array{ready: int, offline: int, other: int, attention: list<array<string, mixed>>},
     *     readerOptions: list<array{id: int, name: string}>, machineOptions: list<array{id: int, name: string}>
     * }
     */
    public function build(int $days = 7, ?int $readerId = null, ?int $machineId = null, string $cause = 'todo'): array
    {
        $days = in_array($days, [1, 7, 30, 0], true) ? $days : 7;
        $cause = ($cause === 'all' || isset(self::CAUSES[$cause])) ? $cause : 'todo';

        // Une seule passe : les refus de la période, classés par cause ; les
        // compteurs des tuiles sortent de cette même liste (500 au plus).
        $refused = $this->logs->search($days, $readerId, $machineId, 'no', 500);
        $counts = array_fill_keys(array_keys(self::CAUSES), 0);
        $counts['autre'] = 0;
        foreach ($refused as $log) {
            ++$counts[$this->causeOf($log)];
        }

        $tiles = [['key' => 'todo', 'label' => $this->translator->trans('access_incidents.tile_todo'), 'count' => count($refused)]];
        foreach (self::CAUSES as $key => $def) {
            $tiles[] = ['key' => $key, 'label' => $this->translator->trans('access_incidents.' . $def['label']), 'count' => $counts[$key]];
        }
        if ($counts['autre'] > 0) {
            $tiles[] = ['key' => 'autre', 'label' => $this->translator->trans('access_incidents.cause_other'), 'count' => $counts['autre']];
        }
        $tiles[] = [
            'key' => 'all', 'label' => $this->translator->trans('access_incidents.tile_all'),
            'count' => $this->logs->countMatching($days, $readerId, $machineId, null),
        ];

        $source = match (true) {
            $cause === 'all' => $this->logs->search($days, $readerId, $machineId, null, self::LIMIT),
            $cause === 'todo' => $refused,
            default => array_values(array_filter($refused, fn (AccessRfidLog $l): bool => $this->causeOf($l) === $cause)),
        };
        $matching = $cause === 'all'
            ? $this->logs->countMatching($days, $readerId, $machineId, null)
            : count($source);

        $rows = [];
        foreach (array_slice($source, 0, self::LIMIT) as $log) {
            $rows[] = ['log' => $log, 'fix' => $this->incidents->of($log), 'cause' => $this->causeOf($log)];
        }

        return [
            'cause' => $cause,
            'days' => $days,
            'readerId' => $readerId,
            'machineId' => $machineId,
            'refined' => $days !== 7 || $readerId !== null || $machineId !== null,
            'tiles' => $tiles,
            'rows' => $rows,
            'shown' => count($rows),
            'matching' => $matching,
            'health' => $this->health(),
            'readerOptions' => array_map(
                static fn ($r): array => ['id' => (int) $r->getId(), 'name' => (string) $r->getName()],
                $this->readers->findForAdmin(),
            ),
            'machineOptions' => array_map(
                static fn ($m): array => ['id' => (int) $m->getId(), 'name' => (string) $m->getNom()],
                $this->machines->findBy([], ['nom' => 'ASC']),
            ),
        ];
    }

    /** Clé de cause d'un refus ; `autre` quand le statut n'est dans aucune famille. */
    private function causeOf(AccessRfidLog $log): string
    {
        foreach (self::CAUSES as $key => $def) {
            if (in_array($log->getStatus(), $def['statuses'], true)) {
                return $key;
            }
        }

        return 'autre';
    }

    /**
     * Santé des lecteurs EN SERVICE (archivés et désactivés exclus : ce sont des
     * choix, pas des pannes). `attention` = ceux qui ne sont pas « prêts ».
     *
     * @return array{ready: int, offline: int, other: int, attention: list<array<string, mixed>>}
     */
    private function health(): array
    {
        $ready = $offline = $other = 0;
        $attention = [];
        foreach ($this->readers->findForAdmin() as $reader) {
            $state = $this->readerHealth->of($reader);
            if (in_array($state['state'], [ReaderHealth::ARCHIVED, ReaderHealth::DISABLED], true)) {
                continue;
            }
            if ($state['state'] === ReaderHealth::READY) {
                ++$ready;
                continue;
            }
            $state['state'] === ReaderHealth::OFFLINE ? ++$offline : ++$other;
            $attention[] = [
                'id' => $reader->getId(),
                'name' => $reader->getName(),
                'where' => $reader->getMachine()?->getNom() ?? $reader->getAccessPoint()?->getNom() ?? '',
                'label' => $state['label'],
                'signal' => $state['signal'],
                'lastSeen' => $reader->getLastSeenAt(),
            ];
        }

        return ['ready' => $ready, 'offline' => $offline, 'other' => $other, 'attention' => $attention];
    }
}
