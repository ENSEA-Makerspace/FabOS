<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\AccessRfidLog;
use App\Entity\RfidReader;
use App\Repository\AccessRfidLogRepository;
use App\Repository\RfidReaderRepository;
use App\Rfid\ReaderCommissioning;
use App\Rfid\ReaderHealth;

/**
 * La fiche d'UN lecteur RFID (proposition `fiche-lecteur`, 2026-10-01, d'après
 * `equipement/equipment-reader-health.png` et `equipment-reader-commissioning.png`).
 *
 * ⚠️ LECTURE SEULE et rien de neuf : l'état vient de `ReaderHealth`, la mise en
 * service de `ReaderCommissioning`, les événements du journal RFID filtré par
 * lecteur. Aucun secret ni jeton n'est exposé (ni `readerToken`, ni UID de badge).
 */
final class ReaderSheet
{
    private const EVENTS = 5;

    public function __construct(
        private readonly RfidReaderRepository $readers,
        private readonly ReaderHealth $health,
        private readonly ReaderCommissioning $commissioning,
        private readonly AccessRfidLogRepository $logs,
    ) {
    }

    /**
     * @return array{
     *   reader: ?RfidReader, health: ?array{state: string, signal: string, label: string}, headline: string,
     *   others: list<array{reader: RfidReader, health: array{state: string, signal: string, label: string}, current: bool}>,
     *   steps: list<array{key: string, done: bool, blocking: bool}>, blockingStep: ?string, commissioning: bool,
     *   events: list<array{who: string, outcome: string, authorized: bool, detail: string, at: \DateTimeImmutable}>
     * }
     */
    public function build(?int $id): array
    {
        $active = array_values(array_filter($this->readers->findForAdmin(), static fn (RfidReader $r): bool => !$r->isArchived()));
        $reader = null;
        foreach ($active as $candidate) {
            if ($id !== null && $candidate->getId() === $id) {
                $reader = $candidate;
            }
        }
        $reader ??= $active[0] ?? null;

        $others = [];
        foreach ($active as $candidate) {
            $others[] = ['reader' => $candidate, 'health' => $this->health->of($candidate), 'current' => $candidate === $reader];
        }
        if ($reader === null) {
            return ['reader' => null, 'health' => null, 'headline' => '', 'others' => [], 'steps' => [], 'blockingStep' => null, 'commissioning' => false, 'events' => []];
        }

        $health = $this->health->of($reader);
        $steps = $this->commissioning->steps($reader);
        $blocking = null;
        foreach ($steps as $step) {
            if (!$step['done'] && $step['blocking']) {
                $blocking = $step['key'];
                break;
            }
        }
        $open = array_filter($steps, static fn (array $s): bool => !$s['done']);

        $events = [];
        foreach ($this->logs->search(0, $reader->getId(), null, null, self::EVENTS) as $log) {
            /** @var AccessRfidLog $log */
            $events[] = [
                'who' => $log->getUtilisateur()?->getDisplayName() ?? 'Badge inconnu',
                'outcome' => $log->isAuthorized() ? 'Accès autorisé' : 'Accès refusé',
                'authorized' => $log->isAuthorized(),
                'detail' => $log->isAuthorized() ? '' : strtolower(str_replace('_', ' ', $log->getStatus())),
                'at' => $log->getCreatedAt(),
            ];
        }

        return [
            'reader' => $reader,
            'health' => $health,
            'headline' => match ($health['state']) {
                ReaderHealth::READY => 'Prêt',
                ReaderHealth::OFFLINE => 'Hors ligne',
                ReaderHealth::UNPAIRED, ReaderHealth::INVALID_PAIRING, ReaderHealth::NEVER_SEEN => 'À configurer',
                ReaderHealth::DISABLED => 'Désactivé',
                default => 'Archivé',
            },
            'others' => $others,
            'steps' => $steps,
            'blockingStep' => $blocking,
            'commissioning' => $open !== [],
            'events' => $events,
        ];
    }
}
