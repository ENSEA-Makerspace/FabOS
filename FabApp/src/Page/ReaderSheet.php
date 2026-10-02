<?php

declare(strict_types=1);

namespace App\Page;

use App\Entity\AccessRfidLog;
use App\Entity\RfidReader;
use App\Repository\AccessRfidLogRepository;
use App\Repository\RfidReaderRepository;
use App\Rfid\ReaderCommissioning;
use App\Rfid\ReaderHealth;
use Symfony\Contracts\Translation\TranslatorInterface;

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
        private readonly TranslatorInterface $translator,
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
        $all = $this->readers->findForAdmin();
        $active = array_values(array_filter($all, static fn (RfidReader $r): bool => !$r->isArchived()));
        $reader = null;
        // La fiche d'un lecteur ARCHIVÉ reste lisible (la liste l'affiche) ; seul
        // le sélecteur « autres lecteurs » ne propose que les actifs.
        foreach ($all as $candidate) {
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
                'who' => $log->getUtilisateur()?->getDisplayName() ?? $this->translator->trans('reader_sheet.unknown_badge'),
                'outcome' => $this->translator->trans($log->isAuthorized() ? 'reader_sheet.outcome_ok' : 'reader_sheet.outcome_refused'),
                'authorized' => $log->isAuthorized(),
                'detail' => $log->isAuthorized() ? '' : $this->statusLabel($log->getStatus()),
                'at' => $log->getCreatedAt(),
            ];
        }

        return [
            'reader' => $reader,
            'health' => $health,
            'headline' => $this->translator->trans(match ($health['state']) {
                ReaderHealth::READY => 'rfid_readers.state_ready',
                ReaderHealth::OFFLINE => 'rfid_readers.state_offline',
                ReaderHealth::UNPAIRED, ReaderHealth::INVALID_PAIRING, ReaderHealth::NEVER_SEEN => 'reader_sheet.headline_setup',
                ReaderHealth::DISABLED => 'rfid_readers.state_disabled',
                default => 'rfid_readers.state_archived',
            }),
            'others' => $others,
            'steps' => $steps,
            'blockingStep' => $blocking,
            'commissioning' => $open !== [],
            'events' => $events,
        ];
    }

    private function statusLabel(string $status): string
    {
        $id = 'rfid_logs.status_' . strtolower($status);
        $label = $this->translator->trans($id);

        return $label !== $id ? $label : strtolower(str_replace('_', ' ', $status));
    }
}
