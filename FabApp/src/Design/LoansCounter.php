<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\Loan;
use App\Repository\LoanRepository;

/**
 * La liste de travail du comptoir « Prêts » (proposition `prets-admin`,
 * 2026-10-01, d'après la planche `productwide/02-prets.png`).
 *
 * ⚠️ LECTURE SEULE, rien de neuf : mêmes prêts que `/admin/loans`
 * (`LoanRepository::findAllSafe`), même état (`Loan::getEffectiveStatus()`).
 * Ce qui s'ajoute : la tuile « À rendre aujourd'hui » (échéance = aujourd'hui,
 * prêt non rendu) et un tri qui met le retard le plus ancien en tête.
 *
 * Forme d'une ligne : `id`, `itemId`, `item`, `borrower`, `taken`, `due`,
 * `lateDays`, `returnedOn` (DateTimeImmutable|null), `state` {label, signal},
 * `due_today`, `returned`. Les dates sont des heures saisies (`|date`).
 *
 * Ce que la planche montre et que FabOS n'a pas : des accessoires cochés au
 * retour (aucun champ « accessoires » sur `LoanableItem` ni `Loan`), un bouton
 * « Relancer » (les rappels de retard partent seuls, `ReminderSettings::LOAN_OVERDUE`,
 * il n'existe pas d'envoi manuel).
 */
final class LoansCounter
{
    /** Tuiles de travail : clé => libellé. `en-cours` est la vue par défaut. */
    private const TILES = [
        'aujourdhui' => 'À rendre aujourd’hui',
        'retard' => 'En retard',
        'en-cours' => 'En cours',
        'rendus' => 'Rendus',
    ];

    /** Prêts rendus montrés : le reste est l'historique de la liste réelle. */
    private const RETURNED_LIMIT = 30;

    public function __construct(private readonly LoanRepository $loans)
    {
    }

    /** @return array{tiles: list<array<string, mixed>>, rows: list<array<string, mixed>>, tile: string, q: string, total: int} */
    public function build(string $tile, string $q): array
    {
        $tile = array_key_exists($tile, self::TILES) ? $tile : 'en-cours';
        $today = new \DateTimeImmutable('today');

        $counts = array_fill_keys(array_keys(self::TILES), 0);
        $rows = [];
        foreach ($this->loans->findAllSafe() as $loan) {
            $row = $this->row($loan, $today);
            if ($row['returned']) {
                ++$counts['rendus'];
            } else {
                ++$counts['en-cours'];
                $row['overdue'] && ++$counts['retard'];
                $row['due_today'] && ++$counts['aujourdhui'];
            }
            $rows[] = $row;
        }

        $shown = array_values(array_filter($rows, static function (array $row) use ($tile, $q): bool {
            $in = match ($tile) {
                'aujourdhui' => $row['due_today'],
                'retard' => $row['overdue'],
                'rendus' => $row['returned'],
                default => !$row['returned'],
            };

            return $in && ($q === '' || mb_stripos($row['item'] . ' ' . $row['borrower'], $q) !== false);
        }));

        // Le retard le plus ancien d'abord, puis l'échéance la plus proche ;
        // les rendus restent du plus récent au plus ancien (ordre du dépôt).
        if ($tile !== 'rendus') {
            usort($shown, static fn (array $a, array $b): int => [$a['due']?->getTimestamp() ?? PHP_INT_MAX, $a['id']] <=> [$b['due']?->getTimestamp() ?? PHP_INT_MAX, $b['id']]);
        } else {
            $shown = \array_slice($shown, 0, self::RETURNED_LIMIT);
        }

        $tiles = [];
        foreach (self::TILES as $key => $label) {
            $tiles[] = [
                'label' => $label,
                'count' => $counts[$key],
                'query' => ['tuile' => $key],
                'active' => $key === $tile,
            ];
        }

        return ['tiles' => $tiles, 'rows' => $shown, 'tile' => $tile, 'q' => $q, 'total' => \count($rows)];
    }

    /** @return array<string, mixed> */
    private function row(Loan $loan, \DateTimeImmutable $today): array
    {
        $status = $loan->getEffectiveStatus();
        $due = $loan->getExpectedReturnDate();
        $returned = $status === 'returned';
        $dueToday = !$returned && $due !== null && $due->format('Y-m-d') === $today->format('Y-m-d');
        $overdue = $status === 'overdue';

        return [
            'id' => (int) $loan->getId(),
            'itemId' => $loan->getItem()?->getId(),
            'item' => $loan->getItem()?->getName() ?? 'Objet prêté',
            'borrower' => $loan->getBorrowerDisplay(),
            'taken' => $loan->getDateTaken(),
            'due' => $due,
            'returnedOn' => $loan->getActualReturnDate(),
            'lateDays' => $overdue && $due !== null ? (int) $due->diff($today)->days : 0,
            'overdue' => $overdue,
            'due_today' => $dueToday,
            'returned' => $returned,
            'state' => match (true) {
                $returned => ['label' => 'Rendu', 'signal' => 'go'],
                $overdue => ['label' => 'En retard', 'signal' => 'stop'],
                $dueToday => ['label' => 'À rendre aujourd’hui', 'signal' => 'caution'],
                default => ['label' => 'En cours', 'signal' => 'wait'],
            },
        ];
    }
}
