<?php

declare(strict_types=1);

namespace App\Twig;

use App\Service\MaterialStock;
use Symfony\Contracts\Translation\TranslatorInterface;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * S206 — `stock_active()` et `stock_of(id)` pour les gabarits des matériaux.
 *
 * 🔴 Éteint, les deux ne rendent RIEN (faux / null) : un gabarit qui s'y tient
 * ne dit jamais le mot « stock ». `stock_of` rend déjà la pastille (`label`,
 * `signal`) ; la quantité exacte (`amount`) n'est à afficher qu'au personnel.
 */
final class MaterialStockExtension extends AbstractExtension
{
    private const SIGNALS = [MaterialStock::IN => 'go', MaterialStock::LOW => 'caution', MaterialStock::OUT => 'stop'];

    public function __construct(private readonly MaterialStock $stock, private readonly TranslatorInterface $translator)
    {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('stock_active', $this->stock->active(...)),
            new TwigFunction('stock_of', $this->of(...)),
            new TwigFunction('stock_moves', fn (int $id): array => $this->stock->moves($id, 10)),
            new TwigFunction('stock_number', $this->stock->number(...)),
            new TwigFunction('stock_unit_label', $this->stock->unitLabel(...)),
            new TwigFunction('stock_units', static fn (): array => MaterialStock::UNITS),
        ];
    }

    /** @return array{state: string, label: string, signal: string, amount: string, number: string, unit: string, threshold: ?string, thresholdNumber: ?string, raw: float}|null */
    public function of(?int $materialId): ?array
    {
        $row = $materialId === null ? null : $this->stock->of($materialId);
        if ($row === null) {
            return null;
        }

        return [
            'state' => $row['state'],
            'label' => $this->translator->trans('stock.state_' . $row['state']),
            'signal' => self::SIGNALS[$row['state']],
            'amount' => $this->stock->format($row['quantity'], $row['unit']),
            'number' => $this->stock->number($row['quantity']),
            'unit' => $row['unit'],
            'threshold' => $row['lowThreshold'] === null ? null : $this->stock->format($row['lowThreshold'], $row['unit']),
            'thresholdNumber' => $row['lowThreshold'] === null ? null : $this->stock->number($row['lowThreshold']),
            'raw' => $row['quantity'],
        ];
    }
}
