<?php

declare(strict_types=1);

namespace App\Service;

use App\Feature\SiteFeatureService;
use App\Mail\Mailer;
use App\Mail\NotificationCategory;
use App\Repository\UtilisateurRepository;
use Doctrine\DBAL\Connection;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S206 — le stock de consommables d'un matériau (`MATERIAL_STOCK`,
 * `MATERIAL_STOCK_MOVE`), sous-fonction activable (`stock`) de Matériaux.
 *
 * 🔴 **Éteint = aucune mention de stock nulle part.** Tout écran demande
 * `active()` avant de dire un mot : la clé de fonctionnalité (par
 * `allowsSurface()`, jamais `isEnabled()` seul) ET les tables. Tant que la
 * migration manque, `isReady()` est faux et la fonction est inerte.
 *
 * Règle d'état, une seule fois ici : « épuisé » = quantité ≤ 0 ; « bientôt
 * épuisé » = sous le seuil (strictement) ; sinon « en stock ». Un matériau sans
 * ligne n'est pas suivi : aucune pastille, rien à dire.
 *
 * Horodatages en UTC (affichés par `|lab_date`).
 */
final class MaterialStock
{
    public const IN = 'in';
    public const LOW = 'low';
    public const OUT = 'out';

    /** Les unités proposées. `pcs` se traduit ; les autres sont des symboles. */
    public const UNITS = ['g', 'kg', 'm', 'L', 'mL', 'pcs'];

    private ?bool $ready = null;

    /** @var array<int, array<string, mixed>>|null */
    private ?array $rows = null;

    public function __construct(
        private readonly Connection $db,
        private readonly SiteFeatureService $features,
        private readonly Mailer $mailer,
        private readonly UtilisateurRepository $users,
        private readonly UrlGeneratorInterface $urls,
        private readonly TranslatorInterface $translator,
    ) {
    }

    public function isReady(): bool
    {
        if ($this->ready === null) {
            try {
                $this->db->fetchOne('SELECT 1 FROM MATERIAL_STOCK LIMIT 1');
                $this->db->fetchOne('SELECT 1 FROM MATERIAL_STOCK_MOVE LIMIT 1');
                $this->ready = true;
            } catch (\Throwable) {
                $this->ready = false;
            }
        }

        return $this->ready;
    }

    /** La fonction est allumée ET ses tables existent : la seule porte des écrans. */
    public function active(): bool
    {
        return $this->features->allowsSurface('stock') && $this->isReady();
    }

    /** @return array<int, array<string, mixed>> par matériau, avec `state` */
    private function all(): array
    {
        if ($this->rows === null) {
            $this->rows = [];
            if ($this->isReady()) {
                foreach ($this->db->fetchAllAssociative('SELECT materialId, quantity, unit, lowThreshold, updatedAt FROM MATERIAL_STOCK') as $row) {
                    $row['materialId'] = (int) $row['materialId'];
                    $row['quantity'] = (float) $row['quantity'];
                    $row['lowThreshold'] = $row['lowThreshold'] === null ? null : (float) $row['lowThreshold'];
                    $row['state'] = self::stateOf($row['quantity'], $row['lowThreshold']);
                    $this->rows[$row['materialId']] = $row;
                }
            }
        }

        return $this->rows;
    }

    /** @return array<string, mixed>|null null : matériau non suivi (ou fonction inactive) */
    public function of(int $materialId): ?array
    {
        return $this->active() ? ($this->all()[$materialId] ?? null) : null;
    }

    public static function stateOf(float $quantity, ?float $threshold): string
    {
        if ($quantity <= 0) {
            return self::OUT;
        }

        return $threshold !== null && $quantity < $threshold ? self::LOW : self::IN;
    }

    /** « 1,5 kg » : nombre sans zéros inutiles, suivi de l'unité. */
    public function format(float $quantity, string $unit): string
    {
        return $this->number($quantity) . ' ' . $this->unitLabel($unit);
    }

    public function number(float $quantity): string
    {
        return rtrim(rtrim(number_format($quantity, 3, ',', ' '), '0'), ',');
    }

    public function unitLabel(string $unit): string
    {
        return $unit === 'pcs' ? $this->translator->trans('stock.unit_pcs') : $unit;
    }

    /**
     * Crée ou règle la ligne d'un matériau. Une quantité qui change passe par le
     * journal (un mouvement « inventaire »), pour que le journal reste la vérité.
     */
    public function configure(int $materialId, float $quantity, string $unit, ?float $threshold, ?int $userId): void
    {
        if (!$this->active()) {
            return;
        }
        $unit = in_array($unit, self::UNITS, true) ? $unit : self::UNITS[0];
        $quantity = max(0.0, round($quantity, 3));
        $threshold = $threshold === null ? null : max(0.0, round($threshold, 3));

        $existing = $this->all()[$materialId] ?? null;
        $this->db->beginTransaction();
        try {
            if ($existing === null) {
                $this->db->executeStatement(
                    'INSERT INTO MATERIAL_STOCK (materialId, quantity, unit, lowThreshold, updatedAt) VALUES (?, 0, ?, ?, ?)',
                    [$materialId, $unit, $threshold, $this->now()],
                );
                $before = 0.0;
            } else {
                $this->db->executeStatement(
                    'UPDATE MATERIAL_STOCK SET unit = ?, lowThreshold = ?, updatedAt = ? WHERE materialId = ?',
                    [$unit, $threshold, $this->now(), $materialId],
                );
                $before = (float) $existing['quantity'];
            }
            if (abs($quantity - $before) > 0.0004) {
                $this->db->executeStatement('UPDATE MATERIAL_STOCK SET quantity = ?, updatedAt = ? WHERE materialId = ?', [$quantity, $this->now(), $materialId]);
                $this->db->executeStatement(
                    'INSERT INTO MATERIAL_STOCK_MOVE (materialId, delta, note, userId, createdAt) VALUES (?, ?, ?, ?, ?)',
                    [$materialId, round($quantity - $before, 3), $this->translator->trans('stock.note_inventory'), $userId, $this->now()],
                );
            }
            $this->db->commit();
        } catch (\Throwable $e) {
            $this->db->rollBack();
            throw $e;
        }
        $this->rows = null;

        if ($existing !== null && abs($quantity - $before) > 0.0004) {
            $this->alertIfCrossed($materialId, $before, $quantity, $threshold);
        }
    }

    /**
     * Une entrée (delta > 0) ou une sortie (delta < 0).
     *
     * @return string|null null si c'est fait ; sinon la clé `stock.err_*` du refus
     */
    public function move(int $materialId, float $delta, string $note, ?int $userId): ?string
    {
        $row = $this->of($materialId);
        $delta = round($delta, 3);
        if ($row === null) {
            return 'stock.err_untracked';
        }
        if ($delta == 0.0) {
            return 'stock.err_zero';
        }

        $this->db->beginTransaction();
        try {
            $before = (float) $this->db->fetchOne('SELECT quantity FROM MATERIAL_STOCK WHERE materialId = ? FOR UPDATE', [$materialId]);
            $after = round($before + $delta, 3);
            if ($after < 0) {
                $this->db->rollBack();

                return 'stock.err_negative';
            }
            $this->db->executeStatement('UPDATE MATERIAL_STOCK SET quantity = ?, updatedAt = ? WHERE materialId = ?', [$after, $this->now(), $materialId]);
            $this->db->executeStatement(
                'INSERT INTO MATERIAL_STOCK_MOVE (materialId, delta, note, userId, createdAt) VALUES (?, ?, ?, ?, ?)',
                [$materialId, $delta, $note === '' ? null : mb_substr($note, 0, 255), $userId, $this->now()],
            );
            $this->db->commit();
        } catch (\Throwable $e) {
            $this->db->rollBack();
            throw $e;
        }
        $this->rows = null;
        $this->alertIfCrossed($materialId, $before, $after, $row['lowThreshold']);

        return null;
    }

    /** @return list<array{delta: float, note: ?string, who: ?string, at: string}> les plus récents d'abord */
    public function moves(int $materialId, int $limit = 10): array
    {
        if (!$this->active()) {
            return [];
        }

        return array_map(static fn (array $r): array => [
            'delta' => (float) $r['delta'], 'note' => $r['note'], 'who' => $r['who'], 'at' => (string) $r['createdAt'],
        ], $this->db->fetchAllAssociative(
            'SELECT m.delta, m.note, m.createdAt, TRIM(CONCAT(COALESCE(u.firstName, \'\'), \' \', COALESCE(u.lastName, \'\'))) AS who
             FROM MATERIAL_STOCK_MOVE m LEFT JOIN UTILISATEUR u ON u.id = m.userId
             WHERE m.materialId = ? ORDER BY m.id DESC LIMIT ' . max(1, $limit),
            [$materialId],
        ));
    }

    /**
     * Les matériaux vivants à surveiller : épuisés ou sous leur seuil.
     *
     * @return list<array{id: int, name: string, category: ?string, state: string, quantity: float, unit: string}>
     */
    public function lowMaterials(): array
    {
        if (!$this->active()) {
            return [];
        }
        $stock = array_filter($this->all(), static fn (array $r): bool => $r['state'] !== self::IN);
        if ($stock === []) {
            return [];
        }

        $out = [];
        foreach ($this->db->fetchAllAssociative(
            'SELECT id, name, category FROM MATERIAL WHERE archivedAt IS NULL AND id IN (?) ORDER BY name',
            [array_keys($stock)],
            [\Doctrine\DBAL\ArrayParameterType::INTEGER],
        ) as $m) {
            $row = $stock[(int) $m['id']];
            $out[] = ['id' => (int) $m['id'], 'name' => (string) $m['name'], 'category' => $m['category'], 'state' => $row['state'], 'quantity' => $row['quantity'], 'unit' => (string) $row['unit']];
        }
        // Les épuisés d'abord.
        usort($out, static fn (array $a, array $b): int => [$a['state'] === self::OUT ? 0 : 1, $a['name']] <=> [$b['state'] === self::OUT ? 0 : 1, $b['name']]);

        return $out;
    }

    /**
     * 🔴 Une alerte PAR FRANCHISSEMENT : seulement quand ce mouvement fait passer
     * la quantité de « au seuil ou au-dessus » à « sous le seuil ». Les mouvements
     * suivants, déjà dessous, se taisent ; un réassort puis une nouvelle chute en
     * refont une.
     */
    private function alertIfCrossed(int $materialId, float $before, float $after, ?float $threshold): void
    {
        if ($threshold === null || !($before >= $threshold && $after < $threshold)) {
            return;
        }
        $name = $this->db->fetchOne('SELECT name FROM MATERIAL WHERE id = ?', [$materialId]);
        $unit = (string) $this->db->fetchOne('SELECT unit FROM MATERIAL_STOCK WHERE materialId = ?', [$materialId]);
        if (!is_string($name)) {
            return;
        }

        $recipients = [];
        foreach ($this->users->findStaff() as $user) {
            $recipients[$user->getId()] = $user;
        }
        foreach ($this->users->findBy(['statut' => 'actif']) as $user) {
            if (in_array('ROLE_ADMIN', $user->getRoles(), true)) {
                $recipients[$user->getId()] = $user;
            }
        }

        $context = [
            'material' => $name,
            'quantity' => $this->format($after, $unit),
            'threshold' => $this->format($threshold, $unit),
            'link' => $this->urls->generate('app_admin_material_edit', ['id' => $materialId], UrlGeneratorInterface::ABSOLUTE_URL),
        ];
        foreach ($recipients as $user) {
            // Non transactionnel : l'interrupteur général du compte l'arrête ; aucune catégorie à couper (GENERAL).
            $this->mailer->queueToUser($user, 'stock_low', $context, NotificationCategory::GENERAL, false);
        }
    }

    private function now(): string
    {
        return (new \DateTimeImmutable('now', new \DateTimeZone('UTC')))->format('Y-m-d H:i:s');
    }
}
