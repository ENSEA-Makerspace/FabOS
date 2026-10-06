<?php

declare(strict_types=1);

namespace App\Page;

use App\Repository\MachineCategoryRepository;
use Doctrine\DBAL\Connection;

/**
 * « Créer une machine » : ce que le lab SAIT déjà, pour que l'écran propose au lieu
 * de demander (demande de l'opérateur : « ne précharge pas les groupes existants »).
 *
 * Ne stocke rien et n'invente aucun catalogue : tout est lu des machines existantes.
 * Un modèle « connu » est un couple marque/modèle déjà saisi — comparé sans casse ni
 * espaces, donc « BambuLab » et « Bambu Lab » sont le même.
 */
final class MachineCreationHints
{
    public function __construct(
        private readonly Connection $db,
        private readonly MachineCategoryRepository $categories,
    ) {
    }

    /** @return array<string, mixed> */
    public function all(): array
    {
        $used = $this->db->fetchAllKeyValue("SELECT categoryLabel, COUNT(*) FROM MACHINE WHERE archivedAt IS NULL AND categoryLabel <> '' GROUP BY categoryLabel");
        $categories = [];
        foreach ($this->categories->allOrdered(includeArchived: false) as $category) {
            $categories[$category->getLabel()] = (int) ($used[$category->getLabel()] ?? 0);
        }
        arsort($categories);

        // Les badges qu'une catégorie demande d'habitude : au moins la moitié de ses machines.
        $badgesByCategory = [];
        foreach ($this->db->fetchAllAssociative("SELECT m.categoryLabel c, mb.badgeId b, COUNT(*) n FROM MACHINE_BADGE mb JOIN MACHINE m ON m.id = mb.machineId WHERE m.archivedAt IS NULL AND m.categoryLabel <> '' GROUP BY 1, 2") as $row) {
            if ((int) $row['n'] * 2 >= (int) ($used[$row['c']] ?? 0)) {
                $badgesByCategory[$row['c']][] = (int) $row['b'];
            }
        }

        return [
            'categories' => $categories,
            'badgesByCategory' => $badgesByCategory,
            'locations' => $this->db->fetchAllKeyValue("SELECT localisation, COUNT(*) n FROM MACHINE WHERE archivedAt IS NULL AND localisation <> '' GROUP BY localisation ORDER BY n DESC, localisation LIMIT 8"),
            'slots' => $this->db->fetchFirstColumn("SELECT granularite FROM MACHINE WHERE granularite <> '' GROUP BY granularite ORDER BY CAST(granularite AS UNSIGNED)"),
            'models' => $this->models(),
            'materials' => $this->vocabulary('materials'),
            'features' => $this->vocabulary('features'),
            'modelNames' => $this->db->fetchFirstColumn("SELECT model FROM MACHINE WHERE model <> '' GROUP BY model ORDER BY COUNT(*) DESC"),
            'manufacturers' => $this->db->fetchFirstColumn("SELECT manufacturer FROM MACHINE WHERE manufacturer <> '' GROUP BY manufacturer ORDER BY COUNT(*) DESC"),
        ];
    }

    /**
     * La fiche d'une machine, prête à en créer une autre : les colonnes BRUTES (pas les
     * valeurs par défaut des accesseurs de l'entité), ses badges, et le nom suivant —
     * « Creality K1 Max n°4 » propose « Creality K1 Max n°5 ».
     *
     * @return array<string, mixed>|null
     */
    public function copyOf(int $machineId): ?array
    {
        $row = $this->db->fetchAssociative('SELECT nom, manufacturer, model, categoryLabel, levelSlug, localisation, granularite, photo, iconSlug, materials, features, requirementDescription, description, venueId FROM MACHINE WHERE id = ?', [$machineId]);
        if ($row === false) {
            return null;
        }
        $base = trim((string) preg_replace('/\s*(n°|#)?\s*\d+$/u', '', (string) $row['nom']));
        $siblings = (int) $this->db->fetchOne('SELECT COUNT(*) FROM MACHINE WHERE nom = ? OR nom LIKE ?', [$base, addcslashes($base, '%_') . ' %']);
        $list = static fn (?string $json): array => array_values(array_filter((array) json_decode($json ?? '[]', true), 'is_string'));
        $level = (int) preg_replace('/\D/', '', (string) $row['levelSlug']);

        return [
            'nom' => $base . ' n°' . (max(1, $siblings) + 1),
            'category' => $row['categoryLabel'] ?: null,
            'level' => $level >= 1 && $level <= 3 ? $level : null,
            'venueId' => (int) $row['venueId'],
            'localisation' => $row['localisation'] ?: null,
            'granularite' => $row['granularite'] ?: null,
            'manufacturer' => $row['manufacturer'] ?: null,
            'model' => $row['model'] ?: null,
            'photo' => $row['photo'] ?: null,
            'icon' => $row['iconSlug'] ?: null,
            'description' => $row['description'] ?: null,
            'materials' => $list($row['materials']),
            'features' => $list($row['features']),
            'requirement' => $row['requirementDescription'] ?: null,
            'badges' => array_map('intval', $this->db->fetchFirstColumn('SELECT badgeId FROM MACHINE_BADGE WHERE machineId = ?', [$machineId])),
        ];
    }

    /**
     * Les mots déjà saisis dans une liste libre (matériaux, caractéristiques), les plus
     * employés d'abord : ils deviennent des étiquettes à cliquer et l'autocomplétion.
     *
     * @return array<string, int>
     */
    private function vocabulary(string $column): array
    {
        $counts = [];
        foreach ($this->db->fetchFirstColumn("SELECT $column FROM MACHINE WHERE archivedAt IS NULL AND $column IS NOT NULL") as $json) {
            foreach ((array) json_decode((string) $json, true) as $word) {
                if (\is_string($word) && trim($word) !== '') {
                    $counts[trim($word)] = ($counts[trim($word)] ?? 0) + 1;
                }
            }
        }
        arsort($counts);

        return $counts;
    }

    /**
     * Les modèles que le lab possède en plusieurs exemplaires d'abord. Chacun désigne
     * son exemplaire le plus récent : c'est sa fiche qu'un clic recopie (`copyOf()`).
     *
     * @return list<array<string, mixed>>
     */
    private function models(): array
    {
        $groups = [];
        foreach ($this->db->fetchAllAssociative("SELECT id, manufacturer, model FROM MACHINE WHERE archivedAt IS NULL AND model <> '' ORDER BY id DESC") as $row) {
            $key = preg_replace('/\s+/', '', mb_strtolower(($row['manufacturer'] ?? '') . '|' . $row['model']));
            $groups[$key] ??= ['count' => 0, 'last' => $row];
            ++$groups[$key]['count'];
        }
        uasort($groups, static fn (array $a, array $b): int => $b['count'] <=> $a['count']);
        $groups = \array_slice($groups, 0, 6);

        $models = [];
        foreach ($groups as $group) {
            $models[] = [
                'id' => (int) $group['last']['id'],
                'label' => trim(($group['last']['manufacturer'] ?? '') . ' ' . $group['last']['model']),
                'count' => $group['count'],
            ];
        }

        return $models;
    }
}
