<?php

declare(strict_types=1);

namespace App\Page;

use App\Repository\MachineCategoryRepository;
use Doctrine\DBAL\ArrayParameterType;
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
            'manufacturers' => $this->db->fetchFirstColumn("SELECT manufacturer FROM MACHINE WHERE manufacturer <> '' GROUP BY manufacturer ORDER BY COUNT(*) DESC"),
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
     * Les modèles que le lab possède en plusieurs exemplaires d'abord. Chacun porte la
     * fiche de son exemplaire le plus récent : c'est elle qu'un clic recopie.
     *
     * @return list<array<string, mixed>>
     */
    private function models(): array
    {
        $groups = [];
        foreach ($this->db->fetchAllAssociative("SELECT id, nom, manufacturer, model, categoryLabel, levelSlug, localisation, granularite, photo, iconSlug, materials, features, requirementDescription, venueId FROM MACHINE WHERE archivedAt IS NULL AND model <> '' ORDER BY id DESC") as $row) {
            $key = preg_replace('/\s+/', '', mb_strtolower(($row['manufacturer'] ?? '') . '|' . $row['model']));
            $groups[$key] ??= ['count' => 0, 'last' => $row];
            ++$groups[$key]['count'];
        }
        uasort($groups, static fn (array $a, array $b): int => $b['count'] <=> $a['count']);
        $groups = \array_slice($groups, 0, 6);

        $ids = array_map(static fn (array $g): int => (int) $g['last']['id'], $groups);
        $badges = [];
        if ($ids !== []) {
            foreach ($this->db->fetchAllAssociative('SELECT machineId, badgeId FROM MACHINE_BADGE WHERE machineId IN (?)', [array_values($ids)], [ArrayParameterType::INTEGER]) as $row) {
                $badges[(int) $row['machineId']][] = (int) $row['badgeId'];
            }
        }

        $models = [];
        foreach ($groups as $group) {
            $last = $group['last'];
            $lines = static fn (?string $json): string => implode("\n", array_filter((array) json_decode($json ?? '[]', true), 'is_string'));
            $models[] = [
                'label' => trim(($last['manufacturer'] ?? '') . ' ' . $last['model']),
                'count' => $group['count'],
                'fill' => [
                    // « Creality K1 Max n°4 » propose « Creality K1 Max n°5 ».
                    'nom' => trim((string) preg_replace('/\s*(n°|#)?\s*\d+$/u', '', $last['nom'])) . ' n°' . ($group['count'] + 1),
                    'categorie' => (string) $last['categoryLabel'],
                    'niveau' => (string) preg_replace('/\D/', '', (string) $last['levelSlug']),
                    'venue' => (string) $last['venueId'],
                    'localisation' => (string) $last['localisation'],
                    'granularite' => (string) $last['granularite'],
                    'manufacturer' => (string) $last['manufacturer'],
                    'model' => (string) $last['model'],
                    'photo' => (string) $last['photo'],
                    'icone' => (string) $last['iconSlug'],
                    'materiaux' => $lines($last['materials']),
                    'caracteristiques' => $lines($last['features']),
                    'prerequis' => (string) $last['requirementDescription'],
                    'badges' => $badges[(int) $last['id']] ?? [],
                ],
            ];
        }

        return $models;
    }
}
