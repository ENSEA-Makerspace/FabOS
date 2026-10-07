<?php

declare(strict_types=1);

namespace App\Form;

use Doctrine\DBAL\Connection;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * « Proposer ce qui existe déjà » (0.7.0, étendu en 0.7.1) : les attributs qu'un
 * `FormType` pose sur un champ libre pour le contrôleur Stimulus `suggest`.
 *
 *     'attr' => $this->suggest->one($this->suggest->known('PLACE', 'localisation')),
 *
 * Le champ reste libre et rien n'est stocké en plus : les mots proposés sont ceux
 * déjà saisis dans la même colonne, les plus employés d'abord.
 *
 * ⚠️ **Pensé pour un lab qui a BEAUCOUP de fiches.** Le contrôleur n'affiche que
 * les `MAX_TILES` premiers mots en tuiles ; les autres restent atteignables par
 * l'autocomplétion du champ. Et on n'envoie jamais plus de `MAX_WORDS` mots.
 */
final class Suggest
{
    public const MAX_TILES = 8;
    private const MAX_WORDS = 150;

    public function __construct(
        private readonly Connection $db,
        private readonly TranslatorInterface $translator,
    ) {
    }

    /**
     * Un champ à UNE valeur. `$other` : le champ se cache derrière une tuile « Autre… ».
     *
     * @param array<string, int> $words mot → nombre d'usages
     * @param array<string, string> $more attributs en plus (`data-suggest-checks-value`…)
     *
     * @return array<string, string>
     */
    public function one(array $words, bool $other = false, array $more = []): array
    {
        return $words === [] ? [] : $this->attr($words, 'one') + ($other ? [
            'data-suggest-other-value' => '1',
            'data-suggest-other-label-value' => $this->translator->trans('form.suggest_other'),
        ] : []) + $more;
    }

    /**
     * Un `<textarea>` « un mot par ligne », rendu en étiquettes.
     *
     * @param array<string, int> $words
     *
     * @return array<string, string>
     */
    public function many(array $words): array
    {
        return $words === [] ? [] : $this->attr($words, 'many') + [
            'data-suggest-add-label-value' => $this->translator->trans('form.suggest_add'),
            'data-suggest-help-label-value' => $this->translator->trans('form.suggest_help'),
        ];
    }

    /**
     * Les valeurs déjà saisies dans une colonne texte, les plus employées d'abord.
     * ⚠️ Table et colonne sont des constantes du code appelant, jamais une saisie.
     *
     * @return array<string, int>
     */
    public function known(string $table, string $column): array
    {
        if (preg_match('/^\w+$/', $table . $column) !== 1) {
            throw new \InvalidArgumentException('Nom de table ou de colonne invalide.');
        }
        try {
            return array_map('intval', $this->db->fetchAllKeyValue(
                "SELECT TRIM($column) v, COUNT(*) n FROM $table WHERE $column IS NOT NULL AND TRIM($column) <> '' GROUP BY v ORDER BY n DESC, v LIMIT " . self::MAX_WORDS,
            ));
        } catch (\Throwable) {
            return []; // une colonne absente retire la proposition, pas l'écran
        }
    }

    /**
     * @param array<string, int> $words
     *
     * @return array<string, string>
     */
    private function attr(array $words, string $mode): array
    {
        // ⚠️ Une LISTE de paires, pas un objet : JSON trie les clés numériques
        // (« 30 », « 45 ») et perdrait l'ordre d'usage.
        $pairs = [];
        foreach (\array_slice($words, 0, self::MAX_WORDS, true) as $word => $count) {
            $pairs[] = [(string) $word, (int) $count];
        }

        return [
            'data-controller' => 'suggest',
            'data-suggest-words-value' => json_encode($pairs, \JSON_UNESCAPED_UNICODE),
            'data-suggest-mode-value' => $mode,
            'data-suggest-max-value' => (string) ($mode === 'many' ? 12 : self::MAX_TILES),
        ];
    }
}
