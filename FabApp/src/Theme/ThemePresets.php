<?php

namespace App\Theme;

/**
 * Le rayon, la densité et l'échelle typographique — **en préréglages** (S166).
 *
 * 🔴 **Des listes FERMÉES, pas des champs de nombre, et c'est la décision qui
 * compte ici.** Un champ « rayon » laisse taper 40 px et transforme chaque carte
 * en gélule ; un champ « taille du texte » laisse taper 24 px et fait déborder
 * chaque composant à hauteur fixe. Trois choix par axe, ça se REGARDE avant de
 * livrer — neuf combinaisons en tout, toutes visibles dans l'aperçu de S167.
 *
 * 🔴 **Le piège trouvé en lisant `style.css`, pas en livrant** : sous 576 px,
 * `--spacing-lg`, `--xl`, `--2xl` et `--3xl` descendent d'un cran dans un
 * `@media`. Le `<style>` du thème est émis APRÈS la feuille, à spécificité égale
 * — donc un `:root` de thème aurait GAGNÉ partout, y compris sur mobile, et
 * aurait supprimé cette réduction sans que rien ne le dise. La densité réémet
 * donc le palier mobile dans le même `@media`.
 * ⚠️ C'est aussi pourquoi les valeurs sont calculées ici et pas écrites à la
 * main : deux barèmes recopiés divergent, et celui qu'on oublie est celui du
 * mobile.
 *
 * ⚠️ **`--font-size-md` n'est PAS réémis** : `style.css` le définit comme
 * `var(--font-size-base)`, donc il suit tout seul. Le réémettre en pixels le
 * figerait, et il cesserait de suivre au premier changement de barème.
 */
final class ThemePresets
{
    /** @var array<string, array{0: int, 1: int}> clé => [rayon, rayon court] en px */
    public const RADIUS = [
        'net' => [2, 1],
        'standard' => [8, 4],
        'doux' => [14, 8],
    ];

    /**
     * ⚠️ Un FACTEUR, pas sept valeurs : le barème garde ses proportions, et le
     * palier mobile se recalcule au lieu d'être recopié.
     *
     * @var array<string, float>
     */
    public const DENSITY = [
        'compacte' => 0.75,
        'standard' => 1.0,
        'aeree' => 1.25,
    ];

    /** @var array<string, float> */
    public const TYPE = [
        'petite' => 0.9375,
        'standard' => 1.0,
        'grande' => 1.125,
    ];

    /** Le barème livré, celui de `style.css`. */
    private const SPACING = ['xs' => 4, 'sm' => 8, 'md' => 16, 'lg' => 24, 'xl' => 32, '2xl' => 48, '3xl' => 64];

    /** ⚠️ Le palier sous 576 px, tel que `style.css` le déclare. */
    private const SPACING_SMALL = ['lg' => 16, 'xl' => 24, '2xl' => 32, '3xl' => 40];

    private const FONT = ['xs' => 12, 'sm' => 14, 'base' => 16, 'lg' => 18, 'xl' => 20, '2xl' => 24, '3xl' => 32, '4xl' => 40];

    public static function isKnown(string $axis, string $value): bool
    {
        return match ($axis) {
            'radius' => array_key_exists($value, self::RADIUS),
            'density' => array_key_exists($value, self::DENSITY),
            'typeScale' => array_key_exists($value, self::TYPE),
            default => false,
        };
    }

    /**
     * Le corps du `<style>` à émettre, ou **une chaîne vide** quand les trois axes
     * sont au préréglage livré.
     *
     * 🔴 **Vide veut dire vide** : sans choix, aucune règle n'est émise et le
     * balisage reste identique au bit près. C'est la même garantie qu'à S160 pour
     * les e-mails, et c'est ce qui rend « je n'ai rien changé » vérifiable.
     */
    public function css(string $radius, string $density, string $typeScale): string
    {
        $rules = [];
        $small = [];

        if ($radius !== '' && $radius !== 'standard' && isset(self::RADIUS[$radius])) {
            [$base, $short] = self::RADIUS[$radius];
            $rules[] = "--border-radius: {$base}px";
            $rules[] = "--border-radius-sm: {$short}px";
        }

        if ($density !== '' && $density !== 'standard' && isset(self::DENSITY[$density])) {
            $factor = self::DENSITY[$density];
            foreach (self::SPACING as $name => $px) {
                $rules[] = '--spacing-' . $name . ': ' . self::scale($px, $factor) . 'px';
            }
            // 🔴 Le palier mobile, réémis dans le MÊME `@media` : sans lui, le
            // thème l'écraserait et les grands écarts resteraient sur un
            // téléphone.
            foreach (self::SPACING_SMALL as $name => $px) {
                $small[] = '--spacing-' . $name . ': ' . self::scale($px, $factor) . 'px';
            }
        }

        if ($typeScale !== '' && $typeScale !== 'standard' && isset(self::TYPE[$typeScale])) {
            $factor = self::TYPE[$typeScale];
            foreach (self::FONT as $name => $px) {
                $rules[] = '--font-size-' . $name . ': ' . self::scale($px, $factor) . 'px';
            }
        }

        if ($rules === []) {
            return '';
        }

        $css = ':root { ' . implode('; ', $rules) . '; }';
        if ($small !== []) {
            $css .= ' @media (max-width: 576px) { :root { ' . implode('; ', $small) . '; } }';
        }

        return $css;
    }

    /** ⚠️ Jamais zéro : un espacement nul colle deux éléments qui doivent se distinguer. */
    private static function scale(int $px, float $factor): int
    {
        return max(1, (int) round($px * $factor));
    }
}
