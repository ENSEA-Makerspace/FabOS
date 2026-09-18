<?php

namespace App\Theme;

/**
 * Le contraste d'une couleur de thème — **mesuré, puis opposé** (S166).
 *
 * 🔴 **Une palette qui échoue est REFUSÉE, pas signalée.** C'est la mesure de
 * sortie de la session, et la différence est tout : un avertissement qu'on peut
 * ignorer se fait ignorer, et le texte illisible part en production avec l'aval
 * apparent de l'écran qui l'a laissé passer.
 *
 * ⚠️ **DEUX contrôles, et un piège qui aurait fait croire à trois.** Le contraste
 * WCAG est SYMÉTRIQUE : « blanc sur la couleur » et « la couleur sur blanc »
 * donnent exactement le même nombre. Les lister séparément aurait affiché deux
 * lignes toujours identiques, et donné l'illusion d'une vérification de plus.
 *
 * 🔴 **Le second contrôle n'est PAS redondant, et un exemple le prouve** : du
 * NOIR pur passe le premier avec 21:1 et échoue le second à 3,39:1 — sa variante
 * éclaircie devient un gris moyen, illisible sur le panneau sombre. Sans ce
 * contrôle, « noir » serait accepté comme couleur d'accent et casserait le thème
 * sombre de tout le site.
 *
 * ⚠️ **Le panneau ÉLEVÉ, pas le panneau de base.** `#342b41` est plus clair que
 * `#2b2335`, donc plus dur pour un accent clair : la marque y marque 5,11 contre
 * 5,75. On mesure sur la surface la plus défavorable — l'autre passerait toute
 * seule.
 *
 * 🅿️ **Ce que ce fichier ne sait pas** : si le texte est grand. WCAG autorise
 * 3:1 au-delà de 18,66 px gras ou 24 px. On applique 4,5:1 partout parce qu'un
 * accent sert surtout à des libellés de 13 px — et parce qu'un seuil qui dépend
 * du gabarit ne serait pas vérifiable ici.
 */
final class ContrastGate
{
    /** WCAG 2.1 AA, texte normal. */
    public const MINIMUM = 4.5;

    /** Le blanc du texte posé sur un bouton d'accent. */
    private const ON_PRIMARY = '#ffffff';

    /**
     * ⚠️ La surface sombre la plus CLAIRE, donc la plus défavorable pour un
     * accent éclairci. Elle vient de `style.css` ; si elle y change, ce nombre
     * doit suivre — d'où la note, plutôt qu'une constante muette.
     */
    private const DARK_SURFACE = '#342b41';

    /**
     * ⚠️ **Le même 50 % que `style.css`.** `--color-primary-text` y vaut
     * `color-mix(in srgb, var(--color-primary) 50%, white)` dans le thème
     * sombre ; mesurer autre chose reviendrait à contrôler une couleur que
     * personne n'affiche.
     */
    private const DARK_MIX = 0.5;

    /**
     * @return array{ok: bool, checks: list<array{key: string, ratio: float, needed: float, ok: bool}>}
     */
    public function check(string $hex): array
    {
        $hex = self::normalise($hex);
        if ($hex === null) {
            return ['ok' => false, 'checks' => []];
        }

        $checks = [
            [
                // Le même nombre que « la couleur sur blanc » — voir la note de
                // classe sur la symétrie.
                'key' => 'primary',
                'ratio' => self::ratio(self::ON_PRIMARY, $hex),
            ],
            [
                'key' => 'dark',
                'ratio' => self::ratio(self::lighten($hex, self::DARK_MIX), self::DARK_SURFACE),
            ],
        ];

        $ok = true;
        foreach ($checks as $i => $check) {
            $passed = $check['ratio'] >= self::MINIMUM;
            $checks[$i]['needed'] = self::MINIMUM;
            $checks[$i]['ok'] = $passed;
            $ok = $ok && $passed;
        }

        return ['ok' => $ok, 'checks' => $checks];
    }

    /** La variante éclaircie que le thème sombre affiche, pour l'aperçu. */
    public function darkVariant(string $hex): ?string
    {
        $hex = self::normalise($hex);

        return $hex === null ? null : self::lighten($hex, self::DARK_MIX);
    }

    /**
     * ⚠️ **Développe `#abc` en `#aabbcc`.** Le formulaire accepte les deux
     * formes ; mesurer la forme courte octet par octet donnerait une luminance
     * fausse, et donc un refus ou un laissez-passer arbitraire.
     */
    public static function normalise(string $hex): ?string
    {
        $hex = strtolower(trim($hex));
        if (preg_match('/^#([0-9a-f]{3}|[0-9a-f]{6})$/', $hex) !== 1) {
            return null;
        }

        if (strlen($hex) === 4) {
            $hex = '#' . $hex[1] . $hex[1] . $hex[2] . $hex[2] . $hex[3] . $hex[3];
        }

        return $hex;
    }

    /** Le rapport de contraste WCAG, de 1 à 21. Symétrique. */
    public static function ratio(string $a, string $b): float
    {
        $la = self::luminance($a);
        $lb = self::luminance($b);

        return (max($la, $lb) + 0.05) / (min($la, $lb) + 0.05);
    }

    /** `color-mix(in srgb, $hex $p%, white)` — en sRGB, comme le navigateur. */
    private static function lighten(string $hex, float $part): string
    {
        $out = '#';
        foreach (self::channels($hex) as $value) {
            $out .= str_pad(dechex((int) round($value * $part + 255 * (1 - $part))), 2, '0', STR_PAD_LEFT);
        }

        return $out;
    }

    /** Luminance relative WCAG. */
    private static function luminance(string $hex): float
    {
        [$r, $g, $b] = array_map(static function (int $value): float {
            $c = $value / 255;

            return $c <= 0.03928 ? $c / 12.92 : (($c + 0.055) / 1.055) ** 2.4;
        }, self::channels($hex));

        return 0.2126 * $r + 0.7152 * $g + 0.0722 * $b;
    }

    /** @return array{0: int, 1: int, 2: int} */
    private static function channels(string $hex): array
    {
        $hex = self::normalise($hex) ?? '#000000';

        return [
            (int) hexdec(substr($hex, 1, 2)),
            (int) hexdec(substr($hex, 3, 2)),
            (int) hexdec(substr($hex, 5, 2)),
        ];
    }
}
