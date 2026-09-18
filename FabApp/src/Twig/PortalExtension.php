<?php

namespace App\Twig;

use App\Media\SiteMediaLibrary;
use App\Service\ThemeManager;
use App\Theme\ContrastGate;
use App\Theme\ThemePreview;
use App\Service\SiteSettingService;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * The branding the current portal asks for, if any.
 *
 * These read through `SiteSettingService::get()`, which already resolves
 * most-specific-first, so a portal's row wins for a request served on its
 * hostname and everything else falls back to the site-wide value. Null means
 * "nothing overridden" and every caller renders exactly what it rendered before.
 */
final class PortalExtension extends AbstractExtension
{
    public function __construct(
        private readonly SiteSettingService $settings,
        private readonly SiteMediaLibrary $media,
        private readonly ThemeManager $themes,
        private readonly ThemePreview $preview,
        private readonly ContrastGate $contrast,
    ) {
    }

    /**
     * La valeur publiée — ou celle du BROUILLON quand on prévisualise (S167).
     *
     * ⚠️ **Un seul endroit décide**, pour les trois fonctions. Dupliquer le test
     * aurait fini par donner un aperçu où le logo suit le brouillon et la couleur
     * non, ce qui est exactement le genre d'aperçu qui ment.
     */
    private function value(string $settingKey, string $draftKey): string
    {
        if ($this->preview->isPreviewing()) {
            return trim((string) ($this->themes->draft()[$draftKey] ?? ''));
        }

        return trim((string) $this->settings->get($settingKey));
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('portal_name', $this->name(...)),
            // 🔴 **`site_logo` depuis S165, et `portal_logo_path` est SUPPRIMÉ.**
            // L'ancien nom renvoyait à un écran « Portails » qui n'existe plus,
            // et il rendait un NOM DE FICHIER que l'appelant devait préfixer
            // lui-même — donc un chemin construit dans un gabarit. Le nouveau
            // rend le chemin public complet, ou `null`. Garder un alias aurait
            // laissé les deux vocabulaires cohabiter sans que rien ne tranche.
            new TwigFunction('site_logo', $this->siteLogo(...)),
            // ⚠️ Même forme que `site_logo()` : un chemin public, ou `null` pour
            // « rends celle livrée ». Un second vocabulaire pour la même idée
            // finirait par diverger.
            new TwigFunction('site_favicon', $this->siteFavicon(...)),
            new TwigFunction('portal_primary_color', $this->primaryColor(...)),
            /*
             * 🔴 **S166b — le jeton de TEXTE d'accent, calculé en PHP.** En thème
             * sombre, `style.css` le dérive avec `color-mix()` et garde un repli
             * statique qui, lui, ne peut pas suivre un thème : un labo qui change
             * sa couleur gardait l'accent de FabOS sur un moteur sans
             * `color-mix()`. Émis d'ici, il suit la palette partout.
             */
            new TwigFunction('portal_primary_text', $this->primaryText(...)),
            // ⚠️ Rend `null` hors aperçu : hors de la grille, personne n'impose
            // un thème à personne.
            new TwigFunction('theme_forced_mode', $this->preview->forcedMode(...)),
        ];
    }

    public function name(): string
    {
        $preview = $this->preview->isPreviewing() ? trim((string) ($this->themes->draft()['orgName'] ?? '')) : '';

        return $preview !== '' ? $preview : $this->settings->getOrgName();
    }

    /**
     * La variante d'accent LISIBLE sur le panneau sombre, ou `null`.
     *
     * ⚠️ `null` quand aucune couleur n'est choisie : `style.css` garde alors ses
     * deux déclarations, qui sont justes pour la marque livrée. Émettre quelque
     * chose ici dans ce cas ne changerait rien et ferait un `<style>` de plus sur
     * chaque page.
     */
    public function primaryText(): ?string
    {
        $value = $this->primaryColor();

        return $value === null ? null : $this->contrast->darkVariant($value);
    }

    /**
     * Le chemin public du logo choisi dans la médiathèque, ou `null`.
     *
     * 🔴 **Plus de chemin libre.** Le réglage porte un `mediaId` ; le nom du
     * fichier vient de la table, pas de la valeur. Un `../../.env` n'a donc plus
     * d'endroit où atterrir : il ne ressemble pas à un mediaId, et même s'il en
     * portait la forme, il ne correspondrait à aucune ligne.
     *
     * ⚠️ **`null` veut dire « rien de choisi », et c'est le cas normal** — le
     * gabarit rend alors le logo livré. Il veut dire la même chose quand la
     * médiathèque n'est pas encore migrée : dans les deux cas le site s'habille
     * comme avant, ce qui est la seule réponse acceptable.
     */
    public function siteLogo(): ?string
    {
        $value = $this->value('site_logo_path', 'logoPath');

        return $value === '' ? null : $this->media->assetPath($value);
    }

    /** Le chemin public de l'icône d'onglet choisie, ou `null`. */
    public function siteFavicon(): ?string
    {
        $value = $this->value('site_favicon_path', 'faviconPath');

        return $value === '' ? null : $this->media->assetPath($value);
    }

    /**
     * A hex colour, or null.
     *
     * ⚠️ **Validated on read, not only on save.** This value is interpolated into
     * a `<style>` block, where Twig's HTML escaping does nothing useful — inside
     * CSS, `}` ends the rule and anything after it is more CSS. Checking the
     * shape here means a row written by an older version, a direct SQL edit or a
     * future form that forgets to validate still cannot inject stylesheet.
     */
    public function primaryColor(): ?string
    {
        $value = $this->value('site_primary_color', 'primaryColor');

        return preg_match('/^#(?:[0-9a-f]{3}|[0-9a-f]{6})$/i', $value) === 1 ? $value : null;
    }
}
