<?php

namespace App\Twig;

use App\Media\SiteMediaLibrary;
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
    ) {
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
            new TwigFunction('portal_primary_color', $this->primaryColor(...)),
        ];
    }

    public function name(): string
    {
        return $this->settings->getOrgName();
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
        $value = trim((string) $this->settings->get('site_logo_path'));

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
        $value = trim((string) $this->settings->get('site_primary_color'));

        return preg_match('/^#(?:[0-9a-f]{3}|[0-9a-f]{6})$/i', $value) === 1 ? $value : null;
    }
}
