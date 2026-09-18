<?php

namespace App\Theme;

use Symfony\Bundle\SecurityBundle\Security;
use Symfony\Component\HttpFoundation\RequestStack;

/**
 * « Rends cette page comme si le brouillon de thème était publié » (S167).
 *
 * 🔴 **L'aperçu rend les VRAIES pages, pas des vignettes dessinées à la main.**
 * C'est la mesure de sortie de la session, et c'est la leçon de
 * [[feedback-fabos-verify-pixels]] : une vignette qu'on dessine à côté prouve
 * qu'on sait dessiner une vignette. Elle ne dit pas si le badigeon `!important`
 * de `style.css` repeint l'accent, ni si un jeton est lu là où on croit. Seule la
 * page elle-même le dit.
 *
 * 🔴 **RÉSERVÉ AUX ADMINISTRATEURS, et vérifié à chaque lecture.** Un brouillon
 * est du travail non publié : un paramètre d'URL qui le montrerait à un visiteur
 * publierait le thème par accident, et à l'insu de celui qui l'a saisi.
 * ⚠️ Le contrôle est fait ICI et pas dans le contrôleur qui affiche la grille :
 * ce sont les pages APERÇUES — l'accueil, le catalogue — qui liraient le
 * brouillon, et aucune d'elles n'appartient à l'administration.
 *
 * ⚠️ **Rien n'est écrit, rien n'est mis en session.** Le mode ne vit que dans
 * l'URL de la requête en cours. Un drapeau en session survivrait à la fermeture
 * de l'aperçu et montrerait un thème non publié à son auteur pendant des heures,
 * sans rien qui l'explique.
 */
final class ThemePreview
{
    public const PARAM = 'theme';
    public const MODE_PARAM = 'theme_mode';
    private const VALUE = 'draft';

    public function __construct(
        private readonly RequestStack $requests,
        private readonly Security $security,
    ) {
    }

    /** Le brouillon doit-il remplacer le publié, pour CETTE requête ? */
    public function isPreviewing(): bool
    {
        $request = $this->requests->getCurrentRequest();

        return $request !== null
            && $request->query->get(self::PARAM) === self::VALUE
            && $this->security->isGranted('ROLE_ADMIN');
    }

    /**
     * Le thème clair ou sombre imposé à la page, ou `null`.
     *
     * 🔴 **Sans ça, la moitié sombre de l'aperçu est un mensonge.** Le thème du
     * site est appliqué par `main.js` depuis `localStorage` : quatre cadres dans
     * la même page partagent le même stockage, donc afficheraient tous le même
     * thème. Il faut donc l'imposer par l'URL, côté serveur, et le VERROUILLER
     * pour que le script ne le réécrive pas au chargement.
     *
     * ⚠️ Le mode n'a de sens qu'en aperçu : hors aperçu il est ignoré, sinon un
     * lien partagé imposerait un thème à qui le reçoit.
     */
    public function forcedMode(): ?string
    {
        if (!$this->isPreviewing()) {
            return null;
        }

        $mode = (string) $this->requests->getCurrentRequest()?->query->get(self::MODE_PARAM);

        return in_array($mode, ['light', 'dark'], true) ? $mode : null;
    }
}
