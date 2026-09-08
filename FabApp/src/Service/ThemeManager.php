<?php

namespace App\Service;

use App\Media\SiteMediaLibrary;

/**
 * Le brouillon de thème, son aperçu et sa publication.
 *
 * 🔴 **`logoPath` porte désormais un `mediaId`, plus un nom de fichier (S165).**
 * Avant, c'était une chaîne libre désignant un fichier qui devait DÉJÀ se trouver
 * dans `public/images/` : poser un logo demandait un accès au serveur, ce qui
 * n'est pas un thème, c'est un déploiement.
 * 🅿️ **Aucune branche de compatibilité, et c'est mesuré** : le 2026-09-08,
 * `SITE_SETTING` ne contient aucune ligne `site_logo_path` et le brouillon porte
 * `logoPath: ""`. Il n'existe pas une seule valeur héritée à convertir — écrire
 * la branche aurait été écrire du code sans cas d'usage, puis le maintenir.
 */
final class ThemeManager
{
    private const DRAFT_KEY = 'theme_draft_v1';

    public function __construct(private readonly SiteSettingService $settings) {}

    /** @return array{orgName: string, venueLabel: string, primaryColor: string, logoPath: string} */
    public function published(): array
    {
        return [
            'orgName' => $this->settings->getOrgName(),
            'venueLabel' => $this->settings->getVenueLabel(),
            'primaryColor' => $this->settings->get('site_primary_color') ?? '',
            'logoPath' => $this->settings->get('site_logo_path') ?? '',
        ];
    }

    public function draft(): array
    {
        $decoded = json_decode($this->settings->get(self::DRAFT_KEY) ?? '', true);

        return is_array($decoded) ? $decoded + $this->published() : $this->published();
    }

    /** @param array<string, mixed> $input */
    public function saveDraft(array $input): array
    {
        $draft = [
            'orgName' => mb_substr(trim((string) ($input['orgName'] ?? '')), 0, 80),
            'venueLabel' => mb_substr(trim((string) ($input['venueLabel'] ?? '')), 0, 80),
            'primaryColor' => trim((string) ($input['primaryColor'] ?? '')),
            'logoPath' => trim((string) ($input['logoPath'] ?? '')),
        ];
        if ($draft['orgName'] === '' || $draft['venueLabel'] === '') {
            throw new \InvalidArgumentException('Les deux noms publics sont obligatoires.');
        }
        if ($draft['primaryColor'] !== '' && preg_match('/^#(?:[0-9a-f]{3}|[0-9a-f]{6})$/i', $draft['primaryColor']) !== 1) {
            throw new \InvalidArgumentException('La couleur doit être un code hexadécimal, par exemple #9E1B56.');
        }
        // 🔴 **Un identifiant de médiathèque, pas un chemin.** La forme est
        // vérifiée ici même si l'écran ne propose qu'une liste : ce point de
        // passage est aussi celui d'un import ou d'une commande, et une valeur
        // qui atteindrait `asset()` sans être un mediaId serait un chemin libre
        // de retour.
        if ($draft['logoPath'] !== '' && !SiteMediaLibrary::isMediaId($draft['logoPath'])) {
            throw new \InvalidArgumentException('Le logo doit être choisi dans la médiathèque.');
        }
        $this->settings->set(self::DRAFT_KEY, json_encode($draft, JSON_THROW_ON_ERROR));

        return $draft;
    }

    public function publish(): void
    {
        $draft = $this->draft();
        $this->settings->set('org_name', $draft['orgName']);
        $this->settings->set('venue_label', $draft['venueLabel']);
        $this->settings->set('site_primary_color', $draft['primaryColor']);
        $this->settings->set('site_logo_path', $draft['logoPath']);
    }

    /**
     * Les images qu'un thème utilise — publiées **ET** en brouillon.
     *
     * 🔴 **Le brouillon compte autant que le publié.** Supprimer l'image qu'un
     * brouillon référence laisserait la publication suivante poser un logo qui
     * n'existe plus : un site à moitié rhabillé, découvert par les visiteurs.
     *
     * @return list<string>
     */
    public function referencedMediaIds(): array
    {
        $ids = [];
        foreach ([$this->published()['logoPath'] ?? '', $this->draft()['logoPath'] ?? ''] as $value) {
            $value = trim((string) $value);
            if ($value !== '' && !in_array($value, $ids, true)) {
                $ids[] = $value;
            }
        }

        return $ids;
    }

    public function discardDraft(): void
    {
        $this->settings->set(self::DRAFT_KEY, json_encode($this->published(), JSON_THROW_ON_ERROR));
    }
}
