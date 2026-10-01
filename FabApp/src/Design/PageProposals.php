<?php

declare(strict_types=1);

namespace App\Design;

/**
 * Les pages revues d'après les planches de référence, proposées à l'opérateur
 * AVANT d'être construites (demande du 2026-10-01 : « des sous-pages dans le
 * menu dev », pas des branches).
 *
 * Une proposition = une entrée ici + un gabarit `site/proposals/{slug}.html.twig`.
 * Le menu Développement en fait une sous-page chacune ; l'index
 * (`/admin/propositions`) rend le tri complet des planches
 * (`docs/references/PROPOSITIONS.md`).
 *
 * ⚠️ Une proposition ne copie pas la planche : elle en garde les bonnes idées
 * (contenu, ordre, mise en page) et les pose sur NOS composants et NOS jetons.
 * `take` = ce qu'on reprend de la planche ; `keep` = ce qu'on garde de chez nous
 * parce que c'est mieux.
 *
 * 🔴 **Retenue ou refusée, une proposition se SUPPRIME** — entrée, gabarit, CSS
 * `pp-*` qui ne sert qu'à elle. Le raisonnement va dans `HISTORY.md`.
 */
final class PageProposals
{
    /**
     * @return list<array{slug: string, title: string, planche: string, current: array{route: string, params: array<string, mixed>, label: string}, shell: string, take: list<string>, keep: list<string>}>
     */
    public function all(): array
    {
        return [
            [
                'slug' => 'espaces-catalogue',
                'title' => 'Catalogue des espaces',
                'planche' => 'espaces/01-catalogue-espaces.png',
                'current' => ['route' => 'app_places', 'params' => [], 'label' => '/places'],
                'shell' => 'public',
                'take' => [
                    'Un bandeau « Libre maintenant » en tête : ce qu’on peut réserver tout de suite, sans parcourir la grille.',
                    'Des cartes qui portent le verbe : un vrai bouton « Réserver » quand vous le pouvez, au lieu d’un petit lien « Voir ».',
                    'Capacité et emplacement lisibles d’un coup d’œil, chacun avec son pictogramme.',
                ],
                'keep' => [
                    'La séparation par POSITION : la tête dit l’état de la salle, le pied dit votre droit. La planche met « Indisponible » sur un bouton sans dire si c’est la salle ou vous.',
                    'Une image par défaut propre à un espace (le plan), plus l’icône d’imprimante empruntée aux machines.',
                ],
            ],
        ];
    }

    /** @return array{slug: string, title: string, planche: string, current: array{route: string, params: array<string, mixed>, label: string}, shell: string, take: list<string>, keep: list<string>}|null */
    public function find(string $slug): ?array
    {
        foreach ($this->all() as $proposal) {
            if ($proposal['slug'] === $slug) {
                return $proposal;
            }
        }

        return null;
    }
}
