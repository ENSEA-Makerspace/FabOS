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
 * `demo` = des liens de démonstration (paramètres lus par la seule proposition).
 * `take` = ce qu'on reprend de la planche ; `keep` = ce qu'on garde de chez nous
 * parce que c'est mieux.
 *
 * 🔴 **Retenue ou refusée, une proposition se SUPPRIME** — entrée, gabarit, CSS
 * `pp-*` qui ne sert qu'à elle. Le raisonnement va dans `HISTORY.md`.
 */
final class PageProposals
{
    /**
     * @return list<array{slug: string, title: string, planche: string, current: array{route: string, params: array<string, mixed>, label: string}, shell: string, take: list<string>, keep: list<string>, demo?: array<string, string>}>
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
                'demo' => ['?vue=ouvert' => 'labo ouvert'],
            ],
            [
                'slug' => 'mes-reservations',
                'title' => 'Mes réservations',
                'planche' => 'espaces/08-mes-reservations.png',
                'current' => ['route' => 'app_my_reservations', 'params' => [], 'label' => '/mes-reservations'],
                'shell' => 'public',
                'take' => [
                    'LA prochaine réservation (ou celle en cours) en tête : quand (« Aujourd’hui · 14:00–17:00 »), quoi, et pour un espace, la fenêtre où votre badge ouvre la porte.',
                    'Le reste en lignes datées — « À venir », puis « Passées » estompées : une réservation se lit par sa date, pas par une affiche.',
                    'Un seul verbe par ligne (« Gérer »).',
                ],
                'keep' => [
                    'Le détail d’une réservation (déplacer, annuler, rétablir) reste sur sa page : pas de mur de boutons dans la liste.',
                    'Les annulées et refusées restent consultables, repliées en bas — la planche les oublie.',
                ],
                'demo' => ['?membre=45' => 'avec les réservations d’alice'],
            ],
            [
                'slug' => 'accueil-membre',
                'title' => 'Accueil d’un membre connecté',
                'planche' => 'users/04-accueil-membre.png',
                'current' => ['route' => 'app_home', 'params' => [], 'label' => '/'],
                'shell' => 'public',
                'take' => [
                    '« À faire » en tête : ce qui bloque ou attend (objet à rendre, formation à terminer, demande en attente, premier badge), chacun avec le verbe qui le règle.',
                    '« Ma prochaine réservation » à un clic, et « Mes accès » à côté.',
                ],
                'keep' => [
                    '« Mes accès » est le composant du profil (`_badge_reach`), pas une copie : ce que le badge ouvre se dit à un seul endroit.',
                    'L’accueil actuel (événements, horaires, personnalisation) reste dessous, inchangé. La planche n’a que le bloc personnel.',
                    'Pas d’« adhésion valide jusqu’au » : FabOS n’a pas d’adhésion payante.',
                ],
                'demo' => ['?membre=45' => 'alice', '?membre=55' => 'frank'],
            ],
        ];
    }

    /** @return array{slug: string, title: string, planche: string, current: array{route: string, params: array<string, mixed>, label: string}, shell: string, take: list<string>, keep: list<string>, demo?: array<string, string>}|null */
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
