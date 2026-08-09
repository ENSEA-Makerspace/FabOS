<?php

namespace App\Feature;

/**
 * Target metadata for the post-S102 operator workspaces.
 *
 * This registry is deliberately descriptive in S103: it drives development
 * prototypes and audit coverage, but it does not grant access. Voters, services
 * and feature adapters remain the enforcement points when S110 activates the
 * new model.
 */
final class FeatureWorkspaceRegistry
{
    /** @var array<string, list<array{label: string, route: string, matches?: list<string>}>> */
    private const TABS = [
        'equipment' => [
            ['label' => 'Machines', 'route' => 'app_admin_machines', 'matches' => ['app_admin_machines_scoped_html', 'app_admin_machines_double_legacy_html']],
            ['label' => 'Catégories', 'route' => 'app_admin_machine_categories'],
            ['label' => 'Modèles & marques', 'route' => 'app_admin_machine_models'],
            ['label' => 'Matériaux', 'route' => 'app_admin_materials'],
            ['label' => 'Maintenance', 'route' => 'app_admin_maintenance'],
            ['label' => 'Réservations', 'route' => 'app_admin_reservations', 'params' => ['reservableType' => 'machine']],
        ],
        'events' => [['label' => 'Événements', 'route' => 'app_admin_events']],
        'loans' => [
            ['label' => 'Objets', 'route' => 'app_admin_loanable_items'],
            ['label' => 'Prêts', 'route' => 'app_admin_loans'],
        ],
        'spaces' => [
            ['label' => 'Espaces', 'route' => 'app_admin_places'],
            ['label' => 'Réservations', 'route' => 'app_admin_reservations', 'params' => ['reservableType' => 'place']],
        ],
        'training' => [['label' => 'Formations', 'route' => 'app_admin_formations', 'matches' => ['app_admin_formations_scoped_html', 'app_admin_formations_double_legacy_html']]],
        'badges' => [['label' => 'Badges', 'route' => 'app_admin_badges']],
        'projects' => [['label' => 'Projets', 'route' => 'app_admin_creations']],
        'pages' => [['label' => 'Pages', 'route' => 'app_admin_lab_pages']],
        'users' => [['label' => 'Utilisateurs', 'route' => 'app_admin_users', 'matches' => ['app_admin_users_scoped_html', 'app_admin_users_double_legacy_html']]],
        'locations' => [['label' => 'Horaires', 'route' => 'app_admin_opening_hours']],
        'packages' => [['label' => 'Packages', 'route' => 'app_admin_usage_rights']],
        'network' => [['label' => 'Institutions', 'route' => 'app_admin_institutions']],
        'configuration' => [
            ['label' => 'Fonctionnalités', 'route' => 'app_admin_features'],
            ['label' => 'Thèmes', 'route' => 'app_admin_themes', 'matches' => ['app_admin_homepage']],
            ['label' => 'Réglages', 'route' => 'app_admin_settings'],
            ['label' => 'E-mails', 'route' => 'app_admin_emails'],
        ],
    ];

    /** @return list<array<string, mixed>> */
    public function all(): array
    {
        return [
            $this->workspace('equipment', 'Équipement', 'machines', ['Machines', 'Catégories', 'Modèles & marques', 'Matériaux', 'Maintenance', 'Réservations'], ['sous-lieu', 'catégorie', 'machine'], ['état', 'disponibilité', 'catégorie'], [
                $this->route('app_machines', 'GET', 'Use', 'machines.view'),
                $this->route('app_admin_machines', 'GET', 'Manage', 'machines.manage'),
                $this->route('app_admin_machine_edit', 'GET|POST', 'Manage', 'machines.update'),
            ], true, true, true),
            $this->workspace('events', 'Événements', 'events', ['Événements', 'Organisateurs', 'Lieux', 'Inscriptions'], ['sous-lieu', 'organisateur', 'lieu'], ['date', 'type', 'organisateur'], [
                $this->route('app_events', 'GET', 'Use', 'events.view'),
                $this->route('app_event_register', 'POST', 'Use', 'events.register'),
                $this->route('app_admin_event_edit', 'GET|POST', 'Manage', 'events.update'),
            ], false, true, true),
            $this->workspace('loans', 'Prêts', 'loans', ['Objets', 'Catégories', 'Prêts'], ['sous-lieu', 'catégorie', 'objet'], ['état', 'catégorie', 'emprunteur'], [
                $this->route('app_loans', 'GET', 'Use', 'loans.view'),
                $this->route('app_admin_loan_new', 'GET|POST', 'Manage', 'loans.create'),
            ], false, true, true),
            $this->workspace('spaces', 'Espaces', 'places', ['Espaces', 'Catégories', 'Réservations'], ['sous-lieu', 'catégorie', 'responsable', 'département'], ['disponibilité', 'catégorie', 'responsable'], [
                $this->route('app_places', 'GET', 'Use', 'places.view'),
                $this->route('app_admin_place_edit', 'GET|POST', 'Manage', 'places.update'),
            ], true, true, true),
            $this->workspace('training', 'Formations', 'formations', ['Formations', 'Catégories', 'Sessions & progression', 'Formateurs'], ['catégorie', 'département', 'institution', 'formateur', 'sous-lieu pour une session physique'], ['catégorie', 'niveau', 'formateur'], [
                $this->route('app_formations', 'GET', 'Use', 'training.view'),
                $this->route('app_admin_formation_edit', 'GET|POST', 'Manage', 'training.update'),
            ], false, false, true),
            $this->workspace('badges', 'Badges', 'badges', ['Badges', 'Attributions', 'Reconnaissances & émetteurs', 'Journal'], ['global FabOS'], ['état', 'émetteur', 'formation'], [
                $this->route('app_badges', 'GET', 'Use', 'badges.view'),
                $this->route('app_admin_badge_edit', 'GET|POST', 'Manage', 'badges.update'),
            ]),
            $this->workspace('projects', 'Galerie de projets', 'projects', ['Projets', 'Modération'], ['politique explicite globale ou sous-lieu'], ['publication', 'auteur', 'catégorie'], [
                $this->route('app_creations', 'GET', 'Use', 'projects.view'),
                $this->route('app_admin_creation_edit', 'GET|POST', 'Manage', 'projects.moderate'),
            ], false, false, true),
            $this->workspace('pages', 'Pages personnalisées', 'lab_pages', ['Pages', 'Navigation', 'Publication'], ['global FabOS'], ['publication', 'niveau de navigation'], [
                $this->route('app_lab_pages', 'GET', 'Use', 'pages.view'),
                $this->route('app_admin_lab_page_edit', 'GET|POST', 'Manage', 'pages.update'),
            ]),
            $this->workspace('users', 'Utilisateurs', null, ['Utilisateurs', 'Groupes', 'Profils publics', 'Échanges QR'], ['catégorie utilisateur'], ['état', 'groupe', 'catégorie'], [
                $this->route('app_admin_users', 'GET', 'Manage', 'users.view'),
                $this->route('app_admin_user_detail', 'GET', 'Manage', 'users.view'),
            ], false, false, true),
            $this->workspace('locations', 'Lieux', null, ['Sous-lieux', 'Horaires'], ['sous-lieu'], ['état', 'sous-lieu'], [
                $this->route('app_admin_opening_hours', 'GET|POST', 'Manage', 'locations.hours.update'),
            ]),
            $this->workspace('packages', 'Packages', null, ['Packages', 'Attributions', 'Quotas', 'Audit'], ['sous-lieu dans les grants'], ['état', 'source', 'bénéficiaire'], [
                $this->route('app_admin_usage_rights', 'GET', 'Manage', 'packages.view'),
                $this->route('app_admin_usage_rights_edit', 'GET|POST', 'Manage', 'packages.update'),
            ], false, true),
            $this->workspace('network', 'Réseau FabOS', null, ['Connexions', 'Synchronisations', 'Journal'], ['instance distante', 'objet partagé'], ['confiance', 'état de synchronisation'], []),
            $this->workspace('configuration', 'Configuration', null, ['État installation', 'Fonctionnalités', 'Thèmes', 'Réglages', 'E-mails', 'Initialisation', 'Développement'], ['global FabOS'], ['état', 'type'], [
                $this->route('app_admin_features', 'GET|POST', 'Manage', 'configuration.features.update'),
                $this->route('app_admin_settings', 'GET|POST', 'Manage', 'configuration.settings.update'),
                $this->route('app_admin_homepage', 'GET|POST', 'Manage', 'themes.draft.update'),
            ]),
        ];
    }

    /** @return array<string, mixed> */
    public function themeContract(): array
    {
        return [
            'workspace' => 'Configuration → Thèmes',
            'domains' => ['Nom public', 'Logos & images', 'Couleurs', 'Menus', 'Blocs de la page d’accueil'],
            'previews' => ['ordinateur', 'mobile', 'clair', 'sombre'],
            'workflow' => ['Enregistrer le brouillon', 'Prévisualiser', 'Publier', 'Revenir à la version publiée'],
            'stableReferences' => 'Les menus référencent les clés du registre, jamais des routes ou du HTML libre.',
        ];
    }

    /** @return array<string, mixed>|null */
    public function forRoute(?string $route, ?string $preferredWorkspace = null): ?array
    {
        if ($route === null || $route === '') {
            return null;
        }

        foreach ($this->all() as $workspace) {
            if ($preferredWorkspace !== null && $workspace['key'] !== $preferredWorkspace) {
                continue;
            }
            $tabs = self::TABS[$workspace['key']] ?? [];
            foreach ($tabs as $tab) {
                if ($route === $tab['route'] || in_array($route, $tab['matches'] ?? [], true)) {
                    return $workspace + ['tabs' => $this->markActiveTab($tabs, $route)];
                }
            }
        }

        return null;
    }

    /** @param list<array{label: string, route: string, matches?: list<string>}> $tabs */
    private function markActiveTab(array $tabs, string $route): array
    {
        return array_map(static function (array $tab) use ($route): array {
            $tab['active'] = $route === $tab['route'] || in_array($route, $tab['matches'] ?? [], true);
            unset($tab['matches']);

            return $tab;
        }, $tabs);
    }

    /** @return array<string, mixed> */
    private function workspace(string $key, string $label, ?string $featureGate, array $sections, array $scopes, array $filters, array $routes, bool $reservations = false, bool $quotas = false, bool $reporting = false): array
    {
        return compact('key', 'label', 'featureGate', 'sections', 'scopes', 'filters', 'routes', 'reservations', 'quotas', 'reporting') + ['actions' => ['Use', 'Manage']];
    }

    /** @return array{name: string, methods: string, right: string, capability: string} */
    private function route(string $name, string $methods, string $right, string $capability): array
    {
        return compact('name', 'methods', 'right', 'capability');
    }
}
