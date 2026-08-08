<?php

namespace App\Nav;

use App\Feature\SiteFeatureService;
use App\Security\RouteAccessChecker;
use Symfony\Component\HttpFoundation\RequestStack;
use App\Repository\LabPageRepository;
use App\Service\SiteSettingService;
use Symfony\Bundle\SecurityBundle\Security;

/**
 * The site navigation, assembled from what is actually switched on.
 *
 * The header used to be fifteen hand-written `feature_enabled()` checks wrapped
 * around hardcoded markup, which meant every new feature was a template edit and
 * an all-disabled group still rendered its wrapper. The rules it encoded were
 * sound; they were just written fifteen times.
 *
 * Three of them are worth stating once, because they are the ones that go wrong:
 *
 *  1. **A group with no visible children is not rendered at all** — not as an
 *     empty dropdown, and not as a heading linking to a page that 404s.
 *  2. **A group's own link follows its children.** Where the natural landing page
 *     is gone, the heading points at the first child that is still there rather
 *     than at a dead route.
 *  3. **Visibility is presentation, never permission.** Hiding an entry hides a
 *     link; the route gate and the firewall decide what actually exists. Nothing
 *     here may be the only thing standing between a user and a page.
 */
final class NavBuilder
{
    public function __construct(
        private readonly SiteFeatureService $features,
        private readonly Security $security,
        private readonly LabPageRepository $labPages,
        private readonly SiteSettingService $settings,
        private readonly RouteAccessChecker $access,
        private readonly RequestStack $requests,
    ) {
    }

    /**
     * @return list<array{kind: string, label: string, translate: bool, route: ?string, items: list<array{label: string, translate: bool, route: string, params: array<string, mixed>, child: bool}>}>
     */
    public function header(): array
    {
        $nav = [];

        $nav[] = $this->link('nav.home', 'app_home');

        // The calendar group keeps its own landing rule: the calendar page itself
        // whenever anything is drawn on the grid, even if the first child listed
        // (equipment) is the thing that is off.
        $nav[] = $this->group(
            'nav.calendar',
            $this->features->hasCalendarLayer() ? 'app_calendar' : null,
            [
                $this->item('cal.book_machine', 'app_calendar', feature: 'machines'),
                $this->item('cal.book_space', 'app_places', feature: 'places'),
                $this->item('nav.events', 'app_events', feature: 'events'),
            ],
        );

        $nav[] = $this->group(
            $this->settings->getLabPagesNavLabel(),
            $this->features->isEnabled('lab_pages') ? 'app_lab_pages' : null,
            [
                $this->item('nav.machines', 'app_machines', feature: 'machines'),
                $this->item('nav.places', 'app_places', feature: 'places'),
                $this->item('nav.materials', 'app_materials', feature: 'materials'),
                $this->item('nav.loans', 'app_loans', feature: 'loans'),
                $this->item('nav.maintenance', 'app_maintenance', feature: 'maintenance'),
                $this->item('nav.staff', 'app_staff', feature: 'staff'),
                $this->item('nav.trainers', 'app_trainers', feature: 'trainers'),
                ...$this->labPageItems(),
            ],
            translateLabel: false,
        );

        $nav[] = $this->group('nav.learn', null, [
            $this->item('nav.trainings', 'app_formations', feature: 'formations'),
            $this->item('nav.badges', 'app_badges', feature: 'badges'),
        ]);

        $nav[] = $this->group('nav.activity', null, [
            $this->item('nav.leaderboard', 'app_leaderboard', feature: 'leaderboard'),
            $this->item('nav.projects', 'app_creations', feature: 'projects'),
        ]);

        // "Mes réservations" and "Mes disponibilités" are personal pages, not places
        // this site has — they belong with the rest of someone's own account rather
        // than beside the machines and the training catalogue. Both are linked from
        // the reservations section of `/profil`, which is now the only way in.
        $nav[] = $this->link('nav.admin', 'app_admin_dashboard', role: 'ROLE_ADMIN');

        // Staff who aren't admin still need the pass desk; admins reach it from the
        // admin sidebar, so only show it here for non-admin staff.
        $nav[] = $this->link(
            'nav.staff_passes',
            'app_staff_access_passes',
            visible: $this->security->isGranted('ROLE_STAFF') && !$this->security->isGranted('ROLE_ADMIN'),
        );

        return array_values(array_filter($nav));
    }

    /**
     * The footer is a flat list of the same entries — no groups, no fallbacks.
     *
     * @return list<array{label: string, translate: bool, route: string, params: array<string, mixed>, child: bool}>
     */
    public function footer(): array
    {
        $items = [
            $this->item('nav.home', 'app_home'),
            $this->item('nav.machines', 'app_machines', feature: 'machines'),
            $this->item('nav.calendar', 'app_calendar', visible: $this->features->hasCalendarLayer()),
            $this->item('nav.trainings', 'app_formations', feature: 'formations'),
            $this->item('nav.leaderboard', 'app_leaderboard', feature: 'leaderboard'),
            $this->item('nav.projects', 'app_creations', feature: 'projects'),
            $this->item('nav.badges', 'app_badges', feature: 'badges'),
            $this->item($this->settings->getLabPagesNavLabel(), 'app_lab_pages', feature: 'lab_pages', translate: false),
            $this->item('nav.places', 'app_places', feature: 'places'),
            $this->item('nav.materials', 'app_materials', feature: 'materials'),
            $this->item('nav.loans', 'app_loans', feature: 'loans'),
            $this->item('nav.maintenance', 'app_maintenance', feature: 'maintenance'),
            $this->item('nav.staff', 'app_staff', feature: 'staff'),
            $this->item('nav.trainers', 'app_trainers', feature: 'trainers'),
            $this->item('nav.events', 'app_events', feature: 'events'),
        ];

        return array_values(array_filter($items));
    }

    /**
     * A few places that certainly still exist — for the error pages.
     *
     * The 404 page used to offer equipment and training as its way out, both
     * hardcoded. On an install where those are switched off, every escape route
     * from the 404 was itself a 404. Reading the footer instead means the
     * suggestions inherit the same rule as the menu: nothing that is off is
     * offered.
     *
     * Wrapped, because an error page that throws while explaining an error is
     * the one failure mode that has nowhere left to go: with no navigation
     * available it falls back to the home link alone.
     *
     * @return list<array{label: string, translate: bool, route: string, params: array<string, mixed>}>
     */
    public function safeDestinations(int $limit = 3): array
    {
        $home = ['label' => 'nav.home', 'translate' => true, 'route' => 'app_home', 'params' => []];

        try {
            $items = $this->footer();
        } catch (\Throwable) {
            return [$home];
        }

        $destinations = [$home];
        foreach ($items as $item) {
            if ($item['route'] === 'app_home' || \count($destinations) > $limit) {
                continue;
            }
            $destinations[] = ['label' => $item['label'], 'translate' => $item['translate'], 'route' => $item['route'], 'params' => $item['params']];
        }

        return $destinations;
    }

    /** @return list<array<string, mixed>> */
    private function labPageItems(): array
    {
        if (!$this->features->isEnabled('lab_pages')) {
            return [];
        }

        try {
            $pages = $this->labPages->findTopLevelWithChildren();
        } catch (\Throwable) {
            // Fail-safe like everywhere else: a nav that cannot load its pages
            // renders without them rather than taking the whole site down.
            return [];
        }

        $items = [];
        foreach ($pages as $page) {
            $items[] = $this->item((string) $page->getTitre(), 'app_lab_page', params: ['id' => $page->getId()], translate: false);
            foreach ($page->getChildren() as $child) {
                $items[] = $this->item((string) $child->getTitre(), 'app_lab_page', params: ['id' => $child->getId()], translate: false, child: true);
            }
        }

        return array_values(array_filter($items));
    }

    /** @return array<string, mixed>|null */
    /**
     * The admin sidebar — **grouped by feature**, and built here rather than in
     * the template.
     *
     * **Why it moved.** It was a 240-line literal array inside
     * `_admin_sidebar.html.twig`, with its own gating written in Twig and its own
     * notion of "current" — a second navigation model beside this one, which is
     * why the admin and the public side felt like two products. One builder now.
     *
     * **Why by feature.** The old headings were shapes of work — Réserver,
     * Activités, Suivi, Configuration — so "where do I manage events?" had no
     * answer you could derive; you had to know. Grouping by the feature that owns
     * the screen means the sidebar reorganises itself as an operator switches
     * things on and off, and a feature that is off takes its whole section with
     * it rather than leaving a heading behind.
     *
     * ⚠️ **Two independent gates, and they answer different questions.**
     * `feature` asks whether this install *has* the thing; `canReach()` asks
     * whether the firewall would let *this person* open it. Both are needed:
     * `staff-access-passes` is the one non-admin page carrying this sidebar, and
     * without the second gate a staff member is shown thirty `/admin` links that
     * all bounce them to `/login`.
     *
     * ⚠️ Presentation, both of them. Hiding an entry hides an entry.
     *
     * @return list<array{label: ?string, items: list<array<string, mixed>>}>
     */
    public function admin(): array
    {
        // Kernel screens. `feature: null` on purpose — an operator who has
        // switched everything off must still be able to reach the switches.
        $sections = [
            $this->adminSection(null, [
                $this->adminItem('Tableau de bord', 'app_admin_dashboard', 'dashboard', [
                    'app_admin_dashboard_alt', 'app_admin_dashboard_scoped_html', 'app_admin_dashboard_legacy_html',
                ]),
                $this->adminItem('État de l’installation', 'app_admin_setup', 'dashboard'),
            ]),
        ];

        // One section per feature that owns admin screens, in the registry's own
        // order so the sidebar and the features screen read the same way down.
        foreach ($this->adminByFeature() as $feature => $spec) {
            $sections[] = $this->adminSection($spec['label'], $spec['items'], $feature);
        }

        $sections[] = $this->adminSection('Le lieu', [
            $this->adminItem('Utilisateurs', 'app_admin_users', 'users', [
                'app_admin_users_scoped_html', 'app_admin_users_double_legacy_html',
                'app_admin_user_new', 'app_admin_user_detail',
            ]),
            $this->adminItem('Horaires', 'app_admin_opening_hours', 'hours'),
            $this->adminItem('Interface accueil', 'app_admin_homepage', 'dashboard'),
            $this->adminItem('Portails', 'app_admin_portals', 'dashboard', ['app_admin_portal_edit']),
        ]);

        $sections[] = $this->adminSection('Configuration', [
            $this->adminItem('Fonctionnalités', 'app_admin_features', 'dashboard'),
            $this->adminItem('Réglages du site', 'app_admin_settings', 'dashboard'),
            $this->adminItem('E-mails', 'app_admin_emails', 'logs'),
            $this->adminItem('Configuration initiale', 'app_admin_wizard', 'dashboard'),
        ]);

        // Development tools are a deliberate, opt-in workspace for this dev
        // installation. They are still ROLE_ADMIN routes; the flag merely keeps
        // them out of the everyday operator navigation. Disable it before a
        // production launch (tracked in docs/PROJECT_STATE.md).
        if ($this->settings->isDevelopmentMode()) {
            $sections[] = $this->adminSection('Développement', [
                $this->adminItem('Design', 'app_admin_design', 'dashboard'),
                $this->adminItem('Pages introuvables', 'app_admin_missing_pages', 'logs'),
            ]);
        }

        return array_values(array_filter($sections));
    }

    /**
     * Which admin screens belong to which feature.
     *
     * ⚠️ The labels are SHORT — they name the section, not the feature. The
     * registry's own labels are operator-facing sentences ("Réserver de
     * l’équipement") written for the features screen, where there is room for a
     * sentence; in a sidebar they wrap to three lines.
     *
     * ⚠️ `bookings` is not a registry feature. Bookings are polymorphic since
     * S8–S10, so the booking screens are gated on "any bookable layer at all"
     * rather than on `machines` — otherwise a spaces-only deployment that books
     * perfectly well loses its reservations screen.
     *
     * @return array<string, array{label: string, items: list<array<string, mixed>|null>}>
     */
    /**
     * The admin section the reader is currently inside, with its items.
     *
     * ⚠️ **This is what lets the sidebar stop being 42 rows long.** With every
     * entry of every feature listed at once the sidebar was taller than most
     * screens, so the thing you were looking for was usually scrolled off. The
     * sidebar shows SECTIONS now and this supplies the one section's contents,
     * which the page renders as a strip across the top — two short lists instead
     * of one long one.
     *
     * Returns null on an admin page that belongs to no section (the dashboard),
     * and the caller simply draws no strip.
     *
     * @return array{label: ?string, items: list<array<string, mixed>>}|null
     */
    public function adminCurrentSection(): array|null
    {
        $route = $this->requests->getCurrentRequest()?->attributes->get('_route');
        if (!is_string($route)) {
            return null;
        }

        foreach ($this->admin() as $section) {
            foreach ($section['items'] as $item) {
                if (in_array($route, $item['activeRoutes'], true)) {
                    // The ungrouped block (dashboard, install health) has no label
                    // and is not a section anyone navigates *within*.
                    return $section['label'] === null ? null : $section;
                }
            }
        }

        return null;
    }

    private function adminByFeature(): array
    {
        return [
            'machines' => ['label' => 'Équipement', 'items' => [
                $this->adminItem('Machines', 'app_admin_machines', 'machines', [
                    'app_admin_machines_scoped_html', 'app_admin_machines_double_legacy_html',
                    'app_admin_machine_new', 'app_admin_machine_edit',
                ]),
                $this->adminItem('Maintenance', 'app_admin_maintenance', 'machines', [
                    'app_admin_maintenance_new', 'app_admin_maintenance_batch',
                ], feature: 'maintenance'),
                $this->adminItem('Utilisations', 'app_admin_usage_logs', 'usage'),
                $this->adminItem('Logs RFID', 'app_admin_access_rfid_logs', 'logs'),
                $this->adminItem('Lecteurs RFID', 'app_admin_rfid_readers', 'logs', [
                    'app_admin_rfid_reader_new', 'app_admin_rfid_reader_edit',
                ]),
            ]],
            'places' => ['label' => 'Espaces', 'items' => [
                $this->adminItem('Espaces', 'app_admin_places', 'machines', [
                    'app_admin_place_new', 'app_admin_place_edit',
                ]),
            ]],
            'bookings' => ['label' => 'Réservations', 'items' => [
                $this->adminItem('Réservations', 'app_admin_reservations', 'reservations'),
                $this->adminItem('Quotas de réservation', 'app_admin_booking_policies', 'dashboard'),
                $this->adminItem('Accès exceptionnels', 'app_staff_access_passes', 'dashboard'),
            ]],
            'events' => ['label' => 'Événements', 'items' => [
                $this->adminItem('Événements', 'app_admin_events', 'reservations', [
                    'app_admin_event_new', 'app_admin_event_edit', 'app_admin_event_registrations',
                ]),
            ]],
            'loans' => ['label' => 'Prêts', 'items' => [
                $this->adminItem('Objets prêtables', 'app_admin_loanable_items', 'machines', [
                    'app_admin_loanable_item_new', 'app_admin_loanable_item_edit',
                ]),
                $this->adminItem('Prêts', 'app_admin_loans', 'reservations', ['app_admin_loan_new']),
            ]],
            'materials' => ['label' => 'Matériaux', 'items' => [
                $this->adminItem('Matériaux', 'app_admin_materials', 'machines', [
                    'app_admin_material_new', 'app_admin_material_edit',
                ]),
            ]],
            'formations' => ['label' => 'Formations', 'items' => [
                $this->adminItem('Formations', 'app_admin_formations', 'formations', [
                    'app_admin_formation_new', 'app_admin_formation_edit', 'app_admin_formation_content',
                ]),
            ]],
            'badges' => ['label' => 'Badges', 'items' => [
                $this->adminItem('Badges', 'app_admin_badges', 'badges', [
                    'app_admin_badge_new', 'app_admin_badge_edit',
                ]),
                // Institutions are the external bodies that recognise a badge;
                // keeping them here makes that relationship visible at the point
                // where an operator manages the badges themselves.
                $this->adminItem('Institutions', 'app_admin_institutions', 'badges', [
                    'app_admin_institution_new', 'app_admin_institution_edit',
                ]),
            ]],
            'projects' => ['label' => 'Créations', 'items' => [
                $this->adminItem('Créations', 'app_admin_creations', 'creations', [
                    'app_admin_creation_new', 'app_admin_creation_edit',
                ]),
            ]],
            'lab_pages' => ['label' => 'Pages du Lab', 'items' => [
                $this->adminItem('Pages du Lab', 'app_admin_lab_pages', 'dashboard', [
                    'app_admin_lab_page_new', 'app_admin_lab_page_edit',
                ]),
            ]],
        ];
    }

    /**
     * @param list<string> $alsoActiveOn extra routes that should light this entry —
     *                     the new/edit screens that belong to the same list
     *
     * @return array<string, mixed>|null
     */
    private function adminItem(string $label, string $route, string $icon, array $alsoActiveOn = [], ?string $feature = null): ?array
    {
        if ($feature !== null && !$this->features->isEnabled($feature)) {
            return null;
        }
        if (!$this->access->canReach($route)) {
            return null;
        }

        return [
            'label' => $label,
            'route' => $route,
            'icon' => $icon,
            'activeRoutes' => [$route, ...$alsoActiveOn],
        ];
    }

    /**
     * A section, or null when nothing in it survived the gates.
     *
     * ⚠️ Rule 1 from `group()` restated: no children, no heading. A section that
     * kept its label after its only entry was gated away is a heading that leads
     * nowhere, and the operator reads it as something broken rather than as
     * something switched off.
     *
     * @param list<array<string, mixed>|null> $items
     *
     * @return array{label: ?string, items: list<array<string, mixed>>}|null
     */
    private function adminSection(?string $label, array $items, ?string $feature = null): ?array
    {
        // `bookings` is the polymorphic case — see adminByFeature().
        if ($feature === 'bookings') {
            if (!$this->features->hasCalendarLayer()) {
                return null;
            }
        } elseif ($feature !== null && !$this->features->isEnabled($feature)) {
            return null;
        }

        $visible = array_values(array_filter($items));

        return $visible === [] ? null : ['label' => $label, 'items' => $visible];
    }

    private function item(
        string $label,
        string $route,
        array $params = [],
        ?string $feature = null,
        ?string $role = null,
        ?bool $visible = null,
        bool $translate = true,
        bool $child = false,
    ): ?array {
        if ($feature !== null && !$this->features->isEnabled($feature)) {
            return null;
        }
        if ($role !== null && !$this->security->isGranted($role)) {
            return null;
        }
        if ($visible === false) {
            return null;
        }

        return ['label' => $label, 'translate' => $translate, 'route' => $route, 'params' => $params, 'child' => $child];
    }

    /** @return array<string, mixed>|null */
    private function link(string $label, string $route, ?string $feature = null, ?string $role = null, ?bool $visible = null): ?array
    {
        $item = $this->item($label, $route, feature: $feature, role: $role, visible: $visible);

        return $item === null ? null : ['kind' => 'link', ...$item, 'items' => []];
    }

    /**
     * @param list<array<string, mixed>|null> $items
     *
     * @return array<string, mixed>|null
     */
    private function group(string $label, ?string $preferredRoute, array $items, bool $translateLabel = true): ?array
    {
        $visible = array_values(array_filter($items));

        // Rule 1: no children, no group. Rule 2: with the natural landing page
        // gone, the heading follows the first child that is still there.
        if ($visible === []) {
            return null;
        }

        return [
            'kind' => 'group',
            'label' => $label,
            'translate' => $translateLabel,
            'route' => $preferredRoute ?? $visible[0]['route'],
            'params' => $preferredRoute !== null ? [] : $visible[0]['params'],
            'items' => $visible,
        ];
    }
}
