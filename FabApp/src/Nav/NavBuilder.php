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
        // ⚠️ **Labelled "Au programme", not "Calendrier" (S146, review).** The page it
        // lands on stopped being able to book in S146c, and "Calendrier" is the most
        // obvious word for "I want to book" — it sent people to a screen that cannot.
        // ⚠️ `nav.calendar` is NOT renamed: it is also the machine page's tab label,
        // and there the calendar really is a calendar you book from.
        //
        // 🔴 **S147, J-13: renaming the group was only half the fix.** The child said
        // "Réserver une machine" and still pointed at `app_calendar` — the one surface
        // built with `booking: false` — so the most action-shaped entry in the public
        // menu led to the only calendar that refuses. It goes to `app_machines` now,
        // which is where a machine's own week and its booking panel live, and it
        // matches its sibling: `cal.book_space` has always pointed at `app_places`.
        // ⚠️ The group itself still lands on `app_calendar`, and that is right — the
        // group is "what is on", not "book something".
        $nav[] = $this->group(
            'nav.whats_on',
            $this->features->hasCalendarLayer() ? 'app_calendar' : null,
            [
                $this->item('cal.book_machine', 'app_machines', feature: 'machines'),
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
            $this->item('nav.whats_on', 'app_calendar', visible: $this->features->hasCalendarLayer()),
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
                $this->adminItem('admin_nav.entry.app_admin_dashboard', 'app_admin_dashboard', 'dashboard', [
                    'app_admin_dashboard_alt',
                ]),
                // ⚠️ État de l'installation moved to Configuration in S130. It is a
                // configuration screen, and sitting beside the dashboard implied it was
                // a landing page rather than a setting.
            ]),
        ];

        // One section per feature that owns admin screens, in the registry's own
        // order so the sidebar and the features screen read the same way down.
        // ⚠️ The key is the section's feature gate by default. A spec may override it
        // with an explicit `gate` — `null` means "this section has no single owning
        // feature, the items gate themselves" (S130, Équipement).
        foreach ($this->adminByFeature() as $feature => $spec) {
            $sections[] = $this->adminSection($spec['label'], $spec['items'], array_key_exists('gate', $spec) ? $spec['gate'] : $feature);
        }

        // ⚠️ **S130 replaced the "Le lieu" grab-bag.** It held Utilisateurs, Horaires,
        // Thèmes, Réseau and Packages — five unrelated workspaces under a heading that
        // named none of them, which is why "where do I manage X?" had no derivable
        // answer. Each is now its own canonical entry, matching the workspace list in
        // `FeatureWorkspaceRegistry`: one workspace, one heading, one place to look.
        $sections[] = $this->adminSection('admin_nav.section.locations', [
            $this->adminItem('admin_nav.entry.app_admin_venues', 'app_admin_venues', 'places', [
                'app_admin_venue_new', 'app_admin_venue_edit', 'app_admin_venue_archive',
            ]),
            $this->adminItem('admin_nav.entry.app_admin_opening_hours', 'app_admin_opening_hours', 'hours'),
        ]);

        $sections[] = $this->adminSection('admin_nav.section.users', [
            $this->adminItem('admin_nav.entry.app_admin_users', 'app_admin_users', 'users', [
                'app_admin_user_new', 'app_admin_user_detail',
            ]),
            // The per-person quota screen. It is the same route as the machine and
            // space quotas, told apart by `reservableType` — see `isCurrent()`.
            $this->adminItem('admin_nav.entry.app_admin_booking_policies', 'app_admin_booking_policies', 'reservations', feature: 'bookings', params: ['reservableType' => 'user']),
        ]);

        // ⚠️ **The shadow is a real destination (S133b), not a dev tool.** It is
        // the screen an operator has to read before S134 turns grants v2 on, so
        // burying it behind the development flag — where `/admin/design` and the
        // three vision pages live — would have made the one prerequisite of the
        // activation invisible on any installation that had switched that flag
        // off, which is every production one.
        $sections[] = $this->adminSection('admin_nav.section.packages', [
            $this->adminItem('admin_nav.entry.app_admin_usage_rights', 'app_admin_usage_rights', 'usage', [
                'app_admin_usage_rights_new', 'app_admin_usage_rights_edit',
            ]),
            $this->adminItem('admin_nav.entry.app_admin_usage_rights_shadow', 'app_admin_usage_rights_shadow', 'usage'),
        ]);

        // Réglages first: it is the entry an operator means when they say
        // "configuration", so it is what the group opens on. État de l'installation
        // moved out of the unlabelled kernel block and Thèmes out of "Le lieu" —
        // both are configuration, and neither was findable where it sat.
        //
        // ⚠️ **Réseau FabOS joined this section in S130d** (operator's call). It was
        // a top-level section of its own holding exactly one screen, so it took a
        // whole sidebar row — level with Équipement's eleven screens and Formations'
        // catalogue — to say one thing, and under S130b's rule it drew no strip at
        // all. What it configures is this instance's identity, its OIDC providers
        // and its trusted peers: settings, not a workspace an operator works in.
        //
        // ⚠️ Institutions is NOT here, though the retired workspace registry listed
        // it as a Réseau tab. `Institution` is referenced by exactly one entity —
        // `Badge`, many-to-many — so it is a badge issuer, not a federated peer. It
        // stays under Badges.
        $sections[] = $this->adminSection('admin_nav.section.configuration', [
            $this->adminItem('admin_nav.entry.app_admin_settings', 'app_admin_settings', 'dashboard'),
            $this->adminItem('admin_nav.entry.app_admin_setup', 'app_admin_setup', 'dashboard'),
            // `/admin/homepage` is the themes draft editor under another address. It
            // was reachable only from the retired workspace tabs; without it here,
            // deleting them would have orphaned the screen outright.
            $this->adminItem('admin_nav.entry.app_admin_themes', 'app_admin_themes', 'dashboard', ['app_admin_homepage']),
            $this->adminItem('admin_nav.entry.app_admin_features', 'app_admin_features', 'dashboard'),
            $this->adminItem('admin_nav.entry.app_admin_emails', 'app_admin_emails', 'logs'),
            $this->adminItem('admin_nav.entry.app_admin_network', 'app_admin_network', 'dashboard'),
            // Last, because it is the one entry you use once and never again.
            $this->adminItem('admin_nav.entry.app_admin_wizard', 'app_admin_wizard', 'dashboard'),
        ]);

        // Development tools are a deliberate, opt-in workspace for this dev
        // installation. They are still ROLE_ADMIN routes; the flag merely keeps
        // them out of the everyday operator navigation. Disable it before a
        // production launch (tracked in docs/PROJECT_STATE.md).
        if ($this->settings->isDevelopmentMode()) {
            $sections[] = $this->adminSection('admin_nav.section.development', [
                $this->adminItem('admin_nav.entry.app_admin_design', 'app_admin_design', 'dashboard'),
                $this->adminItem('admin_nav.entry.app_admin_workspace_vision', 'app_admin_workspace_vision', 'dashboard'),
                $this->adminItem('admin_nav.entry.app_admin_usage_rights_vision', 'app_admin_usage_rights_vision', 'usage'),
                $this->adminItem('admin_nav.entry.app_admin_structure_vision', 'app_admin_structure_vision', 'dashboard'),
                $this->adminItem('admin_nav.entry.app_admin_missing_pages', 'app_admin_missing_pages', 'logs'),
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
    /**
     * The translation key that names the page being looked at.
     *
     * ⚠️ **This is the title of every admin list** (operator, 2026-08-16):
     * "Quotas", not "Gestion des quotas". The word is already written in the
     * navigation strip, so a page that also carries a hand-written heading is
     * maintaining a second, differently-worded copy of it — translated into five
     * languages, and free to drift from the menu that leads there. Reading the
     * menu's own key instead is what let the per-page `*.title` keys be deleted
     * outright rather than merely aligned, and `AdminNavCatalogueTest` — which
     * already fails when an entry lacks its key in any of the five catalogues —
     * is what makes that safe.
     *
     * ⚠️ **The ENTRY, not the section.** Catégories, Modèles & marques, Machines
     * and Matériaux all sit under Équipement, and three of them cannot be called
     * "Équipement". The one exception is `/admin/machines` itself, which says
     * "Équipement" because it is that group's landing page and because that is
     * the rendering the operator approved on screen; it is carried as
     * `titleLabel` on the entry rather than as an `if` on the route somewhere
     * downstream.
     *
     * Returns null on a screen the admin navigation does not name — the staff
     * pass desk, which is not an admin destination at all — and the shell then
     * uses the title it was passed.
     */
    public function adminCurrentTitle(): ?string
    {
        foreach ($this->admin() as $section) {
            foreach ($section['items'] as $item) {
                if ($item['active']) {
                    return $item['titleLabel'] ?? $item['label'];
                }
            }
        }

        return null;
    }

    public function adminCurrentSection(): array|null
    {
        foreach ($this->admin() as $section) {
            foreach ($section['items'] as $item) {
                if (!$item['active']) {
                    continue;
                }
                // The ungrouped block (dashboard, install health) has no label
                // and is not a section anyone navigates *within*.
                //
                // ⚠️ A one-entry section is still returned. It has nothing to
                // navigate *within*, so the template draws no strip for it — but
                // the sidebar still needs to know which row to light, and that is
                // the same question with a different answer.
                return $section['label'] === null ? null : $section;
            }
        }

        return null;
    }

    private function adminByFeature(): array
    {
        return [
            // ⚠️ **S130: this section carries no `feature` gate of its own.** It used
            // to be gated on `machines`, which was fine while it only held machine
            // screens. Matériaux moved in here per the navigation decision, and a
            // section-level `machines` gate would have hidden Matériaux on an install
            // that runs materials without machines — a real configuration, and a
            // silent loss. Every item names its own feature instead, and
            // `adminSection()` still drops the heading when they all gate away.
            'machines' => ['label' => 'admin_nav.section.equipment', 'gate' => null, 'items' => [
                // ⚠️ The one page whose heading is its SECTION and not its entry
                // (operator, 2026-08-16). It is the landing page of Équipement,
                // and it is the rendering that was approved on screen.
                $this->adminItem('admin_nav.entry.app_admin_machines', 'app_admin_machines', 'machines', [
                    'app_admin_machine_new', 'app_admin_machine_edit',
                ], feature: 'machines', titleLabel: 'admin_nav.section.equipment'),
                // Catégories and Modèles & marques existed only as workspace tabs
                // until S130b. Both are live routes; neither was reachable from the
                // sidebar, so an operator who never noticed the second strip could
                // not find them at all.
                $this->adminItem('admin_nav.entry.app_admin_machine_categories', 'app_admin_machine_categories', 'machines', feature: 'machines'),
                $this->adminItem('admin_nav.entry.app_admin_machine_models', 'app_admin_machine_models', 'machines', feature: 'machines'),
                // Moved from its own top-level "Matériaux" section (S130). A material
                // is stock consumed at a machine, not a peer of the equipment group.
                $this->adminItem('admin_nav.entry.app_admin_materials', 'app_admin_materials', 'materials', [
                    'app_admin_material_new', 'app_admin_material_edit',
                ], feature: 'materials'),
                $this->adminItem('admin_nav.entry.app_admin_maintenance', 'app_admin_maintenance', 'machines', [
                    'app_admin_maintenance_new', 'app_admin_maintenance_batch',
                ], feature: ['machines', 'maintenance']),
                $this->adminItem('admin_nav.entry.app_admin_reservations', 'app_admin_reservations', 'reservations', feature: 'bookings', params: ['reservableType' => 'machine']),
                $this->adminItem('admin_nav.entry.app_admin_booking_policies', 'app_admin_booking_policies', 'reservations', feature: 'bookings', params: ['reservableType' => 'machine']),
                $this->adminItem('admin_nav.entry.app_admin_usage_logs', 'app_admin_usage_logs', 'usage', feature: 'machines'),
                $this->adminItem('admin_nav.entry.app_admin_access_rfid_logs', 'app_admin_access_rfid_logs', 'logs', feature: 'machines'),
                $this->adminItem('admin_nav.entry.app_admin_rfid_readers', 'app_admin_rfid_readers', 'logs', [
                    'app_admin_rfid_reader_new', 'app_admin_rfid_reader_edit',
                ], feature: 'machines'),
                $this->adminItem('admin_nav.entry.app_admin_reporting', 'app_admin_reporting', 'usage', feature: 'machines', params: ['workspace' => 'equipment']),
            ]],
            'places' => ['label' => 'admin_nav.section.spaces', 'items' => [
                $this->adminItem('admin_nav.entry.app_admin_places', 'app_admin_places', 'machines', [
                    'app_admin_place_new', 'app_admin_place_edit',
                ]),
                $this->adminItem('admin_nav.entry.app_admin_reservations', 'app_admin_reservations', 'reservations', feature: 'bookings', params: ['reservableType' => 'place']),
                $this->adminItem('admin_nav.entry.app_admin_booking_policies', 'app_admin_booking_policies', 'reservations', feature: 'bookings', params: ['reservableType' => 'place']),
                $this->adminItem('admin_nav.entry.app_admin_reporting', 'app_admin_reporting', 'usage', params: ['workspace' => 'spaces']),
            ]],
            'events' => ['label' => 'admin_nav.section.events', 'items' => [
                $this->adminItem('admin_nav.entry.app_admin_events', 'app_admin_events', 'reservations', [
                    'app_admin_event_new', 'app_admin_event_edit', 'app_admin_event_registrations',
                ]),
                // S146f. The categories screen has to be reachable from the sidebar
                // or it is the machine-categories mistake again: a live route nobody
                // who did not already know the URL could find.
                $this->adminItem('admin_nav.entry.app_admin_event_categories', 'app_admin_event_categories', 'reservations', feature: 'events'),
            ]],
            // ⚠️ **S147, J-19 — a section lands on its FIRST item**, and this one is
            // labelled "Prêts". It used to open the object catalogue, so the entry
            // named after the verb showed you the nouns, and lending something cost
            // three clicks where every other create costs two. Loans happen daily and
            // the catalogue is setup, so the frequent action takes the short path.
            'loans' => ['label' => 'admin_nav.section.loans', 'items' => [
                $this->adminItem('admin_nav.entry.app_admin_loans', 'app_admin_loans', 'reservations', ['app_admin_loan_new']),
                $this->adminItem('admin_nav.entry.app_admin_loanable_items', 'app_admin_loanable_items', 'machines', [
                    'app_admin_loanable_item_new', 'app_admin_loanable_item_edit',
                ]),
            ]],
            'formations' => ['label' => 'admin_nav.section.training', 'items' => [
                $this->adminItem('admin_nav.entry.app_admin_formations', 'app_admin_formations', 'formations', [
                    'app_admin_formation_new', 'app_admin_formation_edit', 'app_admin_formation_content',
                ]),
            ]],
            'badges' => ['label' => 'admin_nav.section.badges', 'items' => [
                $this->adminItem('admin_nav.entry.app_admin_badges', 'app_admin_badges', 'badges', [
                    'app_admin_badge_new', 'app_admin_badge_edit',
                ]),
                // Institutions are the external bodies that recognise a badge;
                // keeping them here makes that relationship visible at the point
                // where an operator manages the badges themselves.
                $this->adminItem('admin_nav.entry.app_admin_institutions', 'app_admin_institutions', 'badges', [
                    'app_admin_institution_new', 'app_admin_institution_edit',
                ]),
            ]],
            'projects' => ['label' => 'admin_nav.section.projects', 'items' => [
                $this->adminItem('admin_nav.entry.app_admin_creations', 'app_admin_creations', 'creations', [
                    'app_admin_creation_new', 'app_admin_creation_edit',
                ]),
            ]],
            // Renamed in S130. "Pages du Lab" described who wrote them; "Pages
            // personnalisées" describes what they are, and matches the registry.
            'lab_pages' => ['label' => 'admin_nav.section.pages', 'items' => [
                $this->adminItem('admin_nav.entry.app_admin_lab_pages', 'app_admin_lab_pages', 'dashboard', [
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
    /**
     * @param string|list<string>|null $feature one feature, or several that must
     *        **all** be enabled. ⚠️ The list form arrived in S130, when Matériaux
     *        moved under Équipement: the section could no longer carry a single
     *        gate for everything inside it, so items that used to inherit
     *        `machines` from their section now name it themselves — and
     *        Maintenance needs `machines` *and* `maintenance`, which a scalar
     *        could not express without silently dropping one of the two.
     * @param array<string, string|int> $params the arguments that make this entry
     *        a distinct destination — see `isCurrent()`
     * @param ?string $titleLabel the key the PAGE heading uses, when it differs
     *        from the menu row's. One entry sets it — see `adminCurrentTitle()`.
     */
    private function adminItem(string $label, string $route, string $icon, array $alsoActiveOn = [], string|array|null $feature = null, array $params = [], ?string $titleLabel = null): ?array
    {
        foreach ((array) ($feature ?? []) as $required) {
            if (!$this->featureAllows($required)) {
                return null;
            }
        }
        // ⚠️ **The parameters go to `canReach()` too.** It answers by generating
        // the URL, and `/admin/reporting/{workspace}` cannot be generated without
        // one — so an unparameterised call threw, was caught as "unreachable",
        // and Reporting disappeared from both strips while `/admin/reporting/…`
        // rendered with no navigation at all. Silent, because vanishing quietly
        // is exactly what that catch is for.
        if (!$this->access->canReach($route, $params)) {
            return null;
        }

        $item = [
            'label' => $label,
            'titleLabel' => $titleLabel,
            'route' => $route,
            'icon' => $icon,
            'params' => $params,
            'activeRoutes' => [$route, ...$alsoActiveOn],
        ];

        return $item + ['active' => $this->isCurrent($item)];
    }

    /**
     * Is this entry the page being looked at?
     *
     * ⚠️ **The route alone stopped being enough in S130b.** Réservations, Quotas
     * and Reporting are one route each serving several sections:
     * `/admin/reservations` is the machine list, the space list and the user list
     * depending on `reservableType`. Matching on the route lit all three at once,
     * and `adminCurrentSection()` — which returns the first section holding a
     * matching entry — then put an operator looking at space reservations inside
     * Équipement. The parameters are part of the address, so they are part of the
     * comparison.
     *
     * A route parameter (`/admin/reporting/{workspace}`) lands in the request
     * attributes and a query parameter (`?reservableType=`) in the query bag;
     * both are read here so a caller never has to say which kind it declared.
     *
     * @param array<string, mixed> $item
     */
    private function isCurrent(array $item): bool
    {
        $request = $this->requests->getCurrentRequest();
        $route = $request?->attributes->get('_route');
        if (!is_string($route) || !in_array($route, $item['activeRoutes'], true)) {
            return false;
        }

        foreach ($item['params'] as $name => $expected) {
            $actual = $request->attributes->get($name) ?? $request->query->get($name);
            if ((string) $actual !== (string) $expected) {
                return false;
            }
        }

        return true;
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
        if ($feature !== null && !$this->featureAllows($feature)) {
            return null;
        }

        $visible = array_values(array_filter($items));

        return $visible === [] ? null : ['label' => $label, 'items' => $visible];
    }

    /**
     * Is this feature name switched on for this install?
     *
     * ⚠️ **`bookings` is not a registry feature.** Bookings are polymorphic since
     * S8–S10, so the booking screens gate on "any bookable layer at all" rather
     * than on `machines` — otherwise a spaces-only deployment that books
     * perfectly well loses its reservations screen. That rule lived only in
     * `adminSection()` until S130b put Réservations and Quotas on individual
     * *items*; asking `isEnabled('bookings')` there would have answered false and
     * hidden them everywhere. One word, one meaning, one place.
     */
    private function featureAllows(string $feature): bool
    {
        return $this->features->allowsSurface($feature);
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
