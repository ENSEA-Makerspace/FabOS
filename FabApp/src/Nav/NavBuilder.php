<?php

namespace App\Nav;

use App\Feature\SiteFeatureService;
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

        $nav[] = $this->link('resv.title', 'app_my_reservations', role: 'ROLE_USER');

        // A bookable person manages their slots and answers requests often enough
        // to deserve a nav link; everyone else reaches it from their profile.
        $user = $this->security->getUser();
        $bookable = $user !== null && method_exists($user, 'isBookable') && $user->isBookable();
        $nav[] = $this->link('booking.my_availability', 'app_person_my_availability', feature: 'person_booking', visible: $bookable);

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
            $this->item('resv.title', 'app_my_reservations', role: 'ROLE_USER'),
        ];

        return array_values(array_filter($items));
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
