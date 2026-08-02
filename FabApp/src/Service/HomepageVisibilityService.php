<?php

namespace App\Service;

use App\Entity\HomepageSectionVisibility;
use App\Feature\SiteFeatureService;
use App\Repository\HomepageSectionVisibilityRepository;
use Doctrine\DBAL\Exception as DbalException;
use Doctrine\ORM\Exception\ORMException;
use Symfony\Component\Security\Core\User\UserInterface;

class HomepageVisibilityService
{
    /**
     * The blocks that live in the HERO DECK rather than in the page's own flow.
     *
     * ⚠️ They are still real sections with real visibility rows — an operator
     * switches them off exactly as before, per audience, from the same screen.
     * What changed in S79 is only WHERE they are drawn: across the bottom of the
     * hero, because "are you open?" and "what's on?" are the two questions a
     * visitor arrives with and both were a scroll and a half down the page.
     *
     * ⚠️ Being in the deck is what takes them out of the personalisation modal.
     * The deck's two panels have a fixed left/right arrangement; offering to
     * drag them up and down a list they are no longer part of would be a control
     * that does nothing — the whole family of bug S79's sibling sessions keep
     * finding. One list, here, read by the personalisation service.
     */
    public const DECK_SECTIONS = ['opening_hours', 'upcoming_events'];

    public const ROLE_ANONYMOUS = 'anonymous';
    public const ROLE_USER = 'user';
    public const ROLE_STAFF = 'staff';
    public const ROLE_ADMIN = 'admin';

    /**
     * @var array<string, array{label: string, feature: ?string, visibleAnonymous: bool, visibleUser: bool, visibleStaff: bool, visibleAdmin: bool, sortOrder: int}>
     */
    private const DEFAULT_SECTIONS = [
        'opening_hours' => [
            'feature' => null,
            'label' => 'Horaires d’ouverture',
            'visibleAnonymous' => true,
            'visibleUser' => true,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 10,
        ],
        'upcoming_events' => [
            'feature' => 'events',
            'label' => 'Événements à venir',
            'visibleAnonymous' => true,
            'visibleUser' => true,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 15,
        ],
        'how_it_works' => [
            'feature' => null,
            'label' => 'Comment ça fonctionne ?',
            'visibleAnonymous' => true,
            'visibleUser' => true,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 20,
        ],
        'fablab_stats' => [
            'feature' => 'machines',
            'label' => 'Statistiques du lieu',
            'visibleAnonymous' => false,
            'visibleUser' => false,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 30,
        ],
        'mini_leaderboard' => [
            'feature' => 'leaderboard',
            'label' => 'Mini classement',
            'visibleAnonymous' => false,
            'visibleUser' => true,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 40,
        ],
        'featured_machines' => [
            'feature' => 'machines',
            'label' => 'Machines à découvrir',
            'visibleAnonymous' => true,
            'visibleUser' => true,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 50,
        ],
        'latest_rfid_logs' => [
            'feature' => 'machines',
            'label' => 'Derniers passages RFID',
            'visibleAnonymous' => false,
            'visibleUser' => false,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 60,
        ],
    ];

    public function __construct(
        private readonly HomepageSectionVisibilityRepository $sections,
        private readonly SiteFeatureService $features,
    ) {
    }

    /** @return array<string, array{label: string, feature: ?string, visibleAnonymous: bool, visibleUser: bool, visibleStaff: bool, visibleAdmin: bool, sortOrder: int}> */
    public function getDefaultSections(): array
    {
        return self::DEFAULT_SECTIONS;
    }

    /** @return array<int, array{sectionKey: string, label: string, visibleAnonymous: bool, visibleUser: bool, visibleStaff: bool, visibleAdmin: bool, sortOrder: int, entity: ?HomepageSectionVisibility}> */
    public function getAdminRows(): array
    {
        $entitiesByKey = [];
        foreach ($this->findConfiguredSections() as $section) {
            $entitiesByKey[$section->getSectionKey()] = $section;
        }

        $rows = [];
        foreach (self::DEFAULT_SECTIONS as $sectionKey => $defaults) {
            $entity = $entitiesByKey[$sectionKey] ?? null;
            $rows[] = $this->buildRow($sectionKey, $defaults, $entity);
        }

        usort($rows, static fn (array $a, array $b): int => $a['sortOrder'] <=> $b['sortOrder']);

        return $rows;
    }

    /** @return array<string, bool> */
    public function getVisibilityMap(?UserInterface $user): array
    {
        $role = $this->resolveAudience($user);
        $visibility = [];

        foreach ($this->getAdminRows() as $row) {
            $visibility[$row['sectionKey']] = $this->featureAllows($row['sectionKey']) && match ($role) {
                self::ROLE_ADMIN => $row['visibleAdmin'],
                self::ROLE_STAFF => $row['visibleStaff'],
                self::ROLE_USER => $row['visibleUser'],
                default => $row['visibleAnonymous'],
            };
        }

        return $visibility;
    }

    /**
     * Whether the feature behind a block is switched on at all.
     *
     * This is deliberately folded into the visibility map rather than checked at
     * each call site: the map is what the controller loads data from, what the
     * template renders from, *and* what the personalisation service builds the
     * section order out of. Gating it here means a block that is off cannot be
     * loaded, cannot be drawn, and cannot be dragged into an order — one rule,
     * three places that used to be able to disagree.
     *
     * ⚠️ It is a *presentation* rule, exactly like the two navs. Hiding a block
     * hides a block; the route gate and the firewall still decide what exists.
     *
     * ⚠️ `mini_leaderboard` is why this exists. `leaderboard` is a real registry
     * feature, so switching it off hid the nav entry and made `/leaderboard`
     * 404 — while the homepage carried on rendering a leaderboard block linking
     * straight at it. Same bug S37 fixed on the error page.
     */
    private function featureAllows(string $sectionKey): bool
    {
        $feature = self::DEFAULT_SECTIONS[$sectionKey]['feature'] ?? null;

        return $feature === null || $this->features->isEnabled($feature);
    }

    public function isVisible(string $sectionKey, ?UserInterface $user): bool
    {
        return $this->getVisibilityMap($user)[$sectionKey] ?? false;
    }

    public function resolveAudience(?UserInterface $user): string
    {
        if ($user === null) {
            return self::ROLE_ANONYMOUS;
        }

        $roles = $user->getRoles();
        if (in_array('ROLE_ADMIN', $roles, true)) {
            return self::ROLE_ADMIN;
        }

        if (in_array('ROLE_STAFF', $roles, true)) {
            return self::ROLE_STAFF;
        }

        return self::ROLE_USER;
    }

    /** @return HomepageSectionVisibility[] */
    private function findConfiguredSections(): array
    {
        try {
            return $this->sections->findOrdered();
        } catch (DbalException|ORMException $exception) {
            return [];
        } catch (\RuntimeException $exception) {
            return [];
        }
    }

    /**
     * @param array{label: string, feature: ?string, visibleAnonymous: bool, visibleUser: bool, visibleStaff: bool, visibleAdmin: bool, sortOrder: int} $defaults
     * @return array{sectionKey: string, label: string, visibleAnonymous: bool, visibleUser: bool, visibleStaff: bool, visibleAdmin: bool, sortOrder: int, entity: ?HomepageSectionVisibility}
     */
    private function buildRow(string $sectionKey, array $defaults, ?HomepageSectionVisibility $entity): array
    {
        return [
            'sectionKey' => $sectionKey,
            // The label is code-owned, not operator data: the admin screen renders it
            // as a caption, never as an input, and AdminController writes it back from
            // this same table on save. Letting a stored row win meant a snapshot taken
            // at the last save shadowed the code — which is why renaming a label here
            // silently did nothing on any install that had ever saved the form.
            'label' => $defaults['label'],
            'visibleAnonymous' => $entity?->isVisibleAnonymous() ?? $defaults['visibleAnonymous'],
            'visibleUser' => $entity?->isVisibleUser() ?? $defaults['visibleUser'],
            'visibleStaff' => $entity?->isVisibleStaff() ?? $defaults['visibleStaff'],
            'visibleAdmin' => $entity?->isVisibleAdmin() ?? $defaults['visibleAdmin'],
            'sortOrder' => $entity?->getSortOrder() ?? $defaults['sortOrder'],
            'entity' => $entity,
        ];
    }
}
