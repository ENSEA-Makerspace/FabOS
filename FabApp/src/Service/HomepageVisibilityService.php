<?php

namespace App\Service;

use App\Entity\HomepageSectionVisibility;
use App\Repository\HomepageSectionVisibilityRepository;
use Doctrine\DBAL\Exception as DbalException;
use Doctrine\ORM\Exception\ORMException;
use Symfony\Component\Security\Core\User\UserInterface;

class HomepageVisibilityService
{
    public const ROLE_ANONYMOUS = 'anonymous';
    public const ROLE_USER = 'user';
    public const ROLE_STAFF = 'staff';
    public const ROLE_ADMIN = 'admin';

    /**
     * @var array<string, array{label: string, visibleAnonymous: bool, visibleUser: bool, visibleStaff: bool, visibleAdmin: bool, sortOrder: int}>
     */
    private const DEFAULT_SECTIONS = [
        'opening_hours' => [
            'label' => 'Horaires d’ouverture',
            'visibleAnonymous' => true,
            'visibleUser' => true,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 10,
        ],
        'how_it_works' => [
            'label' => 'Comment ça fonctionne ?',
            'visibleAnonymous' => true,
            'visibleUser' => true,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 20,
        ],
        'fablab_stats' => [
            'label' => 'Statistiques FabLab',
            'visibleAnonymous' => false,
            'visibleUser' => false,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 30,
        ],
        'mini_leaderboard' => [
            'label' => 'Mini classement',
            'visibleAnonymous' => false,
            'visibleUser' => true,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 40,
        ],
        'featured_machines' => [
            'label' => 'Machines à découvrir',
            'visibleAnonymous' => true,
            'visibleUser' => true,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 50,
        ],
        'latest_rfid_logs' => [
            'label' => 'Derniers passages RFID',
            'visibleAnonymous' => false,
            'visibleUser' => false,
            'visibleStaff' => true,
            'visibleAdmin' => true,
            'sortOrder' => 60,
        ],
    ];

    public function __construct(private readonly HomepageSectionVisibilityRepository $sections)
    {
    }

    /** @return array<string, array{label: string, visibleAnonymous: bool, visibleUser: bool, visibleStaff: bool, visibleAdmin: bool, sortOrder: int}> */
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
            $visibility[$row['sectionKey']] = match ($role) {
                self::ROLE_ADMIN => $row['visibleAdmin'],
                self::ROLE_STAFF => $row['visibleStaff'],
                self::ROLE_USER => $row['visibleUser'],
                default => $row['visibleAnonymous'],
            };
        }

        return $visibility;
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
     * @param array{label: string, visibleAnonymous: bool, visibleUser: bool, visibleStaff: bool, visibleAdmin: bool, sortOrder: int} $defaults
     * @return array{sectionKey: string, label: string, visibleAnonymous: bool, visibleUser: bool, visibleStaff: bool, visibleAdmin: bool, sortOrder: int, entity: ?HomepageSectionVisibility}
     */
    private function buildRow(string $sectionKey, array $defaults, ?HomepageSectionVisibility $entity): array
    {
        return [
            'sectionKey' => $sectionKey,
            'label' => $entity?->getLabel() ?: $defaults['label'],
            'visibleAnonymous' => $entity?->isVisibleAnonymous() ?? $defaults['visibleAnonymous'],
            'visibleUser' => $entity?->isVisibleUser() ?? $defaults['visibleUser'],
            'visibleStaff' => $entity?->isVisibleStaff() ?? $defaults['visibleStaff'],
            'visibleAdmin' => $entity?->isVisibleAdmin() ?? $defaults['visibleAdmin'],
            'sortOrder' => $entity?->getSortOrder() ?? $defaults['sortOrder'],
            'entity' => $entity,
        ];
    }
}
