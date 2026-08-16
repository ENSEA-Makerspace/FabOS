<?php

declare(strict_types=1);

namespace App\UsageRights;

use App\Entity\Utilisateur;
use Doctrine\DBAL\Connection;

/**
 * Which groups a person is in — resolved audiences included (S133b).
 *
 * Two kinds of membership, and conflating them is the mistake this class exists
 * to prevent:
 *
 *  - **Stored.** A row in `USER_GROUP_MEMBER`. Staff, Formateurs, Manager, Super
 *    user and any group the lab invents.
 *  - **Resolved.** `user` is every active local account and `guest` is nobody
 *    signed in. Neither ever gets a membership row: "every account" is not a list
 *    somebody maintains, and a provisioning step that forgets to add the row
 *    would silently strip a member of the baseline audience. The vision document
 *    is explicit about this and it is worth restating in code.
 *
 * ⚠️ **Roles seed the built-ins, they do not replace them.** `ROLE_STAFF` puts
 * somebody in `staff` today because that is where the truth currently lives; a
 * stored membership row does so as well, and the union is the answer. When S134
 * moves the truth into groups, the role half comes out and nothing else has to
 * change.
 *
 * ⚠️ Fail-safe on read, like every other table that arrives by migration: an
 * install running this code before the S133b migration resolves the audiences it
 * can compute and no stored ones, rather than throwing on an admin screen.
 */
final class AudienceResolver
{
    public const ADMIN = 'admin';
    public const MANAGER = 'manager';
    public const STAFF = 'staff';
    public const SUPERUSER = 'superuser';
    public const TRAINERS = 'trainers';
    /** Resolved, never stored — every active local account. */
    public const USER = 'user';
    /** Resolved, never stored — nobody signed in. */
    public const GUEST = 'guest';

    /** The seven the interface shows as protected, in the vision document's order. */
    public const BUILTIN = [self::ADMIN, self::MANAGER, self::STAFF, self::SUPERUSER, self::TRAINERS, self::USER, self::GUEST];

    /** @var array<string, list<string>> */
    private array $memo = [];

    public function __construct(private readonly Connection $db)
    {
    }

    /**
     * Every group key that applies, resolved audiences first.
     *
     * @return list<string>
     */
    public function keysFor(?Utilisateur $user): array
    {
        if (!$user instanceof Utilisateur || $user->getId() === null) {
            return [self::GUEST];
        }

        $id = (string) $user->getId();
        if (isset($this->memo[$id])) {
            return $this->memo[$id];
        }

        $roles = $user->getRoles();
        $keys = [self::USER];

        foreach ([
            'ROLE_ADMIN' => self::ADMIN,
            'ROLE_MANAGER' => self::MANAGER,
            'ROLE_STAFF' => self::STAFF,
            'ROLE_SUPER_USER' => self::SUPERUSER,
            // Both spellings are in this codebase's history and neither is
            // authoritative enough to drop; a member in either is a trainer.
            'ROLE_TRAINER' => self::TRAINERS,
            'ROLE_FORMATEUR' => self::TRAINERS,
        ] as $role => $key) {
            if (in_array($role, $roles, true)) {
                $keys[] = $key;
            }
        }

        foreach ($this->storedKeysFor((int) $user->getId()) as $key) {
            $keys[] = $key;
        }

        return $this->memo[$id] = array_values(array_unique($keys));
    }

    /**
     * The seven built-ins plus whatever the lab has added, for a screen.
     *
     * @return list<array{key: string, label: string, description: ?string, builtin: bool, virtual: bool, members: int}>
     */
    public function catalogue(): array
    {
        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT g.groupKey, g.label, g.description, g.builtin, g.virtual,
                        (SELECT COUNT(*) FROM USER_GROUP_MEMBER m WHERE m.groupId = g.id) AS members
                 FROM USER_GROUP g ORDER BY g.builtin DESC, g.label ASC',
            );
        } catch (\Throwable) {
            return [];
        }

        return array_map(static fn (array $row): array => [
            'key' => (string) $row['groupKey'],
            'label' => (string) $row['label'],
            'description' => $row['description'] !== null ? (string) $row['description'] : null,
            'builtin' => (bool) $row['builtin'],
            'virtual' => (bool) $row['virtual'],
            'members' => (int) $row['members'],
        ], $rows);
    }

    /** @return list<string> */
    private function storedKeysFor(int $userId): array
    {
        try {
            return array_map('strval', $this->db->fetchFirstColumn(
                'SELECT g.groupKey FROM USER_GROUP_MEMBER m INNER JOIN USER_GROUP g ON g.id = m.groupId WHERE m.userId = :user',
                ['user' => $userId],
            ));
        } catch (\Throwable) {
            return [];
        }
    }
}
