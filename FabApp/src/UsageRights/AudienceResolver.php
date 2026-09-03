<?php

declare(strict_types=1);

namespace App\UsageRights;

use App\Entity\Utilisateur;
use App\Reservation\LabClock;
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

    /**
     * ⚠️ **S159g — le résolveur dépend désormais de L'INSTANT.** Une appartenance
     * peut être datée, donc « dans quels groupes est cette personne » n'a de
     * réponse que « maintenant ». Sa mémoïsation PAR REQUÊTE reste juste — une
     * requête ne dure pas assez pour qu'une date bascule — mais une mémoïsation
     * plus longue, sur un worker par exemple, ne le serait plus.
     */
    /**
     * 🔴 **`LabClock`, et pas `new \DateTimeImmutable()`.** Les bornes
     * d'appartenance sont de « convention B » au sens de `LabClock` : l'heure
     * MURALE du labo, stockée telle quelle. Les comparer à un `now` en UTC — ce
     * que faisait `storedKeysFor()` — décale la fenêtre de l'offset du labo, deux
     * heures à Paris en été, et **toujours dans le sens permissif** : une
     * appartenance expirée reste vue comme valide pendant ces deux heures. C'est
     * exactement le balayage que `LabClock` dit « consigné, pas fait ».
     *
     * ⚠️ Facultative pour que les commandes qui construisent un résolveur NEUF à
     * la main continuent de marcher ; sans elle, on retombe sur l'ancien
     * comportement — celui d'avant, jamais un neuf à moitié.
     */
    public function __construct(
        private readonly Connection $db,
        private readonly UserGroupSchema $schema,
        private readonly ?LabClock $clock = null,
    ) {
    }

    /** L'instant à comparer aux bornes stockées : heure murale du labo. */
    private function now(): \DateTimeImmutable
    {
        return $this->clock?->now() ?? new \DateTimeImmutable();
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

        return $this->memo[$id] = $this->compute($user, $this->storedKeysFor((int) $user->getId()));
    }

    /**
     * Pré-résoudre TOUTE une liste de personnes en une seule requête (S153c).
     *
     * ⚠️ **Pourquoi ça existe** : `keysFor()` interroge `USER_GROUP_MEMBER` une
     * fois par personne, ce qui est juste pour une question isolée et devient une
     * requête par ligne sur un écran qui en pose deux cents. Le filtre « droit
     * d'usage » de la liste des utilisateurs pose exactement cette question-là.
     *
     * 🔴 **Et ça ne DUPLIQUE pas la règle d'appartenance** : les deux chemins
     * passent par `compute()`. Une seconde copie du tableau rôle → groupe est
     * précisément ce qui aurait dérivé le jour où S134 déplace la vérité dans les
     * groupes — le commentaire en tête de cette classe le dit déjà.
     *
     * @param iterable<Utilisateur> $users
     */
    public function primeFor(iterable $users): void
    {
        $pending = [];
        foreach ($users as $user) {
            $id = $user->getId();
            if ($id !== null && !isset($this->memo[(string) $id])) {
                $pending[(int) $id] = $user;
            }
        }
        if ($pending === []) {
            return;
        }

        $stored = array_fill_keys(array_keys($pending), []);
        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT m.userId, g.groupKey FROM USER_GROUP_MEMBER m
                 INNER JOIN USER_GROUP g ON g.id = m.groupId
                 WHERE m.userId IN (:ids)' . $this->schema->activeClause('m'),
                ['ids' => array_keys($pending)] + $this->schema->activeParams(new \DateTimeImmutable()),
                ['ids' => \Doctrine\DBAL\ArrayParameterType::INTEGER],
            );
            foreach ($rows as $row) {
                $stored[(int) $row['userId']][] = (string) $row['groupKey'];
            }
        } catch (\Throwable) {
            // Même repli que `storedKeysFor()` : les audiences calculables
            // restent, les stockées manquent. Jamais une exception sur un écran.
        }

        foreach ($pending as $id => $user) {
            $this->memo[(string) $id] = $this->compute($user, $stored[$id] ?? []);
        }
    }

    /*
     * 🔴 **`roleKeysFor()` a été RETIRÉE en S159h, et son retrait est une
     * correction, pas un ménage.**
     *
     * Elle répondait « cette personne resterait-elle dans ce groupe si la ligne
     * disparaissait ? », en calculant les audiences que les RÔLES donnent. Elle
     * avait un sens tant que les rôles étaient une source indépendante. Après le
     * contract, `getRoles()` dérive des appartenances : la question devenait
     * CIRCULAIRE — elle répondait toujours oui pour un groupe intégré, parce que
     * le rôle venait de la ligne même dont on demandait si elle était nécessaire.
     *
     * Conséquence mesurée à l'écran : le bouton « Retirer » disparaissait de
     * TOUTES les appartenances aux groupes intégrés, et la ligne affichait
     * « par son rôle » — une phrase devenue fausse, `UTILISATEUR_ROLE` n'existant
     * plus. Plus personne ne pouvait être retiré de `staff` depuis l'écran des
     * groupes.
     *
     * ⚠️ Il n'y a plus de distinction de source à faire : pour un groupe non
     * virtuel, l'appartenance est toujours une ligne, et elle se retire.
     */


    /**
     * L'union des trois appartenances : l'audience résolue `user`, celles que les
     * rôles impliquent, et les lignes stockées.
     *
     * @param list<string> $storedKeys
     * @return list<string>
     */
    private function compute(Utilisateur $user, array $storedKeys): array
    {
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

        foreach ($storedKeys as $key) {
            $keys[] = $key;
        }

        return array_values(array_unique($keys));
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

    /**
     * L'INVERSE de `keysFor()` : qui est dans ce groupe ? (S144e)
     *
     * 🔴 **Pourquoi il fallait l'écrire, et pourquoi maintenant.** L'aperçu
     * d'activation de `/admin/settings` comptait `COUNT(DISTINCT a.userId)` sur
     * les attributions — c'est-à-dire les seules attributions PERSONNELLES. Depuis
     * S159, la seule surface d'écriture humaine est l'attribution à un GROUPE, et
     * la conversion a déplacé les trois dernières lignes personnelles : toutes les
     * attributions vivantes ont donc `userId = NULL`, et `COUNT(DISTINCT NULL)`
     * vaut **zéro**. Mesuré à l'écran le 2026-09-03 : « 4 forfaits actifs couvrent
     * 0 membres », et zéro pour les quatre capacités, pendant que l'enforcement
     * décide réellement. Un aperçu d'activation qui annonce toujours zéro n'est
     * pas prudent, il est mort — et il invite à conclure de travers.
     *
     * 🔴 **Et c'est un INVERSE, pas une seconde règle d'appartenance.** Écrire ici
     * un second tableau rôle → groupe, ou une seconde clause de dates, ferait
     * exactement ce que l'en-tête de cette classe interdit : deux copies qui
     * dérivent. La sonde `app:s153:package-probe` vérifie donc les DEUX sens sur
     * chaque compte — `k ∈ keysFor(p)` ⟺ `p ∈ memberIdsFor(k)` — et c'est ce
     * contrôle, pas ce commentaire, qui tient la promesse.
     *
     * ⚠️ `guest` ne rend JAMAIS personne : c'est l'audience de qui n'a pas de
     * compte. Rendre la liste des comptes serait l'exact contraire de son sens.
     * ⚠️ `user` rend TOUS les comptes, sans ligne d'appartenance — `compute()`
     * l'accorde à toute personne qu'on lui passe, sans condition.
     *
     * @return list<int>
     */
    public function memberIdsFor(string $groupKey, ?\DateTimeImmutable $at = null): array
    {
        if ($groupKey === self::GUEST) {
            return [];
        }

        try {
            if ($groupKey === self::USER) {
                return array_map('intval', $this->db->fetchFirstColumn('SELECT id FROM UTILISATEUR'));
            }

            return array_map('intval', $this->db->fetchFirstColumn(
                'SELECT m.userId FROM USER_GROUP_MEMBER m INNER JOIN USER_GROUP g ON g.id = m.groupId
                 WHERE g.groupKey = :key' . $this->schema->activeClause('m'),
                ['key' => $groupKey] + $this->schema->activeParams($at ?? $this->now()),
            ));
        } catch (\Throwable) {
            // Même repli que `storedKeysFor()` : une installation d'avant la
            // migration S133b rend « personne », pas une erreur sur un écran.
            return [];
        }
    }

    /** @return list<string> */
    private function storedKeysFor(int $userId): array
    {
        try {
            return array_map('strval', $this->db->fetchFirstColumn(
                'SELECT g.groupKey FROM USER_GROUP_MEMBER m INNER JOIN USER_GROUP g ON g.id = m.groupId
                 WHERE m.userId = :user' . $this->schema->activeClause('m'),
                ['user' => $userId] + $this->schema->activeParams($this->now()),
            ));
        } catch (\Throwable) {
            return [];
        }
    }
}
