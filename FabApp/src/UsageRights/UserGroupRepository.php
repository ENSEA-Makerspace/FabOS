<?php

declare(strict_types=1);

namespace App\UsageRights;

use Doctrine\DBAL\ArrayParameterType;
use Doctrine\DBAL\Connection;

/**
 * L'ÉCRITURE des groupes — celle qui n'existait pas (S158a).
 *
 * 🔴 **Le défaut que ce fichier répare, et il est mesuré.** `USER_GROUP` et
 * `USER_GROUP_MEMBER` existent depuis `Version20260816130000`, qui sème les sept
 * intégrés. Depuis, **rien** ne les écrivait : pas un contrôleur, pas un service.
 * Des sept groupes, seuls comptaient donc ceux qu'un RÔLE implique et l'audience
 * résolue `user` — et « Stagiaires » ou « Bénévoles », que
 * `USAGE_RIGHTS_VISION.md` nomme explicitement, étaient inatteignables. Le
 * formulaire « attribuer à un groupe » d'un forfait était un contrôle dont la
 * portée utile se réglait ailleurs, sur l'écran des rôles.
 *
 * ⚠️ C'est la même famille que `USAGE_RIGHT_ASSIGNMENT.groupId`, qui a vécu deux
 * sessions sans écriture — voir `UsagePackageRepository::assignGroup()`. **Un
 * demi-modèle sans surface d'écriture se lit comme une fonctionnalité et se
 * comporte comme une absence.**
 *
 * ⚠️ **Ce dépôt ÉCRIT ; il ne décide de rien.** L'appartenance effective — celle
 * qui ouvre un droit — reste l'union que `AudienceResolver` calcule : les lignes
 * d'ici, PLUS les rôles, PLUS l'audience `user`. Recalculer cette union ici en
 * serait une deuxième vérité, et elle dériverait le jour où la moitié rôle sort.
 *
 * ⚠️ **`groupKey` est IMMUABLE.** C'est par elle que les forfaits, le résolveur et
 * la migration se retrouvent ; la renommer déplacerait des droits en silence.
 * Le libellé et la description, eux, sont du texte pour les humains.
 */
final class UserGroupRepository
{
    /** ⚠️ Un `groupKey` fait 60 caractères en base ; le libellé 120, la description 255. */
    private const KEY_MAX = 60;

    public function __construct(private readonly Connection $db)
    {
    }

    /**
     * Tous les groupes, avec le nombre de lignes d'appartenance STOCKÉES.
     *
     * ⚠️ **« Stockées », et le mot compte** : ce n'est pas le nombre de personnes
     * que le groupe atteint. Un intégré comme `staff` tire ses membres des rôles
     * et peut afficher 0 ici tout en couvrant tout le monde. L'écran doit montrer
     * les deux, sans quoi il ment — voir `admin-groups.html.twig`.
     *
     * @return list<array{id:int,key:string,label:string,description:?string,builtin:bool,virtual:bool,stored:int,bundles:int}>
     */
    public function all(): array
    {
        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT g.id, g.groupKey, g.label, g.description, g.builtin, g.virtual,
                        (SELECT COUNT(*) FROM USER_GROUP_MEMBER m WHERE m.groupId = g.id) AS stored,
                        (SELECT COUNT(*) FROM USAGE_RIGHT_ASSIGNMENT a
                          WHERE a.groupId = g.id AND a.revokedAt IS NULL) AS bundles
                 FROM USER_GROUP g
                 ORDER BY g.builtin DESC, g.label ASC',
            );
        } catch (\Throwable) {
            // ⚠️ Repli honnête pour une installation sans la migration S133b :
            // aucune ligne, plutôt qu'une erreur sur un écran d'administration.
            return [];
        }

        return array_map(static fn (array $row): array => [
            'id' => (int) $row['id'],
            'key' => (string) $row['groupKey'],
            'label' => (string) $row['label'],
            'description' => $row['description'] !== null ? (string) $row['description'] : null,
            'builtin' => (bool) $row['builtin'],
            'virtual' => (bool) $row['virtual'],
            'stored' => (int) $row['stored'],
            // 🔴 Combien de forfaits perdraient leur attribution si on supprimait
            // ce groupe : `USAGE_RIGHT_ASSIGNMENT.groupId` est en `ON DELETE
            // CASCADE`, donc supprimer un groupe RETIRE des droits. L'écran doit
            // le dire avant, pas after coup.
            'bundles' => (int) $row['bundles'],
        ], $rows);
    }

    /** @return array{id:int,key:string,label:string,description:?string,builtin:bool,virtual:bool,stored:int,bundles:int}|null */
    public function find(int $id): ?array
    {
        foreach ($this->all() as $row) {
            if ($row['id'] === $id) {
                return $row;
            }
        }

        return null;
    }

    /**
     * Créer un groupe libre.
     *
     * ⚠️ **Jamais `builtin`, jamais `virtual`.** Les sept intégrés viennent de la
     * migration et leur statut protégé est une propriété du produit, pas une case
     * qu'un écran offre. Et `virtual` veut dire « résolu depuis le compte, jamais
     * stocké » : un groupe que l'opérateur crée a forcément des membres écrits.
     */
    public function create(string $label, string $description): int
    {
        $label = trim($label);
        if ($label === '') {
            throw new \InvalidArgumentException('Le nom du groupe est obligatoire.');
        }

        $key = $this->uniqueKey($label);
        $this->db->insert('USER_GROUP', [
            'groupKey' => $key,
            'label' => mb_substr($label, 0, 120),
            'description' => mb_substr(trim($description), 0, 255) ?: null,
            'builtin' => 0,
            'virtual' => 0,
            'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
        ]);

        return (int) $this->db->lastInsertId();
    }

    /**
     * Renommer, ou réécrire la description. **La clé ne bouge pas** — c'est par
     * elle que les forfaits retrouvent le groupe.
     */
    public function rename(int $id, string $label, string $description): void
    {
        $label = trim($label);
        if ($label === '') {
            throw new \InvalidArgumentException('Le nom du groupe est obligatoire.');
        }

        $this->db->update('USER_GROUP', [
            'label' => mb_substr($label, 0, 120),
            'description' => mb_substr(trim($description), 0, 255) ?: null,
        ], ['id' => $id]);

        // ⚠️ Pas de test sur `affected_rows` : MySQL rend 0 quand rien ne change,
        // et réenregistrer un libellé identique n'est pas une erreur. La question
        // posée est l'existence — même leçon que `UsagePackageRepository::save()`.
        if (!$this->db->fetchOne('SELECT 1 FROM USER_GROUP WHERE id = :id', ['id' => $id])) {
            throw new \InvalidArgumentException('Groupe introuvable.');
        }
    }

    /**
     * Supprimer un groupe libre.
     *
     * 🔴 **Supprimer un groupe RETIRE des droits, et en silence si on n'y prend
     * pas garde.** `USAGE_RIGHT_ASSIGNMENT.groupId` porte un `ON DELETE CASCADE` :
     * les forfaits attribués à ce groupe perdent leur attribution avec lui. C'est
     * pour ça que `all()` compte `bundles` et que l'écran le dit dans sa
     * confirmation.
     *
     * ⚠️ **Un intégré ne se supprime pas.** Sa clé est nommée par la migration, par
     * le résolveur et par la vision ; le refus est ici, pas seulement dans le
     * gabarit — un POST ne se laisse pas convaincre par un bouton absent.
     */
    public function delete(int $id): void
    {
        $group = $this->find($id);
        if ($group === null) {
            throw new \InvalidArgumentException('Groupe introuvable.');
        }
        if ($group['builtin']) {
            throw new \InvalidArgumentException('Un groupe intégré ne peut pas être supprimé.');
        }

        $this->db->delete('USER_GROUP', ['id' => $id]);
    }

    /**
     * Les identifiants des personnes dont l'appartenance est ÉCRITE dans ce
     * groupe — pas celles qu'un rôle y met.
     *
     * @return list<int>
     */
    public function storedMemberIds(int $groupId): array
    {
        try {
            return array_map('intval', $this->db->fetchFirstColumn(
                'SELECT userId FROM USER_GROUP_MEMBER WHERE groupId = :g',
                ['g' => $groupId],
            ));
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * Les groupes où l'appartenance de cette personne est ÉCRITE — en une requête.
     *
     * ⚠️ L'inverse de `storedMemberIds()`, et il existe pour la fiche du membre :
     * poser la question groupe par groupe y ferait sept requêtes pour une page.
     *
     * @return list<int>
     */
    public function storedGroupIdsFor(int $userId): array
    {
        try {
            return array_map('intval', $this->db->fetchFirstColumn(
                'SELECT groupId FROM USER_GROUP_MEMBER WHERE userId = :u',
                ['u' => $userId],
            ));
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * ⚠️ **Ajouter à un groupe VIRTUEL est refusé.** `user` et `guest` sont
     * résolus depuis le compte et n'ont jamais de ligne : en écrire une créerait
     * une appartenance que le résolveur n'utilise pas, donc un contrôle qui a
     * l'air d'agir et n'agit pas.
     */
    public function addMember(int $groupId, int $userId): void
    {
        $group = $this->find($groupId);
        if ($group === null) {
            throw new \InvalidArgumentException('Groupe introuvable.');
        }
        if ($group['virtual']) {
            throw new \InvalidArgumentException("Cette audience est résolue depuis le compte : elle n'a pas de membres à inscrire.");
        }

        try {
            $this->db->insert('USER_GROUP_MEMBER', [
                'groupId' => $groupId,
                'userId' => $userId,
                'addedAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
            ]);
        } catch (\Throwable) {
            // La clé primaire (groupId, userId) est la garde ; réajouter
            // quelqu'un n'est pas une erreur qui mérite d'arrêter l'opérateur.
            throw new \InvalidArgumentException('Cette personne est déjà dans ce groupe.');
        }
    }

    /**
     * 🔴 **LA GARDE DU DERNIER ADMINISTRATEUR, posée AVANT d'en avoir besoin
     * (S159b).**
     *
     * Aujourd'hui, retirer quelqu'un du groupe `admin` ne lui retire pas
     * `ROLE_ADMIN` : le rôle est la source, le groupe n'en est qu'un reflet. La
     * garde est donc sans effet visible — et c'est exactement pourquoi elle se
     * pose maintenant. Le jour où `getRoles()` lira les groupes, **retirer le
     * dernier membre du groupe `admin` verrouillerait l'installation hors de
     * son propre administrateur**, et ce jour-là il sera trop tard pour y penser.
     *
     * ⚠️ **Elle vit dans le dépôt, pas dans un gabarit.** Les deux écrans qui
     * retirent une appartenance — la fiche du groupe et la fiche du membre —
     * passent par ici, et un POST ne se laisse pas convaincre par un bouton
     * absent. C'est le même choix que le refus de supprimer un groupe intégré.
     *
     * ⚠️ **Elle compte les lignes STOCKÉES du groupe `admin`**, pas les
     * détenteurs de `ROLE_ADMIN` : c'est la chose que ce retrait va modifier, et
     * la seule dont il puisse répondre. `AccountGuard` garde l'autre bout —
     * l'anonymisation d'un compte — et les deux resteront nécessaires.
     */
    public function removeMember(int $groupId, int $userId): void
    {
        $group = $this->find($groupId);
        if ($group !== null && $group['key'] === AudienceResolver::ADMIN && $group['stored'] <= 1) {
            throw new \InvalidArgumentException(
                "Impossible de retirer la dernière personne du groupe des administrateurs : l'installation deviendrait inaccessible.",
            );
        }

        $this->db->delete('USER_GROUP_MEMBER', ['groupId' => $groupId, 'userId' => $userId]);
    }

    /**
     * Combien de lignes stockées par groupe, pour une liste — une requête, pas une
     * par ligne.
     *
     * @param list<int> $groupIds
     * @return array<int, int>
     */
    public function storedCounts(array $groupIds): array
    {
        if ($groupIds === []) {
            return [];
        }
        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT groupId, COUNT(*) AS n FROM USER_GROUP_MEMBER WHERE groupId IN (:ids) GROUP BY groupId',
                ['ids' => $groupIds],
                ['ids' => ArrayParameterType::INTEGER],
            );
        } catch (\Throwable) {
            return [];
        }

        $out = [];
        foreach ($rows as $row) {
            $out[(int) $row['groupId']] = (int) $row['n'];
        }

        return $out;
    }

    /**
     * Une clé stable tirée du libellé, unique en base.
     *
     * ⚠️ Un libellé entièrement non-latin (« Стажёры ») donnerait une clé vide :
     * on retombe alors sur `groupe`, que le suffixe rend unique. Une clé vide
     * casserait l'index unique et, pire, le rapprochement par clé.
     */
    private function uniqueKey(string $label): string
    {
        $base = strtolower((string) preg_replace('/[^a-z0-9]+/i', '-', $this->deaccent($label)));
        $base = trim($base, '-');
        if ($base === '') {
            $base = 'groupe';
        }
        $base = mb_substr($base, 0, self::KEY_MAX - 3);

        $key = $base;
        $n = 2;
        while ($this->db->fetchOne('SELECT 1 FROM USER_GROUP WHERE groupKey = :k', ['k' => $key])) {
            $key = $base . '-' . $n;
            ++$n;
        }

        return $key;
    }

    private function deaccent(string $value): string
    {
        $converted = @iconv('UTF-8', 'ASCII//TRANSLIT', $value);

        return $converted === false ? $value : $converted;
    }
}
