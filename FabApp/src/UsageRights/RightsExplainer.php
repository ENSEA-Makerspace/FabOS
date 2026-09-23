<?php

declare(strict_types=1);

namespace App\UsageRights;

use App\Entity\Badge;
use App\Entity\Formation;
use App\Entity\Machine;
use App\Entity\Utilisateur;
use App\Repository\FormationRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Service\MachineAccessService;
use App\Service\QuizCatalogService;
use App\Service\TrainingQualificationService;
use Doctrine\DBAL\Connection;

/**
 * S192 — « pourquoi cette personne a-t-elle ce droit ? », répondu DEPUIS L'ÉCRAN.
 *
 * 🔴 **Cette classe n'accorde rien et ne refuse rien.** Chaque réponse vient du
 * lecteur qui décide déjà :
 *   - les droits d'usage → `UsageRightsService::verdict()` et `pathsFor()`, qui
 *     pose la question exactement comme le verdict ;
 *   - les machines au badge → `MachineAccessService::reachFor()`, qui suit la
 *     MÊME règle que le scan (`badgeRule()`).
 * Elle ne fait que mettre bout à bout ce qui était su séparément : un forfait, le
 * groupe qui le porte, jusqu'à quand, à quel lieu ; un badge, la formation qui le
 * donne, la date où il a été obtenu, les machines qu'il ouvre.
 *
 * ⚠️ **L'échéance affichée est la PLUS PROCHE des deux** : la fin de
 * l'attribution du forfait au groupe, et la fin de l'appartenance de la personne
 * à ce groupe. Un droit s'arrête dès que l'un des deux maillons tombe.
 *
 * ⚠️ **Aucun identifiant de badge ne sort d'ici** : seulement « enregistré » et
 * ses quatre derniers caractères, pour distinguer deux cartes. Pour beaucoup de
 * cartes bon marché, l'identifiant EST le secret — le connaître suffit à cloner.
 */
final class RightsExplainer
{
    public function __construct(
        private readonly UsageRightsService $rights,
        private readonly MachineAccessService $machineAccess,
        private readonly UtilisateurBadgeRepository $userBadges,
        private readonly FormationRepository $formations,
        private readonly Connection $db,
        private readonly QuizCatalogService $catalog,
        private readonly TrainingQualificationService $qualification,
    ) {
    }

    /**
     * @return array{
     *     active: bool,
     *     admin: bool,
     *     capabilities: list<array{capability: UsageCapability, verdict: UsageRightVerdict, paths: list<array{package: string, via: string, group: ?string, venue: ?string, until: ?string}>}>,
     *     badge: array{registered: bool, hint: ?string},
     *     badges: list<array{badge: Badge, obtainedAt: ?\DateTimeInterface, direct: bool, formations: list<Formation>}>,
     *     open: list<array{machine: Machine, via: list<Badge>}>,
     *     free: list<Machine>,
     *     closed: list<array{machine: Machine, required: list<Badge>}>,
     * }
     */
    public function explain(Utilisateur $user): array
    {
        $memberships = $this->membershipsUntil((int) $user->getId());

        $capabilities = [];
        foreach ($this->rights->overview($user) as $row) {
            $paths = [];
            if ($row['verdict']->allowed && $row['verdict']->reason === 'granted') {
                foreach ($this->rights->pathsFor($user, $row['capability']->key) as $path) {
                    $paths[] = [
                        'package' => $path['package'],
                        'via' => $path['source'],
                        'group' => $path['source'] === 'group' ? ($path['sourceLabel'] !== '' ? $path['sourceLabel'] : $path['groupKey']) : null,
                        'venue' => $path['venue'],
                        'until' => self::earliest($path['until'], $path['groupKey'] !== null ? ($memberships[$path['groupKey']] ?? null) : null),
                    ];
                }
            }
            $capabilities[] = ['capability' => $row['capability'], 'verdict' => $row['verdict'], 'paths' => self::distinct($paths)];
        }

        $badges = [];
        foreach ($this->userBadges->findBy(['utilisateur' => $user]) as $held) {
            $badge = $held->getBadge();
            if ($badge instanceof Badge) {
                // 🔴 S192 — le lecteur ouvre à quiconque POSSÈDE le badge, d'où
                // qu'il vienne ; « Mes badges » ne montre que ceux dont la
                // formation est validée. Un badge attribué à la main, formation
                // non faite, ouvrait donc la machine tout en restant invisible
                // au profil. On le DIT, avec la même mesure que le profil —
                // c'est précisément la réponse à « pourquoi a-t-elle accès ? ».
                $formation = $this->formations->findVisibleByBadge($badge);
                $badges[] = [
                    'badge' => $badge,
                    'obtainedAt' => $held->getDateObtention(),
                    'direct' => $formation !== null && !$this->qualification->getStatus($formation, $user)['badgeUnlocked'],
                    // ⚠️ Les formations INTERNES (supports de quiz, « [FABOS
                    // SECTION] … ») portent aussi le badge : on ne nomme que la
                    // formation qu'un membre peut ouvrir.
                    'formations' => array_values(array_filter(
                        $this->formations->findBy(['badge' => $badge]),
                        fn (Formation $formation): bool => !$this->catalog->isInternalQuizFormation($formation),
                    )),
                ];
            }
        }

        $open = $free = $closed = [];
        foreach ($this->machineAccess->reachFor($user) as $reach) {
            match ($reach['status']) {
                'authorized' => $open[] = ['machine' => $reach['machine'], 'via' => $reach['matched']],
                'no_badge_required' => $free[] = $reach['machine'],
                'missing_badge' => $closed[] = ['machine' => $reach['machine'], 'required' => $reach['required']],
                default => null,
            };
        }

        $rfid = trim((string) $user->getIdentifiantRfid());

        return [
            'active' => $user->getStatut() === 'actif',
            'admin' => in_array('ROLE_ADMIN', $user->getRoles(), true),
            'capabilities' => $capabilities,
            'badge' => [
                'registered' => $rfid !== '',
                'hint' => $rfid !== '' ? self::maskRfid($rfid) : null,
            ],
            'badges' => $badges,
            'open' => $open,
            'free' => $free,
            'closed' => $closed,
        ];
    }

    /** « ••••A1F2 » : assez pour reconnaître SA carte, trop peu pour la copier. */
    public static function maskRfid(string $rfid): string
    {
        $rfid = trim($rfid);

        return mb_strlen($rfid) <= 4 ? '••••' : '••••' . mb_substr($rfid, -4);
    }

    /** @return array<string, ?string> groupKey → fin d'appartenance (null = sans fin) */
    private function membershipsUntil(int $userId): array
    {
        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT g.groupKey, m.validUntil FROM USER_GROUP_MEMBER m INNER JOIN USER_GROUP g ON g.id = m.groupId WHERE m.userId = :user',
                ['user' => $userId],
            );
        } catch (\Throwable) {
            return [];
        }
        $out = [];
        foreach ($rows as $row) {
            $out[(string) $row['groupKey']] = $row['validUntil'] !== null ? (string) $row['validUntil'] : null;
        }

        return $out;
    }

    private static function earliest(?string $a, ?string $b): ?string
    {
        if ($a === null) {
            return $b;
        }
        if ($b === null) {
            return $a;
        }

        return strcmp($a, $b) <= 0 ? $a : $b;
    }

    /**
     * @param list<array{package: string, via: string, group: ?string, venue: ?string, until: ?string}> $paths
     *
     * @return list<array{package: string, via: string, group: ?string, venue: ?string, until: ?string}>
     */
    private static function distinct(array $paths): array
    {
        $seen = [];
        foreach ($paths as $path) {
            $seen[json_encode($path)] = $path;
        }

        return array_values($seen);
    }
}
