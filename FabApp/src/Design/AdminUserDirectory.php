<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\Utilisateur;
use App\Service\QuizCatalogService;
use App\Repository\AccessRfidLogRepository;
use App\Repository\FormationRepository;
use App\Repository\ProgressionRepository;
use App\Repository\UtilisateurRepository;
use App\Service\TrainingQualificationService;
use Doctrine\DBAL\Connection;

/**
 * L'annuaire des utilisateurs, orienté TRAVAIL (proposition
 * `annuaire-utilisateurs`, 2026-10-01, d'après la planche
 * `users/08-annuaire-utilisateurs.png`).
 *
 * ⚠️ LECTURE SEULE, rien de neuf : mêmes comptes que `/admin/utilisateurs`
 * (`UtilisateurRepository::findForAdminFilters`), mêmes formations visibles que
 * la progression de la liste actuelle (`countVisible` / catégories internes
 * exclues). Ce qui s'ajoute : le tri en tuiles de travail, la dernière
 * activité, le badge enregistré et l'échéance d'une appartenance de groupe.
 *
 * 🔴 « Expire bientôt » = une appartenance de GROUPE (`USER_GROUP_MEMBER.validUntil`)
 * qui prend fin dans les 30 jours. Il n'existe pas de « date d'adhésion » sur
 * le compte : la planche parle d'adhésion, FabOS n'a que des appartenances à
 * durée limitée.
 *
 * Horodatages : tout ce qui est rendu ici est un horodatage machine (UTC) →
 * `|lab_date` côté gabarit.
 */
final class AdminUserDirectory
{
    private const SOON_DAYS = 30;

    /** Ordre et libellés des tuiles. */
    private const TILES = [
        '' => 'Tous',
        'valider' => 'À valider',
        'suspendus' => 'Suspendus ou inactifs',
        'expire' => 'Expire bientôt',
        'sans-badge' => 'Sans badge',
    ];

    public function __construct(
        private readonly UtilisateurRepository $users,
        private readonly AccessRfidLogRepository $rfidLogs,
        private readonly ProgressionRepository $progressions,
        private readonly FormationRepository $formations,
        private readonly Connection $db,
    ) {
    }

    /**
     * @return array{tiles: list<array<string, mixed>>, rows: list<array<string, mixed>>, total: int, shown: int, tile: string, q: string, formationsTotal: int}
     */
    public function build(string $tile, string $q): array
    {
        $tile = array_key_exists($tile, self::TILES) ? $tile : '';
        $q = trim($q);

        $all = $this->users->findForAdminFilters(['q' => '', 'statut' => '', 'package' => '', 'groupe' => '']);
        $lastPass = $this->lastPass();
        $done = $this->completedCounts();
        $expiring = $this->expiringMemberships();

        $rows = [];
        foreach ($all as $user) {
            $rows[] = $this->row($user, $lastPass, $done, $expiring);
        }

        $counts = array_fill_keys(array_keys(self::TILES), 0);
        foreach ($rows as $row) {
            ++$counts[''];
            foreach ($row['tiles'] as $key) {
                ++$counts[$key];
            }
        }

        $tiles = [];
        foreach (self::TILES as $key => $label) {
            $tiles[] = [
                'label' => $label,
                'count' => $counts[$key],
                'query' => ['tuile' => $key],
                'active' => $key === $tile,
            ];
        }

        $shown = array_values(array_filter($rows, static function (array $row) use ($tile, $q): bool {
            if ($tile !== '' && !in_array($tile, $row['tiles'], true)) {
                return false;
            }

            return $q === '' || mb_stripos($row['name'] . ' ' . $row['email'], $q) !== false;
        }));

        return [
            'tiles' => $tiles,
            'rows' => $shown,
            'total' => count($rows),
            'shown' => count($shown),
            'tile' => $tile,
            'q' => $q,
            'formationsTotal' => $this->formations->countVisible(),
        ];
    }

    /**
     * @param array<int, \DateTimeImmutable> $lastPass
     * @param array<int, int> $done
     * @param array<int, array{until: \DateTimeImmutable, group: string}> $expiring
     * @return array<string, mixed>
     */
    private function row(Utilisateur $user, array $lastPass, array $done, array $expiring): array
    {
        $id = (int) $user->getId();
        $statut = mb_strtolower(trim($user->getStatut()));
        $pending = in_array($statut, ['pending', 'en attente'], true);
        $suspended = in_array($statut, ['inactif', 'inactive', 'banned', 'banni'], true);
        $active = !$pending && !$suspended;
        $unconfirmed = $user->getStatusKey() === 'user_status.unconfirmed';
        $hasBadge = trim((string) $user->getIdentifiantRfid()) !== '';
        $soon = $expiring[$id] ?? null;

        $tiles = [];
        if ($pending) {
            $tiles[] = 'valider';
        }
        if ($suspended) {
            $tiles[] = 'suspendus';
        }
        if ($soon !== null) {
            $tiles[] = 'expire';
        }
        if ($active && !$hasBadge) {
            $tiles[] = 'sans-badge';
        }

        // Dernière activité : le plus récent de la dernière connexion et du
        // dernier passage de badge. Deux horodatages machine, donc comparables.
        $login = $user->getDerniereConnexion();
        $pass = $lastPass[$id] ?? null;
        $last = null;
        $lastKind = '';
        if ($pass !== null && ($login === null || $pass >= $login)) {
            [$last, $lastKind] = [$pass, 'Badge'];
        } elseif ($login !== null) {
            [$last, $lastKind] = [$login, 'Connexion'];
        }

        // Le signal et le texte de statut : le même mapping que la liste actuelle.
        $signal = $unconfirmed || $pending ? 'caution' : ($statut === 'actif' ? 'go' : (in_array($statut, ['banned', 'banni'], true) ? 'stop' : 'muted'));
        $note = match (true) {
            $unconfirmed => 'Adresse non confirmée',
            $pending => 'En attente de validation',
            $suspended => 'Accès coupé',
            $soon !== null => $soon['group'],
            default => '',
        };

        return [
            'id' => $id,
            'name' => $user->getDisplayName(),
            'email' => (string) $user->getEmail(),
            'initials' => mb_substr((string) ($user->getFirstName() ?? $user->getUsername()), 0, 1) . mb_substr((string) $user->getLastName(), 0, 1),
            'statusKey' => $user->getStatusKey(),
            'signal' => $signal,
            'note' => $note,
            'until' => $soon['until'] ?? null,
            'hasBadge' => $hasBadge,
            'trainingsDone' => $done[$id] ?? 0,
            'last' => $last,
            'lastKind' => $lastKind,
            'tiles' => $tiles,
            // « Examiner » = il y a quelque chose à décider ; « Ouvrir » sinon.
            'examine' => $pending || $suspended || $unconfirmed || $soon !== null,
        ];
    }

    /** @return array<int, \DateTimeImmutable> id utilisateur → dernier passage de badge */
    private function lastPass(): array
    {
        $out = [];
        foreach ($this->rfidLogs->createQueryBuilder('log')
            ->select('IDENTITY(log.utilisateur) AS userId, MAX(log.createdAt) AS lastAt')
            ->where('log.utilisateur IS NOT NULL')
            ->groupBy('log.utilisateur')
            ->getQuery()
            ->getArrayResult() as $row) {
            $out[(int) $row['userId']] = new \DateTimeImmutable((string) $row['lastAt']);
        }

        return $out;
    }

    /** @return array<int, int> id utilisateur → formations visibles terminées */
    private function completedCounts(): array
    {
        $out = [];
        foreach ($this->progressions->createQueryBuilder('progression')
            ->select('IDENTITY(progression.utilisateur) AS userId, COUNT(progression.id) AS done')
            ->innerJoin('progression.formation', 'formation')
            ->andWhere('progression.completed = :completed')
            ->andWhere('formation.archivedAt IS NULL')
            ->andWhere('formation.categorie IS NULL OR formation.categorie NOT IN (:internal)')
            ->setParameter('completed', true)
            ->setParameter('internal', [
                QuizCatalogService::INTERNAL_CATEGORY,
                TrainingQualificationService::PHYSICAL_CATEGORY,
            ])
            ->groupBy('progression.utilisateur')
            ->getQuery()
            ->getArrayResult() as $row) {
            $out[(int) $row['userId']] = (int) $row['done'];
        }

        return $out;
    }

    /**
     * Appartenances de groupe qui prennent fin dans les 30 jours (la plus proche
     * par personne).
     *
     * @return array<int, array{until: \DateTimeImmutable, group: string}>
     */
    private function expiringMemberships(): array
    {
        $now = new \DateTimeImmutable('now', new \DateTimeZone('UTC'));
        $out = [];
        foreach ($this->db->fetchAllAssociative(
            'SELECT m.userId, m.validUntil, g.label FROM USER_GROUP_MEMBER m
               INNER JOIN USER_GROUP g ON g.id = m.groupId
              WHERE m.validUntil IS NOT NULL AND m.validUntil > :now AND m.validUntil <= :soon
              ORDER BY m.validUntil DESC',
            [
                'now' => $now->format('Y-m-d H:i:s'),
                'soon' => $now->modify('+' . self::SOON_DAYS . ' days')->format('Y-m-d H:i:s'),
            ],
        ) as $row) {
            // ORDER BY DESC puis écrasement : la dernière écrite est la plus proche.
            $out[(int) $row['userId']] = [
                'until' => new \DateTimeImmutable((string) $row['validUntil']),
                'group' => 'Groupe « ' . $row['label'] . ' »',
            ];
        }

        return $out;
    }
}
