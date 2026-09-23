<?php

namespace App\Training;

use App\Entity\Formation;
use App\Entity\Utilisateur;
use App\Mail\Mailer;
use App\Mail\NotificationCategory;
use App\Service\SiteSettingService;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Psr\Log\LoggerInterface;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;

/**
 * Le fil PRIVÉ entre un apprenant et l'équipe de formation (S183b).
 *
 * 🔴 **Un fil par (formation, apprenant), et un fil ne contient qu'UN apprenant.**
 * L'invariant de la phase — « aucun message privé ne bascule implicitement vers
 * la cohorte » — est donc une propriété du SCHÉMA, pas une règle qu'un écran
 * devrait penser à appliquer. Il n'y a aucun fil à plusieurs apprenants, donc
 * aucune requête qui pourrait en élargir un. C'est le même raisonnement qu'à S183
 * pour les annonces, où `queueToUser()` ne prend qu'un destinataire.
 *
 * 🔴 **L'équipe, c'est le groupe `trainers`, pas « les administrateurs ».**
 * Mesuré le 2026-09-23 : chaque formation porte « Équipe FabLab » comme
 * formateur — un libellé, pas une personne. Le seul modèle fidèle est une boîte
 * d'ÉQUIPE, et l'équipe qui existe est le groupe `trainers` (→ `ROLE_TRAINER`),
 * que l'opérateur gère. Administrer n'est pas un droit de lecture sur les
 * messages privés d'un apprenant.
 *
 * 🔴 **FabOS est la source ; l'e-mail est une COPIE.** Le message est écrit en
 * base d'abord, et les copies partent ensuite dans un `try` : une panne d'envoi
 * ne perd jamais le message, qui reste lisible dans le fil avec son compteur de
 * non-lus. C'est la règle écrite dans le plan de l'ancienne Phase I.
 *
 * ⚠️ **DBAL et tolérant à l'absence des tables**, comme `MailOverrides` : le code
 * se déploie avant la migration, et sans les tables le lien « Écrire à
 * l'équipe » n'apparaît simplement pas.
 */
final class FormationThreads
{
    /** ⚠️ Au-delà, un « message » est un document ; il a sa place ailleurs. */
    public const MAX_LENGTH = 4000;

    private ?bool $storageReady = null;

    public function __construct(
        private readonly Connection $db,
        private readonly EntityManagerInterface $em,
        private readonly CohortAnnouncer $cohort,
        private readonly Mailer $mailer,
        private readonly SiteSettingService $settings,
        private readonly UrlGeneratorInterface $urls,
        private readonly LoggerInterface $logger,
    ) {
    }

    public function isAvailable(): bool
    {
        return $this->isStorageReady();
    }

    /**
     * ⚠️ `getRoles()` et pas `isGranted()` : ce service sert aussi à décider qui
     * reçoit une copie par e-mail, c'est-à-dire pour des personnes qui ne sont
     * PAS celle de la requête en cours.
     */
    public function isTrainer(Utilisateur $user): bool
    {
        return in_array('ROLE_TRAINER', $user->getRoles(), true) && $user->getStatut() === 'actif';
    }

    /** Un apprenant peut-il écrire à l'équipe de cette formation ? */
    public function canWrite(Formation $formation, Utilisateur $user): bool
    {
        return $this->isStorageReady() && $this->cohort->isMember($formation, $user);
    }

    /**
     * Le fil de cet apprenant pour cette formation — créé s'il le faut.
     *
     * 🔴 **`INSERT IGNORE` sur la contrainte d'unicité, puis relecture.** « Lire
     * puis créer » laisse entre les deux une fenêtre où deux onglets ouvrent
     * deux fils : l'équipe répondrait dans l'un pendant que l'apprenant attend
     * dans l'autre. C'est la base qui tranche.
     *
     * @return array<string, mixed>|null
     */
    public function threadFor(Formation $formation, Utilisateur $learner, bool $create): ?array
    {
        if (!$this->isStorageReady() || $formation->getId() === null || $learner->getId() === null) {
            return null;
        }

        try {
            if ($create) {
                $this->db->executeStatement(
                    'INSERT IGNORE INTO FORMATION_THREAD (formationId, learnerId, createdAt) VALUES (?, ?, NOW())',
                    [$formation->getId(), $learner->getId()],
                );
            }

            $row = $this->db->fetchAssociative(
                'SELECT * FROM FORMATION_THREAD WHERE formationId = ? AND learnerId = ?',
                [$formation->getId(), $learner->getId()],
            );
        } catch (\Throwable) {
            return null;
        }

        return is_array($row) ? $row : null;
    }

    /** @return array<string, mixed>|null */
    public function find(int $threadId): ?array
    {
        if (!$this->isStorageReady()) {
            return null;
        }

        try {
            $row = $this->db->fetchAssociative('SELECT * FROM FORMATION_THREAD WHERE id = ?', [$threadId]);
        } catch (\Throwable) {
            return null;
        }

        return is_array($row) ? $row : null;
    }

    /**
     * Qui peut LIRE ce fil : son apprenant, et l'équipe. Personne d'autre.
     *
     * 🔴 **Pas « les administrateurs ».** Un fil privé l'est vis-à-vis du reste
     * du labo, administration comprise ; seuls ceux qui y répondent le lisent.
     *
     * @param array<string, mixed> $thread
     */
    public function canRead(array $thread, Utilisateur $viewer): bool
    {
        return (int) $thread['learnerId'] === (int) $viewer->getId() || $this->isTrainer($viewer);
    }

    /** @return list<array<string, mixed>> les plus anciens d'abord */
    public function messages(int $threadId): array
    {
        try {
            return $this->db->fetchAllAssociative(
                'SELECT m.*, u.firstName, u.lastName FROM FORMATION_THREAD_MESSAGE m
                 LEFT JOIN UTILISATEUR u ON u.id = m.authorId
                 WHERE m.threadId = ? ORDER BY m.createdAt ASC, m.id ASC',
                [$threadId],
            );
        } catch (\Throwable) {
            return [];
        }
    }

    /**
     * Écrit un message, PUIS envoie les copies.
     *
     * 🔴 **Dans cet ordre, et les copies ne peuvent pas faire échouer l'écriture.**
     * Une panne d'envoi ne perd jamais le message interne — c'est la règle du
     * plan, et c'est ce qui fait de l'e-mail une copie plutôt que le canal.
     *
     * ⚠️ **Un destinataire à la fois, jamais une liste**, comme les annonces :
     * quand un apprenant écrit, chaque formateur reçoit SON e-mail, et aucun ne
     * voit l'adresse d'un autre.
     *
     * @param array<string, mixed> $thread
     */
    public function post(array $thread, Formation $formation, Utilisateur $author, string $body): ?int
    {
        $body = trim($body);
        if ($body === '' || mb_strlen($body) > self::MAX_LENGTH || !mb_check_encoding($body, 'UTF-8')) {
            return null;
        }

        $threadId = (int) $thread['id'];

        $messageId = 0;

        try {
            // ⚠️ L'identifiant est lu JUSTE après l'`INSERT`, dans la transaction :
            // le lire après la mise à jour du fil marcherait par hasard de moteur.
            $this->db->transactional(function () use ($threadId, $author, $body, &$messageId): void {
                $this->db->executeStatement(
                    'INSERT INTO FORMATION_THREAD_MESSAGE (threadId, authorId, body, createdAt) VALUES (?, ?, ?, NOW())',
                    [$threadId, $author->getId(), $body],
                );
                $messageId = (int) $this->db->lastInsertId();
                $this->db->executeStatement('UPDATE FORMATION_THREAD SET lastMessageAt = NOW() WHERE id = ?', [$threadId]);
            });
        } catch (\Throwable $e) {
            $this->logger->error('Un message de fil de formation n\'a pas pu être écrit.', ['thread' => $threadId, 'error' => $e->getMessage()]);

            return null;
        }

        // ⚠️ L'auteur a lu son propre message, évidemment : sans ça il verrait
        // son envoi compté comme « non lu » dans sa propre boîte.
        $this->markRead($threadId, (int) $author->getId());

        try {
            $this->sendCopies($thread, $formation, $author, $body);
        } catch (\Throwable $e) {
            $this->logger->warning('Les copies e-mail d\'un message de fil n\'ont pas pu partir ; le message est intact.', [
                'thread' => $threadId,
                'error' => $e->getMessage(),
            ]);
        }

        return $messageId;
    }

    public function markRead(int $threadId, int $userId): void
    {
        try {
            $this->db->executeStatement(
                'INSERT INTO FORMATION_THREAD_READ (threadId, userId, lastReadAt) VALUES (?, ?, NOW())
                 ON DUPLICATE KEY UPDATE lastReadAt = NOW()',
                [$threadId, $userId],
            );
        } catch (\Throwable) {
            // Un compteur de non-lus faux d'un cran vaut mieux qu'une page qui tombe.
        }
    }

    /**
     * Les messages non lus de ce fil pour cette personne.
     *
     * ⚠️ **Ses propres messages ne comptent jamais**, et « jamais ouvert » veut
     * dire « tout est non lu », pas « rien ».
     */
    public function unread(int $threadId, int $userId): int
    {
        try {
            return (int) $this->db->fetchOne(
                'SELECT COUNT(*) FROM FORMATION_THREAD_MESSAGE m
                 LEFT JOIN FORMATION_THREAD_READ r ON r.threadId = m.threadId AND r.userId = ?
                 WHERE m.threadId = ? AND m.authorId <> ? AND (r.lastReadAt IS NULL OR m.createdAt > r.lastReadAt)',
                [$userId, $threadId, $userId],
            );
        } catch (\Throwable) {
            return 0;
        }
    }

    /**
     * La boîte de l'équipe : tous les fils, les plus récents d'abord, avec les
     * non-lus DE CELUI QUI REGARDE.
     *
     * ⚠️ **Les non-lus sont par personne, pas par fil.** Deux formateurs
     * partagent la boîte, pas leur lecture : que l'un ait lu un message ne dit
     * rien de l'autre.
     *
     * @return list<array<string, mixed>>
     */
    public function inbox(Utilisateur $viewer): array
    {
        if (!$this->isStorageReady()) {
            return [];
        }

        try {
            $rows = $this->db->fetchAllAssociative(
                'SELECT t.id, t.formationId, t.learnerId, t.lastMessageAt,
                        f.titre AS formationTitle, u.firstName, u.lastName,
                        (SELECT m.body FROM FORMATION_THREAD_MESSAGE m WHERE m.threadId = t.id ORDER BY m.createdAt DESC, m.id DESC LIMIT 1) AS lastBody
                 FROM FORMATION_THREAD t
                 LEFT JOIN FORMATION f ON f.id = t.formationId
                 LEFT JOIN UTILISATEUR u ON u.id = t.learnerId
                 WHERE t.lastMessageAt IS NOT NULL
                 ORDER BY t.lastMessageAt DESC',
            );
        } catch (\Throwable) {
            return [];
        }

        foreach ($rows as $i => $row) {
            $rows[$i]['unread'] = $this->unread((int) $row['id'], (int) $viewer->getId());
        }

        return $rows;
    }

    /** Le total des non-lus de l'équipe, pour ce formateur — pour un badge. */
    public function inboxUnread(Utilisateur $viewer): int
    {
        return array_sum(array_column($this->inbox($viewer), 'unread'));
    }

    /**
     * 🔴 **Efface les fils d'un apprenant anonymisé (appelé par `AccountAnonymiser`).**
     * Un fil privé EST une donnée personnelle de son apprenant : il part en
     * entier. Les réponses que cette personne aurait écrites comme FORMATRICE
     * dans le fil d'un autre restent — elles appartiennent à la conversation de
     * cet autre — et son nom y devient « Anonyme #id » par l'anonymisation
     * elle-même.
     * ⚠️ Pas de clé étrangère vers `UTILISATEUR` : un compte n'est jamais
     * supprimé, seulement anonymisé, donc une cascade ne se déclencherait jamais.
     */
    public function forgetLearner(int $userId): void
    {
        if (!$this->isStorageReady()) {
            return;
        }

        try {
            $this->db->executeStatement('DELETE FROM FORMATION_THREAD WHERE learnerId = ?', [$userId]);
            $this->db->executeStatement('DELETE FROM FORMATION_THREAD_READ WHERE userId = ?', [$userId]);
        } catch (\Throwable $e) {
            $this->logger->error('Les fils de formation d\'un compte anonymisé n\'ont pas pu être effacés.', ['user' => $userId, 'error' => $e->getMessage()]);
        }
    }

    /** @param array<string, mixed> $thread */
    private function sendCopies(array $thread, Formation $formation, Utilisateur $author, string $body): void
    {
        $learnerId = (int) $thread['learnerId'];
        $authorName = trim(($author->getFirstName() ?? '') . ' ' . ($author->getLastName() ?? ''));
        $fromLearner = (int) $author->getId() === $learnerId;

        $recipients = [];
        if ($fromLearner) {
            // L'apprenant écrit : chaque formateur reçoit sa copie.
            foreach ($this->trainers() as $trainer) {
                if ((int) $trainer->getId() !== (int) $author->getId()) {
                    $recipients[] = [$trainer, $this->link('app_trainer_thread', ['thread' => (int) $thread['id']])];
                }
            }
        } else {
            // L'équipe répond : seul l'apprenant du fil reçoit la copie.
            $learner = $this->em->find(Utilisateur::class, $learnerId);
            if ($learner instanceof Utilisateur && $learner->getStatut() === 'actif') {
                $recipients[] = [$learner, $this->link('app_formation_thread', ['id' => $formation->getId()])];
            }
        }

        foreach ($recipients as [$recipient, $url]) {
            // `transactional: false` : la copie respecte l'opt-out MESSAGE. Le
            // message, lui, est déjà dans FabOS — couper la copie ne perd rien.
            $this->mailer->queueToUser($recipient, 'formation_message', [
                'formation' => $formation->getTitre(),
                'author' => $authorName,
                'body' => $body,
                'thread_url' => $url,
            ], NotificationCategory::MESSAGE, false);
        }
    }

    /** @return list<Utilisateur> */
    private function trainers(): array
    {
        $out = [];
        foreach ($this->em->getRepository(Utilisateur::class)->findBy(['statut' => 'actif']) as $user) {
            if ($this->isTrainer($user)) {
                $out[] = $user;
            }
        }

        return $out;
    }

    /**
     * ⚠️ `null` quand l'URL publique n'est pas réglée : le mail dit alors où
     * répondre sans lien. Un lien vers `localhost` dans un e-mail est pire
     * qu'aucun lien.
     *
     * @param array<string, mixed> $params
     */
    private function link(string $route, array $params): ?string
    {
        $base = $this->settings->getPublicBaseUrl();
        if ($base === '') {
            return null;
        }

        try {
            return $base . $this->urls->generate($route, $params, UrlGeneratorInterface::ABSOLUTE_PATH);
        } catch (\Throwable) {
            return null;
        }
    }

    private function isStorageReady(): bool
    {
        if ($this->storageReady !== null) {
            return $this->storageReady;
        }

        try {
            return $this->storageReady = $this->db->createSchemaManager()->tablesExist(['FORMATION_THREAD', 'FORMATION_THREAD_MESSAGE', 'FORMATION_THREAD_READ']);
        } catch (\Throwable) {
            return $this->storageReady = false;
        }
    }
}
