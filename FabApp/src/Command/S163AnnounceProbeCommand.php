<?php

namespace App\Command;

use App\Entity\Event;
use App\Entity\Utilisateur;
use App\Event\EventAnnouncer;
use App\Mail\NotificationCategory;
use App\Mail\NotificationPreferences;
use Doctrine\DBAL\Connection;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S163 — la sonde de l'annonce d'un événement.
 *
 * 🔴 **Elle N'APPELLE PAS `announce()`, et c'est la décision qui structure tout
 * le fichier.** Un appel écrirait pour de vrai à tous les membres actifs du
 * labo ; une sonde qui déclenche l'effet qu'elle mesure n'est pas une sonde,
 * c'est un envoi. Ce qui est donc mesuré, ce sont les trois pièces dont
 * `announce()` est faite, chacune isolément :
 *
 *   1. `recipients()` — les trois filtres, chacun basculé PUIS remis ;
 *   2. `claim()` — la course, sur un événement JETABLE, sans qu'un mail parte ;
 *   3. `canAnnounce()` — les quatre refus, en mémoire, sans base du tout.
 *
 * 🅿️ **Ce qui n'est donc PAS mesuré, dit franchement** : la boucle d'envoi
 * elle-même. Elle est de trois lignes et n'a pas de branche ; ce qui pouvait
 * mal tourner — la course, les filtres, les gardes — est ici.
 *
 * ⚠️ **L'événement jetable est créé ARCHIVÉ et ANNULÉ**, donc invisible partout,
 * et supprimé à la fin. La sonde vérifie le compte de la table avant et après.
 */
#[AsCommand(name: 'app:s163:announce-probe', description: 'S163 : prouve que « annoncer » ne part qu\'une fois, qu\'un membre désabonné n\'est pas compté mais garde ses mails d\'inscription, et que les quatre refus tiennent. N\'envoie rien.')]
final class S163AnnounceProbeCommand extends Command
{
    public function __construct(
        private readonly EventAnnouncer $announcer,
        private readonly NotificationPreferences $preferences,
        private readonly Connection $db,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $io->section('1. Les quatre refus — en mémoire, sans base');
        $future = new \DateTimeImmutable('+10 days');
        $now = new \DateTimeImmutable();

        $ok = (new Event())->setTitre('Sonde')->setDateDebut($future);
        $this->check($io, $failures, 'un événement futur, non annoncé, est annonçable', $this->announcer->canAnnounce($ok, $now));

        $past = (new Event())->setTitre('Sonde')->setDateDebut(new \DateTimeImmutable('-1 day'));
        $this->check($io, $failures, '🔴 un événement PASSÉ ne l\'est pas', !$this->announcer->canAnnounce($past, $now));

        $cancelled = (new Event())->setTitre('Sonde')->setDateDebut($future)->callOff('sonde', $now);
        $this->check($io, $failures, '🔴 un événement ANNULÉ non plus', !$this->announcer->canAnnounce($cancelled, $now));

        $archived = (new Event())->setTitre('Sonde')->setDateDebut($future)->archive();
        $this->check($io, $failures, 'un événement ARCHIVÉ non plus', !$this->announcer->canAnnounce($archived, $now));

        $already = (new Event())->setTitre('Sonde')->setDateDebut($future)->setAnnouncedAt($now);
        $this->check($io, $failures, '🔴 un événement DÉJÀ ANNONCÉ non plus', !$this->announcer->canAnnounce($already, $now));

        $io->section('2. 🔴 Deux clics n\'annoncent qu\'une fois');
        $eventsBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM EVENEMENT');
        $id = $this->createThrowawayEvent();

        try {
            $first = $this->announcer->claim($id, 42, $now);
            $second = $this->announcer->claim($id, 99, $now);
            $this->check($io, $failures, 'le premier clic pose la marque', $first);
            $this->check($io, $failures, '🔴 le second ne touche AUCUNE ligne', !$second);

            $row = $this->db->fetchAssociative('SELECT announcedAt, announcedCount FROM EVENEMENT WHERE id = ?', [$id]);
            $this->check($io, $failures, 'la date est écrite', ($row['announcedAt'] ?? null) !== null);
            // ⚠️ 42 et pas 99 : le second appel n'a rien écrasé non plus. Une
            // course qui « ne renvoie pas » mais réécrit le compte laisserait
            // l'écran mentir sur ce qui s'est passé.
            $this->check($io, $failures, 'le compte du PREMIER clic est conservé', (int) ($row['announcedCount'] ?? 0) === 42);
        } finally {
            $this->db->executeStatement('DELETE FROM EVENEMENT WHERE id = ?', [$id]);
        }

        $this->check($io, $failures, 'l\'événement jetable est supprimé', (int) $this->db->fetchOne('SELECT COUNT(*) FROM EVENEMENT') === $eventsBefore);

        $io->section('3. Un membre désabonné n\'est pas compté — et garde ses inscriptions');
        $victim = $this->pickRecipient();
        if (!$victim instanceof Utilisateur) {
            $io->warning('Aucun membre actif abonné aux annonces : la section 3 n\'a pas pu être MESURÉE.');
            $failures[] = 'section 3 non mesurable — aucun membre actif abonné';
        } else {
            $userId = (int) $victim->getId();
            $io->writeln('   cobaye : #' . $userId);
            $before = \count($this->announcer->recipients());

            $this->db->executeStatement(
                'INSERT INTO USER_NOTIFICATION_OPTOUT (userId, category, optedOutAt) VALUES (?, ?, NOW())',
                [$userId, NotificationCategory::NEWS],
            );

            try {
                $after = $this->announcer->recipients();
                $this->check($io, $failures, 'il sort de la liste des destinataires', \count($after) === $before - 1);
                $this->check($io, $failures, 'et c\'est bien LUI qui en sort', !$this->contains($after, $userId));

                // 🔴 L'invariant qui compte : couper les annonces ne coupe PAS
                // les confirmations. `EVENT` n'est pas désabonnable — s'il
                // l'était, un inscrit perdrait la confirmation de sa propre
                // inscription en refusant la publicité.
                $this->check($io, $failures, '🔴 ses mails d\'INSCRIPTION passent toujours', $this->preferences->accepts($userId, NotificationCategory::EVENT));
                $this->check($io, $failures, 'et ses annonces, non', !$this->preferences->accepts($userId, NotificationCategory::NEWS));
            } finally {
                $this->db->executeStatement(
                    'DELETE FROM USER_NOTIFICATION_OPTOUT WHERE userId = ? AND category = ?',
                    [$userId, NotificationCategory::NEWS],
                );
            }

            $this->check($io, $failures, 'la préférence est REMISE comme avant', $this->preferences->accepts($userId, NotificationCategory::NEWS));
            $this->check($io, $failures, 'et il est de retour dans la liste', \count($this->announcer->recipients()) === $before);
        }

        $io->section('4. Le compte annoncé est celui des destinataires réels');
        $recipients = $this->announcer->recipients();
        $io->writeln('   ' . \count($recipients) . ' membre(s) recevraient une annonce aujourd\'hui');
        // ⚠️ Aucun compte inactif dans la liste : `AccountAnonymiser` passe les
        // comptes effacés en « inactif », donc les exclure ne demande pas de
        // clause spéciale — ce que cette assertion vérifie plutôt que de le
        // supposer.
        $this->check($io, $failures, 'aucun compte inactif dans la liste', !$this->hasInactive($recipients));

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S163 verte. Aucun courrier mis en file, aucune donnée laissée derrière.');

        return Command::SUCCESS;
    }

    /**
     * ⚠️ **Archivé ET annulé dès l'insertion** : entre sa création et sa
     * suppression, cet événement ne doit apparaître sur aucune page — ni
     * l'agenda public, ni la liste d'administration.
     */
    private function createThrowawayEvent(): int
    {
        $this->db->executeStatement(
            "INSERT INTO EVENEMENT (titre, dateDebut, createdAt, archivedAt, cancelledAt, guestsAllowed, locationMode)
             VALUES ('[SONDE S163]', NOW(), NOW(), NOW(), NOW(), 0, 'onsite')",
        );

        return (int) $this->db->lastInsertId();
    }

    private function pickRecipient(): ?Utilisateur
    {
        $rows = $this->announcer->recipients();

        return $rows[0] ?? null;
    }

    /** @param Utilisateur[] $users */
    private function contains(array $users, int $id): bool
    {
        foreach ($users as $user) {
            if ((int) $user->getId() === $id) {
                return true;
            }
        }

        return false;
    }

    /** @param Utilisateur[] $users */
    private function hasInactive(array $users): bool
    {
        foreach ($users as $user) {
            if ($user->getStatut() !== 'actif') {
                return true;
            }
        }

        return false;
    }

    /** @param list<string> $failures */
    private function check(SymfonyStyle $io, array &$failures, string $what, bool $ok): void
    {
        $io->writeln(($ok ? '   <info>✓</info> ' : '   <error>✗</error> ') . $what);
        if (!$ok) {
            $failures[] = $what;
        }
    }
}
