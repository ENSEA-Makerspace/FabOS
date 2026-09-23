<?php

namespace App\Command;

use App\Entity\Formation;
use App\Entity\Progression;
use App\Entity\Utilisateur;
use App\Training\CohortAnnouncer;
use App\Training\FormationThreads;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S183b — la sonde du fil privé apprenant ↔ équipe.
 *
 * 🔴 **La mesure qui compte : un AUTRE apprenant ne lit pas le fil.** C'est
 * l'invariant de la phase, et c'est lui qu'on vérifie d'abord — avec, à côté, le
 * cas qu'on oublie : un administrateur qui n'est PAS formateur ne le lit pas non
 * plus. Administrer n'est pas un droit de lecture.
 *
 * 🔴 **Elle n'envoie AUCUN courrier, et le MESURE.** Quand un apprenant écrit,
 * chaque formateur réel recevrait une copie : la sonde ne poste donc jamais comme
 * apprenant. Elle poste comme un auteur jetable côté « équipe », dont la copie va
 * à un apprenant jetable qui a coupé ses e-mails — et elle compte `EMAIL_LOG`
 * avant et après.
 *
 * ⚠️ **Trois comptes jetables**, sans groupe, supprimés à la fin ; les comptes
 * réels ne sont que LUS (un formateur, un administrateur non formateur), jamais
 * écrits.
 */
#[AsCommand(name: 'app:s183:thread-probe', description: 'S183b : prouve qu\'un autre apprenant et un admin non formateur ne lisent pas un fil privé, qu\'un fil est unique par apprenant, et qu\'aucun courrier ne part. Remet tout en place.')]
final class S183ThreadProbeCommand extends Command
{
    public function __construct(
        private readonly FormationThreads $threads,
        private readonly CohortAnnouncer $cohort,
        private readonly EntityManagerInterface $em,
        private readonly Connection $db,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        if (!$this->threads->isAvailable()) {
            $io->error('Les tables du fil n\'existent pas : la migration Version20260923090000 n\'a pas été lancée.');

            return Command::FAILURE;
        }

        $formation = $this->em->getRepository(Formation::class)->findOneBy([], ['id' => 'ASC']);
        if (!$formation instanceof Formation) {
            $io->error('Aucune formation : rien à mesurer.');

            return Command::FAILURE;
        }

        $mailsBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG');
        $threadsBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM FORMATION_THREAD');
        $usersBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM UTILISATEUR');

        $learner = $this->throwaway('apprenant');
        $other = $this->throwaway('autre');
        $team = $this->throwaway('equipe');
        $this->em->flush();
        $ids = [(int) $learner->getId(), (int) $other->getId(), (int) $team->getId()];

        try {
            $io->section('1. 🔴 UN fil par (formation, apprenant) — la base tranche');
            $first = $this->threads->threadFor($formation, $learner, true);
            $again = $this->threads->threadFor($formation, $learner, true);
            $this->check($io, $failures, 'deux demandes ⇒ le MÊME fil', $first !== null && $again !== null && $first['id'] === $again['id']);
            $otherThread = $this->threads->threadFor($formation, $other, true);
            $this->check($io, $failures, 'un autre apprenant ⇒ un AUTRE fil', $otherThread !== null && $otherThread['id'] !== $first['id']);

            $io->section('2. 🔴 Qui LIT le fil');
            $this->check($io, $failures, 'son apprenant, oui', $this->threads->canRead($first, $learner));
            $this->check($io, $failures, '🔴 un AUTRE apprenant, NON', !$this->threads->canRead($first, $other));

            [$trainer, $adminOnly] = $this->realReaders();
            if ($trainer instanceof Utilisateur) {
                $this->check($io, $failures, 'un formateur (#' . $trainer->getId() . '), oui', $this->threads->canRead($first, $trainer));
            } else {
                $io->warning('Aucun membre du groupe `trainers` : la moitié « formateur » n\'a pas pu être mesurée.');
                $failures[] = 'aucun formateur réel pour la mesure';
            }
            if ($adminOnly instanceof Utilisateur) {
                // Le cas qu'on oublie : administrer n'est pas un droit de lecture.
                $this->check($io, $failures, '🔴 un admin NON formateur (#' . $adminOnly->getId() . '), NON', !$this->threads->canRead($first, $adminOnly));
            } else {
                $io->writeln('   (aucun administrateur non formateur sur cette installation — cas non mesurable ici)');
            }

            $io->section('3. La cohorte est celle de S183 — une seule définition');
            $sample = $this->em->getRepository(Progression::class)->findOneBy([]);
            if ($sample instanceof Progression && $sample->getUtilisateur() instanceof Utilisateur && $sample->getFormation() instanceof Formation) {
                $this->check($io, $failures, 'une progression réelle ⇒ membre de sa cohorte', $this->cohort->isMember($sample->getFormation(), $sample->getUtilisateur()));
            }
            $this->check($io, $failures, 'un compte sans progression ⇒ PAS membre, donc pas d\'onglet', !$this->cohort->isMember($formation, $learner));

            $io->section('4. Écrire, compter les non-lus');
            $posted = $this->threads->post($first, $formation, $team, "Première réponse\nsur deux lignes.");
            $this->check($io, $failures, 'le message est écrit', $posted !== null);
            $this->check($io, $failures, 'le fil a une date de dernier message', $this->threads->find((int) $first['id'])['lastMessageAt'] !== null);
            $this->check($io, $failures, 'l\'apprenant a 1 non-lu', $this->threads->unread((int) $first['id'], (int) $learner->getId()) === 1);
            $this->check($io, $failures, '⚠️ l\'auteur a 0 non-lu — son propre message ne compte pas', $this->threads->unread((int) $first['id'], (int) $team->getId()) === 0);
            $this->threads->markRead((int) $first['id'], (int) $learner->getId());
            $this->check($io, $failures, 'ouvrir le fil remet à 0', $this->threads->unread((int) $first['id'], (int) $learner->getId()) === 0);
            $this->check($io, $failures, 'un message vide est REFUSÉ', $this->threads->post($first, $formation, $team, "   \n ") === null);
            $this->check($io, $failures, 'l\'autre fil n\'a RIEN reçu', $this->threads->messages((int) $otherThread['id']) === []);

            $io->section('5. 🔴 Aucun courrier n\'est parti');
            $this->check($io, $failures, 'EMAIL_LOG est inchangé', (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG') === $mailsBefore);

            $io->section('6. L\'anonymisation efface les fils de l\'apprenant');
            $this->threads->forgetLearner((int) $learner->getId());
            $this->check($io, $failures, 'son fil a disparu', $this->threads->find((int) $first['id']) === null);
            $this->check($io, $failures, 'et ses messages avec (cascade)', (int) $this->db->fetchOne('SELECT COUNT(*) FROM FORMATION_THREAD_MESSAGE WHERE threadId = ?', [$first['id']]) === 0);
            $this->check($io, $failures, 'le fil d\'un AUTRE apprenant n\'a pas bougé', $this->threads->find((int) $otherThread['id']) !== null);
        } finally {
            foreach ($ids as $id) {
                $this->threads->forgetLearner($id);
                $this->db->executeStatement('DELETE FROM FORMATION_THREAD_READ WHERE userId = ?', [$id]);
            }
            $this->db->executeStatement('DELETE FROM FORMATION_THREAD_MESSAGE WHERE authorId IN (?, ?, ?)', $ids);
            $this->db->executeStatement('DELETE FROM UTILISATEUR WHERE id IN (?, ?, ?)', $ids);
        }

        $io->section('7. Tout est remis en place');
        $this->check($io, $failures, 'fils : compte de départ', (int) $this->db->fetchOne('SELECT COUNT(*) FROM FORMATION_THREAD') === $threadsBefore);
        $this->check($io, $failures, 'comptes : compte de départ', (int) $this->db->fetchOne('SELECT COUNT(*) FROM UTILISATEUR') === $usersBefore);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S183b verte. Aucun courrier, rien laissé derrière.');

        return Command::SUCCESS;
    }

    /** ⚠️ E-mails COUPÉS : c'est ce qui garantit qu'aucune copie ne part vers eux. */
    private function throwaway(string $tag): Utilisateur
    {
        $suffix = bin2hex(random_bytes(4));
        $user = (new Utilisateur())
            ->setEmail('sonde-s183-' . $tag . '-' . $suffix . '@invalid')
            ->setUsername('sonde-s183-' . $tag . '-' . $suffix)
            ->setPassword('!')
            ->setFirstName('Sonde ' . $tag)
            ->setStatut('actif')
            ->setNotificationEmail(false);
        $this->em->persist($user);

        return $user;
    }

    /**
     * Un formateur réel et un administrateur réel qui n'est PAS formateur — lus
     * seulement, jamais écrits.
     *
     * @return array{0: ?Utilisateur, 1: ?Utilisateur}
     */
    private function realReaders(): array
    {
        $trainer = null;
        $adminOnly = null;
        foreach ($this->em->getRepository(Utilisateur::class)->findBy(['statut' => 'actif']) as $user) {
            $roles = $user->getRoles();
            if ($trainer === null && in_array('ROLE_TRAINER', $roles, true)) {
                $trainer = $user;
            }
            if ($adminOnly === null && in_array('ROLE_ADMIN', $roles, true) && !in_array('ROLE_TRAINER', $roles, true)) {
                $adminOnly = $user;
            }
        }

        return [$trainer, $adminOnly];
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
