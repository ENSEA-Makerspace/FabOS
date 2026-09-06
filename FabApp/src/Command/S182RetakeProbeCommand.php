<?php

namespace App\Command;

use App\Entity\Formation;
use App\Entity\Progression;
use App\Entity\Utilisateur;
use App\Service\GuidedTrainingService;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S182 — la sonde qui prouve qu'on ne perd pas ce qu'on a acquis.
 *
 * 🔴 **La mesure de sortie, écrite dans la feuille de route** : « une reprise ne
 * réinitialise pas ce qui était acquis ». Lire le code montre un `max()` sur le
 * score d'un quiz ; seule une recompilation lancée deux fois, avant et après une
 * régression, prouve que la progression du PARCOURS ne recule pas non plus.
 *
 * ⚠️ **Tout se passe dans une transaction ANNULÉE.** La sonde force une
 * progression terminée, relance la synchronisation, et la base ressort
 * inchangée.
 */
#[AsCommand(name: 'app:s182:retake-probe', description: 'S182 : prouve qu\'une recompilation ne retire ni la complétion, ni sa date, ni le score.')]
final class S182RetakeProbeCommand extends Command
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly GuidedTrainingService $guided,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $user = $this->em->getRepository(Utilisateur::class)->findOneBy([]);
        if ($user === null) {
            $io->error('Il faut au moins un membre.');

            return Command::FAILURE;
        }

        /*
         * 🔴 **Le choix de la formation EST la sonde.** Une formation que ce
         * membre a réellement terminée ne prouverait rien : l'ancien code
         * gardait déjà la date quand le parcours restait complet. Ce qu'il
         * faut, c'est une formation dont le parcours n'est PAS complet pour
         * lui — c'est exactement l'état dans lequel un admin met tout le monde
         * en ajoutant un quiz obligatoire. Là, l'ancien code écrivait
         * `setCompleted(false)` et `setDateEnd(null)`.
         */
        $formation = null;
        foreach ($this->em->getRepository(Formation::class)->findBy([]) as $candidate) {
            if (str_starts_with($candidate->getTitre(), '[FABOS') || str_starts_with($candidate->getTitre(), 'Validation physique')) {
                continue;
            }
            $progress = $this->guided->getProgress($candidate, $user);
            if (($progress['completedAll'] ?? false) === false) {
                $formation = $candidate;
                break;
            }
        }
        if ($formation === null) {
            $io->error('Aucune formation au parcours INCOMPLET pour ce membre : la sonde ne mesurerait rien.');

            return Command::FAILURE;
        }
        $io->writeln('<comment>Parcours INCOMPLET pour ce membre — c\'est le cas où l\'ancien code effaçait la date.</comment>');

        $before = $this->snapshot();
        $io->writeln(sprintf('Formation « %s », membre #%d.', $formation->getTitre(), (int) $user->getId()));

        $this->em->beginTransaction();

        try {
            $progression = $this->em->getRepository(Progression::class)->findOneBy([
                'utilisateur' => $user,
                'formation' => $formation,
            ]);
            if (!$progression instanceof Progression) {
                $progression = (new Progression())->setUtilisateur($user)->setFormation($formation);
                $this->em->persist($progression);
            }

            /*
             * On force l'état « terminé », avec une date de fin qui est le fait
             * historique que la recompilation ne doit pas pouvoir effacer.
             * ⚠️ **Elle se calcule à partir de `dateDebut`, pas d'une date en
             * dur.** La base porte une contrainte `chk_progression_dates` qui
             * refuse une fin antérieure au début — ma première version écrivait
             * le 1er janvier et s'est fait rejeter. La contrainte a raison, et
             * c'est une bonne nouvelle pour cette session : l'ordre des dates
             * est déjà garanti par le schéma.
             */
            $finishedOn = $progression->getDateDebut()->modify('+1 day');
            $progression->setScore(100)->setCompleted(true)->setDateEnd($finishedOn);
            $this->em->flush();

            $io->section('1. L\'état acquis, avant recompilation');
            $io->writeln(sprintf('   score=%d terminé=%s fin=%s', $progression->getScore(), $progression->isCompleted() ? 'oui' : 'non', $progression->getDateEnd()?->format('Y-m-d') ?? 'null'));

            $io->section('2. On recompile — c\'est ce que fait chaque quiz rendu, et chaque quiz AJOUTÉ par un admin');
            $this->guided->synchronizeParentProgress($formation, $user, true);
            $this->em->refresh($progression);

            $this->check($io, $failures, 'la complétion tient', $progression->isCompleted());
            $this->check($io, $failures, 'la DATE de fin tient — c\'est un fait sur le passé', $progression->getDateEnd()?->format('Y-m-d H:i:s') === $finishedOn->format('Y-m-d H:i:s'));
            $this->check($io, $failures, 'le score ne recule pas', $progression->getScore() >= 100);

            $io->writeln(sprintf('   après : score=%d terminé=%s fin=%s', $progression->getScore(), $progression->isCompleted() ? 'oui' : 'non', $progression->getDateEnd()?->format('Y-m-d') ?? 'null'));

            /*
             * 🔴 **Le cas qui cassait vraiment, et qui n'est pas une reprise.**
             * `$requiredQuizTotal` est le nombre de quiz obligatoires
             * AUJOURD'HUI : un admin qui en ajoute un faisait retomber
             * `pathCompleted` à `false` pour TOUS ceux qui avaient fini — et
             * `dateEnd` était remis à `null` avec. La recompilation ci-dessus
             * est exactement ce chemin-là.
             */
            $io->section('3. Et le badge, lui, ne se retire jamais');
            $io->writeln('   <comment>`ProgressionBadgeSubscriber` accorde et n\'a aucun chemin de révocation — vérifié par lecture. C\'est ce qui rendait l\'ancien comportement incohérent : on POSSÉDAIT le badge d\'une formation que la progression déclarait non terminée.</comment>');
        } finally {
            $this->em->rollback();
            $this->em->clear();
        }

        $io->section('4. La base est rendue telle qu\'elle était');
        $after = $this->snapshot();
        $io->writeln('   ' . json_encode($after));
        $this->check($io, $failures, 'aucune ligne créée ni modifiée', $before === $after);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S182 verte.');

        return Command::SUCCESS;
    }

    /** @return array<string, int> */
    private function snapshot(): array
    {
        return [
            'progressions' => (int) $this->em->createQuery('SELECT COUNT(p.id) FROM App\Entity\Progression p')->getSingleScalarResult(),
            'terminees' => (int) $this->em->createQuery('SELECT COUNT(p.id) FROM App\Entity\Progression p WHERE p.completed = true')->getSingleScalarResult(),
            'badges' => (int) $this->em->createQuery('SELECT COUNT(ub.dateObtention) FROM App\Entity\UtilisateurBadge ub')->getSingleScalarResult(),
        ];
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
