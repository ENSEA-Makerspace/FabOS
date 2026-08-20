<?php

declare(strict_types=1);

namespace App\Command;

use App\Entity\Event;
use App\Entity\EventRegistration;
use App\Entity\Progression;
use App\Entity\Utilisateur;
use App\Event\EventRegistrationService;
use App\Repository\EventRepository;
use App\Repository\FormationRepository;
use App\Repository\ProgressionRepository;
use App\Repository\UtilisateurRepository;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/** THROWAWAY — S146e against the real database, in a ROLLED-BACK transaction. */
#[AsCommand(name: 'app:probe:s146e')]
final class S146eProbeCommand extends Command
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly EventRegistrationService $registrations,
        private readonly EventRepository $events,
        private readonly FormationRepository $formations,
        private readonly ProgressionRepository $progressions,
        private readonly UtilisateurRepository $users,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $connection = $this->em->getConnection();
        @$connection->setNestTransactionsWithSavepoints(true);

        $before = (int) $connection->fetchOne('SELECT COUNT(*) FROM PROGRESSION');
        $regsBefore = (int) $connection->fetchOne('SELECT COUNT(*) FROM EVENT_REGISTRATION');
        $io->text(sprintf('before: %d progressions, %d registrations', $before, $regsBefore));

        $connection->beginTransaction();
        $pass = 0;
        $fail = 0;
        $check = function (string $what, bool $ok) use ($io, &$pass, &$fail): void {
            $ok ? $pass++ : $fail++;
            $io->text(($ok ? '  OK   ' : '  FAIL ') . $what);
        };

        try {
            $formation = $this->formations->findOneBy(['titre' => 'Formation découpe laser CO2']);
            $member = $this->users->findOneBy([]);

            // A session of that training, far enough ahead to be open.
            $session = (new Event())
                ->setTitre('PROBE Séance')
                ->setDateDebut(new \DateTimeImmutable('+30 days 18:00'))
                ->setDateFin(new \DateTimeImmutable('+30 days 20:00'))
                ->setFormation($formation)
                ->setGuestsAllowed(true)
                ->setCapacite(1);
            $this->em->persist($session);
            $this->em->flush();

            // The member must start with no progression on it, or the probe proves nothing.
            $pre = $this->progressions->findOneBy(['utilisateur' => $member, 'formation' => $formation]);
            if ($pre !== null) {
                $this->em->remove($pre);
                $this->em->flush();
            }

            $result = $this->registrations->register($session, $member);
            $check('registration accepted: ' . ($result->ok ? 'yes' : 'no — ' . $result->message), $result->ok);

            $this->em->clear();
            $formation = $this->formations->find($formation->getId());
            $member = $this->users->find($member->getId());
            $progression = $this->progressions->findOneBy(['utilisateur' => $member, 'formation' => $formation]);

            $check('taking the place enrolled them in the training', $progression instanceof Progression);
            if ($progression !== null) {
                // 🔴 The whole point.
                $check('enrolled but NOT completed', $progression->isCompleted() === false);
                $check('enrolled with score 0 — attending certifies nothing', $progression->getScore() === 0);
                $check('no finish date claimed', $progression->getDateEnd() === null);
            }

            // 🔴 A second registration must not touch existing progress.
            $progression->setScore(80);
            $this->em->flush();
            $this->em->clear();

            $session = $this->events->findOneBy(['titre' => 'PROBE Séance']);
            $member = $this->users->find($member->getId());
            $again = $this->registrations->register($session, $member);
            $check('registering twice is refused (' . $again->code . ')', !$again->ok);

            $this->em->clear();
            $kept = $this->progressions->findOneBy([
                'utilisateur' => $this->users->find($member->getId()),
                'formation' => $this->formations->find($formation->getId()),
            ]);
            $check('an existing score of 80 survives untouched', $kept->getScore() === 80);
            $check('and is still not completed', $kept->isCompleted() === false);

            // ⚠️ The waiting list is not the room: capacity is 1 and it is taken.
            $other = $this->users->findOneBy(['email' => null]) ?? null;
            $guestResult = $this->registrations->register(
                $this->events->findOneBy(['titre' => 'PROBE Séance']),
                null,
                'probe-guest@example.invalid',
                'Invité sonde',
            );
            $check('a guest is waitlisted, not enrolled (' . ($guestResult->isWaitlisted() ? 'waitlisted' : $guestResult->code) . ')',
                $guestResult->ok && $guestResult->isWaitlisted());

            $this->em->clear();
            $guestProgressions = (int) $connection->fetchOne(
                'SELECT COUNT(*) FROM PROGRESSION p JOIN UTILISATEUR u ON u.id = p.userId WHERE u.email = ?',
                ['probe-guest@example.invalid'],
            );
            $check('no progression exists for the guest', $guestProgressions === 0);

            // An event with NO training enrols nobody.
            $plain = (new Event())
                ->setTitre('PROBE Atelier sans formation')
                ->setDateDebut(new \DateTimeImmutable('+31 days 10:00'))
                ->setGuestsAllowed(true);
            $this->em->persist($plain);
            $this->em->flush();
            $countBefore = (int) $connection->fetchOne('SELECT COUNT(*) FROM PROGRESSION');
            $this->registrations->register($plain, $this->users->find($member->getId()));
            $countAfter = (int) $connection->fetchOne('SELECT COUNT(*) FROM PROGRESSION');
            $check('an event with no training enrols nobody', $countAfter === $countBefore);
        } catch (\Throwable $e) {
            $io->error($e->getMessage() . ' @ ' . $e->getFile() . ':' . $e->getLine());
            $fail++;
        } finally {
            while ($connection->isTransactionActive()) {
                $connection->rollBack();
            }
        }

        $this->em->clear();
        $after = (int) $connection->fetchOne('SELECT COUNT(*) FROM PROGRESSION');
        $regsAfter = (int) $connection->fetchOne('SELECT COUNT(*) FROM EVENT_REGISTRATION');
        $io->text(sprintf('after rollback: %d progressions, %d registrations', $after, $regsAfter));
        $clean = $after === $before && $regsAfter === $regsBefore;
        $io->text($clean ? '  OK   nothing survived the rollback' : '  FAIL rows survived — INVESTIGATE');
        $io->success(sprintf('%d passed, %d failed, rollback %s', $pass, $fail, $clean ? 'clean' : 'DIRTY'));

        return $fail === 0 && $clean ? Command::SUCCESS : Command::FAILURE;
    }
}
