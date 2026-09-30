<?php

namespace App\Command;

use App\Entity\AccessPoint;
use App\Entity\Badge;
use App\Entity\Place;
use App\Entity\Utilisateur;
use App\Entity\Venue;
use App\Repository\UtilisateurRepository;
use App\Reservation\LabClock;
use App\Reservation\ReservableType;
use App\Reservation\ReservationService;
use App\Rfid\DoorAccessDecision;
use App\Schedule\ScheduleResolver;
use App\Service\PlaceBadges;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S204 — une pièce qui exige un badge : réserver l'exige, et la porte s'ouvre
 * au badge SEUL pendant les heures d'ouverture.
 *
 * Sur une pièce et une porte de SONDE (transaction annulée), dans le lieu
 * « default » et à ses heures réelles :
 *   1. sans badge : porte refusée (missing_badge), réservation refusée
 *      (TRAINING_REQUIRED) ;
 *   2. avec le badge, heures d'ouverture : porte OUVERTE sans réservation ;
 *   3. avec le badge, hors des heures : le badge ne suffit plus (no_booking_now) ;
 *   4. la même pièce SANS badge exigé : comportement d'avant (no_booking_now),
 *      et la réservation n'est pas refusée pour la formation.
 */
#[AsCommand(name: 'app:s204:place-badge-probe', description: 'S204 : une pièce exige un badge — réservation refusée sans, porte ouverte au badge seul pendant les heures d’ouverture. Transaction annulée.')]
final class S204PlaceBadgeProbeCommand extends Command
{
    public function __construct(
        private readonly Connection $db,
        private readonly EntityManagerInterface $em,
        private readonly UtilisateurRepository $users,
        private readonly PlaceBadges $placeBadges,
        private readonly DoorAccessDecision $doors,
        private readonly ReservationService $reservations,
        private readonly ScheduleResolver $schedule,
        private readonly LabClock $clock,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];
        if (!$this->placeBadges->isReady()) {
            $io->error('La migration S204 (PLACE_BADGE) n’est pas passée.');

            return Command::FAILURE;
        }
        $venue = $this->em->getRepository(Venue::class)->findOneBy(['slug' => 'default']);
        $member = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            if (!\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $member = $candidate;
                break;
            }
        }
        $badge = $this->em->getRepository(Badge::class)->findOneBy(['nom' => 'Accès MetalFab'])
            ?? $this->em->getRepository(Badge::class)->findOneBy([]);
        // Un instant où le lieu est OUVERT, et un où il est fermé, mesurés sur ses vrais horaires.
        [$open, $closed] = $this->instants($venue);
        if (!$venue instanceof Venue || !$member instanceof Utilisateur || !$badge instanceof Badge || $open === null || $closed === null) {
            $io->error('Il faut le lieu « default » avec des horaires, un membre et un badge.');

            return Command::FAILURE;
        }
        $io->writeln(sprintf('   membre #%d, badge « %s », ouvert %s, fermé %s', $member->getId(), $badge->getNom(), $open->format('D H:i'), $closed->format('D H:i')));
        $counts = fn (): array => array_map(fn (string $t): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM ' . $t), ['PLACE', 'PLACE_BADGE', 'RESERVATION', 'UTILISATEUR_BADGE']);
        $before = $counts();

        $this->db->beginTransaction();
        try {
            $this->db->executeStatement('DELETE FROM UTILISATEUR_BADGE WHERE utilisateurId = ? AND badgeId = ?', [$member->getId(), $badge->getId()]);
            $place = (new Place())->setNom('Pièce de sonde S204')->setVenue($venue);
            $this->em->persist($place);
            $door = (new AccessPoint())->setNom('Porte de sonde S204')->setVenue($venue)->setPlace($place);
            $this->em->persist($door);
            $this->em->flush();
            $this->placeBadges->set($place, [$badge]);
            $slotStart = $this->clock->storedFormOf($open->modify('+1 day'));

            $io->section('1. Sans le badge');
            $this->check($io, $failures, 'porte refusée : missing_badge', $this->doors->decide($door, $member, $open)['status'] === 'missing_badge');
            $refused = $this->reservations->book(ReservableType::Place, (int) $place->getId(), $member, $slotStart, $slotStart->modify('+1 hour'));
            $this->check($io, $failures, 'réservation refusée : ' . ($refused->code ?? 'acceptée !') . ' — « ' . $refused->message . ' »', !$refused->ok && $refused->code === 'TRAINING_REQUIRED' && str_contains((string) $refused->message, $badge->getNom()));

            $io->section('2. Avec le badge, pendant les heures');
            $this->db->executeStatement('INSERT INTO UTILISATEUR_BADGE (utilisateurId, badgeId, dateObtention) VALUES (?, ?, NOW())', [$member->getId(), $badge->getId()]);
            $verdict = $this->doors->decide($door, $member, $open);
            $this->check($io, $failures, '🔴 porte OUVERTE au badge seul, sans réservation (' . $verdict['status'] . ')', $verdict['allowed'] && $verdict['status'] === 'badge_open_hours');
            $booked = $this->reservations->book(ReservableType::Place, (int) $place->getId(), $member, $slotStart, $slotStart->modify('+1 hour'));
            $this->check($io, $failures, 'la réservation n’est plus refusée pour la formation (' . ($booked->ok ? 'acceptée' : $booked->code) . ')', $booked->ok || $booked->code !== 'TRAINING_REQUIRED');

            $io->section('3. Avec le badge, hors des heures');
            $late = $this->doors->decide($door, $member, $closed);
            $this->check($io, $failures, 'le badge seul ne suffit plus (' . $late['status'] . ')', !$late['allowed'] && $late['status'] === 'no_booking_now');

            $io->section('4. La même pièce sans badge exigé');
            $this->placeBadges->set($place, []);
            $this->check($io, $failures, 'comportement d’avant : la porte attend une réservation (no_booking_now)', $this->doors->decide($door, $member, $open)['status'] === 'no_booking_now');
        } finally {
            $this->db->rollBack();
            $this->em->clear();
        }

        $this->check($io, $failures, 'rien n’est resté (pièces, liens, réservations, badges détenus)', $counts() === $before);
        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S204 verte. Transaction annulée.');

        return Command::SUCCESS;
    }

    /** @return array{0: ?\DateTimeImmutable, 1: ?\DateTimeImmutable} un instant ouvert, un instant fermé (heure du labo) */
    private function instants(?Venue $venue): array
    {
        if (!$venue instanceof Venue) {
            return [null, null];
        }
        $open = $closed = null;
        $day = $this->clock->now()->setTime(0, 0);
        for ($i = 0; $i < 14 && ($open === null || $closed === null); ++$i, $day = $day->modify('+1 day')) {
            $window = $this->schedule->openMinutesFor($venue->getId(), $day);
            if ($window !== null && $open === null) {
                $open = $day->modify('+' . ($window['start'] + 30) . ' minutes');
            }
            if ($closed === null && !$this->schedule->isOpenAt($venue->getId(), $day->modify('+3 hours'))) {
                $closed = $day->modify('+3 hours');
            }
        }

        return [$open, $closed];
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
