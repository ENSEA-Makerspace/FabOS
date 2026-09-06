<?php

namespace App\Command;

use App\Entity\AccessPoint;
use App\Entity\Place;
use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Reservation\LabClock;
use App\Reservation\ReservableType;
use App\Rfid\DoorAccessDecision;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S178 — la sonde qui prouve qu'annuler une réservation FERME la porte.
 *
 * 🔴 **C'est la mesure de sortie écrite dans la feuille de route** : « annuler
 * une réservation retire l'accès immédiatement, prouvé par une sonde ». Lire le
 * code montrerait qu'aucune révocation n'est nécessaire ; seule une décision
 * prise deux fois, avant et après l'annulation, prouve que la porte se referme.
 *
 * ⚠️ **Tout se passe dans une transaction ANNULÉE.** La sonde crée une porte,
 * une réservation, décide, annule, redécide — et la base ressort inchangée. Une
 * sonde qui laisse des lignes derrière elle est une sonde qui se lance une fois.
 *
 * ⚠️ **Elle écrit des heures MURALES** (convention B) : `dateDebut`/`dateFin`
 * stockent les digits saisis. Écrire un instant UTC ici décalerait la sonde de
 * deux heures en été et lui ferait mesurer autre chose que ce qu'elle annonce.
 */
#[AsCommand(name: 'app:s178:door-probe', description: 'S178 : prouve qu\'une réservation ouvre la porte et que son annulation la referme, immédiatement.')]
final class S178DoorProbeCommand extends Command
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly DoorAccessDecision $doors,
        private readonly LabClock $clock,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $place = $this->em->getRepository(Place::class)->findOneBy(['archivedAt' => null]);
        $user = $this->em->getRepository(Utilisateur::class)->findOneBy([]);
        if ($place === null || $user === null) {
            $io->error('Il faut au moins un espace et un membre en base.');

            return Command::FAILURE;
        }

        $before = $this->countRows();
        $io->writeln(sprintf('Espace « %s », membre #%d. Lignes avant : %s', $place->getNom(), (int) $user->getId(), json_encode($before)));

        $this->em->beginTransaction();

        try {
            $point = (new AccessPoint())
                ->setNom('SONDE — porte S178')
                ->setKind('door')
                ->setVenue($place->getVenue())
                ->setPlace($place);
            $this->em->persist($point);

            $io->section('1. Sans réservation, la porte reste fermée');
            $verdict = $this->doors->decide($point, $user);
            $this->check($io, $failures, 'refusé, et la raison est « pas de réservation »', !$verdict['allowed'] && $verdict['status'] === 'no_booking_now');

            $io->section('2. Une réservation en cours ouvre la porte');
            // ⚠️ `storedFormOf()` : on écrit les DIGITS que Doctrine attend pour
            // une colonne de convention B, pas un instant.
            $now = $this->clock->now();
            $booking = (new Reservation())
                ->setUtilisateur($user)
                ->setDateDebut($this->clock->storedFormOf($now->modify('-30 minutes')))
                ->setDateFin($this->clock->storedFormOf($now->modify('+30 minutes')))
                ->setStatut('confirmed')
                ->setMotif('SONDE S178')
                ->setReservable(ReservableType::Place, (int) $place->getId(), $place->getNom());
            $this->em->persist($booking);
            $this->em->flush();

            $verdict = $this->doors->decide($point, $user);
            $this->check($io, $failures, 'autorisé pendant le créneau', $verdict['allowed'] && $verdict['status'] === 'booking_in_progress');
            $this->check($io, $failures, 'et la décision NOMME la réservation', $verdict['reservationId'] === $booking->getId());

            $io->section('3. Une réservation de QUELQU\'UN D\'AUTRE n\'ouvre rien');
            $other = $this->em->getRepository(Utilisateur::class)->findOneBy([]);
            foreach ($this->em->getRepository(Utilisateur::class)->findBy([], null, 5) as $candidate) {
                if ($candidate->getId() !== $user->getId()) {
                    $other = $candidate;
                    break;
                }
            }
            if ($other !== null && $other->getId() !== $user->getId()) {
                $verdict = $this->doors->decide($point, $other);
                $this->check($io, $failures, 'un autre membre reste dehors', !$verdict['allowed']);
            } else {
                $io->warning('Un seul membre en base : le cas « quelqu\'un d\'autre » n\'est pas mesuré.');
            }

            $io->section('4. 🔴 On annule — et la porte se referme, sans aucune révocation');
            $booking->setStatut(Reservation::STATUS_CANCELLED);
            $this->em->flush();
            $this->em->clear(Reservation::class);

            $verdict = $this->doors->decide($point, $user);
            $this->check($io, $failures, 'refusé immédiatement après l\'annulation', !$verdict['allowed'] && $verdict['status'] === 'no_booking_now');
            $io->writeln('   <comment>Rien n\'a été révoqué : rien n\'avait été accordé. La question est reposée à chaque badge.</comment>');

            $io->section('5. Une porte ARCHIVÉE gagne sur tout le reste');
            $booking->setStatut('confirmed');
            $point->archive();
            $this->em->flush();
            $verdict = $this->doors->decide($point, $user);
            $this->check($io, $failures, 'porte archivée ⇒ refusé même avec réservation valide', !$verdict['allowed'] && $verdict['status'] === 'access_point_archived');

            $io->section('6. Une porte sans espace nommé ne se décide pas par réservation');
            $orphan = (new AccessPoint())->setNom('SONDE — portail')->setKind('gate')->setVenue($place->getVenue());
            $verdict = $this->doors->decide($orphan, $user);
            $this->check($io, $failures, 'portail d\'entrée ⇒ « aucun espace lié », pas une ouverture', !$verdict['allowed'] && $verdict['status'] === 'no_place_bound');

            $io->section('7. Personne inconnue');
            $this->check($io, $failures, 'badge non rattaché ⇒ refusé', !$this->doors->decide($point, null)['allowed']);
        } finally {
            // ⚠️ `finally` : une assertion qui jette laisserait sinon la
            // transaction ouverte ET les lignes de sonde en base — exactement ce
            // que la sonde promet d'éviter.
            $this->em->rollback();
            $this->em->clear();
        }

        $io->section('8. La base est rendue telle qu\'elle était');
        $after = $this->countRows();
        $io->writeln('   Lignes après : ' . json_encode($after));
        $this->check($io, $failures, 'aucune ligne créée ni modifiée', $before === $after);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S178 verte.');

        return Command::SUCCESS;
    }

    /** @return array<string, int> */
    private function countRows(): array
    {
        return [
            'access_point' => (int) $this->em->createQuery('SELECT COUNT(a.id) FROM App\Entity\AccessPoint a')->getSingleScalarResult(),
            'reservation' => (int) $this->em->createQuery('SELECT COUNT(r.id) FROM App\Entity\Reservation r')->getSingleScalarResult(),
            'cancelled' => (int) $this->em->createQuery("SELECT COUNT(r.id) FROM App\Entity\Reservation r WHERE r.statut = 'cancelled'")->getSingleScalarResult(),
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
