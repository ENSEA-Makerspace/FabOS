<?php

namespace App\Reservation;

use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Repository\MachineRepository;
use App\Repository\PlaceRepository;
use App\Repository\ReservationRepository;
use App\Service\MachineQualificationService;
use App\Service\OpeningHoursProvider;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\SecurityBundle\Security;

/**
 * The single chokepoint every booking goes through, whatever kind of resource it
 * is and whichever entry point asked for it. Before this existed the JSON API
 * and the place form each re-implemented the same four checks, and drifted: the
 * place form had no access gate at all.
 *
 * Callers own payload parsing (JSON shape, form fields, date formats) and
 * presentation; this owns the booking rules. Anything that decides whether a
 * booking is *allowed* — the future check, opening hours, overlap, and the
 * per-resource access gate — belongs here, so the later permission work
 * (cert-gating, quotas, access passes) has exactly one place to hook into.
 */
final class ReservationService
{
    private const MOTIF_MAX_LENGTH = 500;

    public function __construct(
        private readonly ReservationRepository $reservations,
        private readonly ReservableResolver $reservables,
        private readonly MachineRepository $machines,
        private readonly PlaceRepository $places,
        private readonly OpeningHoursProvider $openingHours,
        private readonly MachineQualificationService $machineAccess,
        private readonly EntityManagerInterface $em,
        private readonly Security $security,
    ) {
    }

    /**
     * Attempt a booking. Dates must already be parsed; $motif already trimmed
     * (null or '' both mean "no reason given").
     */
    public function book(
        ReservableType $type,
        int $id,
        Utilisateur $user,
        \DateTimeImmutable $start,
        \DateTimeImmutable $end,
        ?string $motif = null,
    ): BookingResult {
        $name = $this->reservables->nameFor($type, $id);
        if ($name === null) {
            return BookingResult::refused(
                $type === ReservableType::Machine ? 'MACHINE_NOT_FOUND' : 'RESOURCE_NOT_FOUND',
                $this->notFoundMessage($type),
                404,
            );
        }

        $refusal = $this->checkAccess($type, $id, $user);
        if ($refusal !== null) {
            return $refusal;
        }

        $now = new \DateTimeImmutable('now', new \DateTimeZone('Europe/Paris'));
        if ($start <= $now) {
            return BookingResult::refused('DATE_DEBUT_PAST', 'La date de début doit être dans le futur.', 400);
        }

        if ($end <= $start) {
            return BookingResult::refused('DATE_FIN_BEFORE_START', 'L’heure de fin doit être après l’heure de début.', 400);
        }

        $openingHoursError = $this->openingHours->validateReservationPeriod($start, $end);
        if ($openingHoursError !== null) {
            return BookingResult::refused('FABLAB_CLOSED', $openingHoursError, 400);
        }

        $motif = $motif === null ? null : (trim($motif) ?: null);
        if ($motif !== null && mb_strlen($motif) > self::MOTIF_MAX_LENGTH) {
            return BookingResult::refused('MOTIF_TOO_LONG', 'Le motif ne doit pas dépasser 500 caractères.', 400);
        }

        if ($this->reservations->hasOverlap($type, $id, $start, $end)) {
            return BookingResult::refused('RESERVATION_OVERLAP', $this->overlapMessage($type), 409);
        }

        $reservation = (new Reservation())
            ->setUtilisateur($user)
            ->setDateDebut($start)
            ->setDateFin($end)
            ->setMotif($motif)
            ->setStatut('confirmed');

        // Populate the legacy machineId/placeId first — those setters dual-write
        // the polymorphic triple — then set it authoritatively, so a kind with no
        // legacy column (a person) lands the same way. Both calls drop out when
        // the contract migration removes the columns.
        $this->mirrorLegacyAssociation($reservation, $type, $id);
        $reservation->setReservable($type, $id, $name);

        $this->em->persist($reservation);
        $this->em->flush();

        return BookingResult::created($reservation);
    }

    /**
     * Per-resource-kind gate. Machines require their training badges; spaces are
     * open to any member today; people will gate on the booking-policy engine
     * once it exists. Admins bypass, as they did before.
     */
    private function checkAccess(ReservableType $type, int $id, Utilisateur $user): ?BookingResult
    {
        if ($type !== ReservableType::Machine || $this->security->isGranted('ROLE_ADMIN')) {
            return null;
        }

        $machine = $this->machines->find($id);
        if ($machine === null) {
            return null;
        }

        $status = $this->machineAccess->getStatus($machine, $user);
        if ($status['authorized']) {
            return null;
        }

        $missingNames = array_map(static fn ($badge): string => $badge->getNom(), $status['missingBadges']);
        $practicalMissing = ($status['trainingBlockReason'] ?? null) === 'physical_training_required';

        $message = $practicalMissing
            ? 'La formation pratique doit être validée avant de réserver cette machine.'
            : ($missingNames === []
                ? 'La formation nécessaire pour cette machine doit être validée avant toute réservation.'
                : 'Formation requise : obtenez ' . implode(', ', $missingNames) . ' avant de réserver cette machine.');

        return BookingResult::refused(
            $practicalMissing ? 'PRACTICAL_TRAINING_REQUIRED' : 'TRAINING_REQUIRED',
            $message,
            403,
            ['missingBadges' => $missingNames],
        );
    }

    /**
     * Keep machineId/placeId in step with the polymorphic pair. Dropped whole
     * once the contract migration removes those columns.
     */
    private function mirrorLegacyAssociation(Reservation $reservation, ReservableType $type, int $id): void
    {
        if ($type === ReservableType::Machine) {
            $machine = $this->machines->find($id);
            if ($machine !== null) {
                $reservation->setMachine($machine);
            }
        }

        if ($type === ReservableType::Place) {
            $place = $this->places->find($id);
            if ($place !== null) {
                $reservation->setPlace($place);
            }
        }
    }

    private function notFoundMessage(ReservableType $type): string
    {
        return match ($type) {
            ReservableType::Machine => 'Machine introuvable',
            ReservableType::Place => 'Espace introuvable',
            ReservableType::User => 'Personne introuvable',
        };
    }

    private function overlapMessage(ReservableType $type): string
    {
        return match ($type) {
            ReservableType::Machine => 'Créneau déjà réservé pour cette machine',
            ReservableType::Place => 'Ce créneau est déjà réservé pour cet espace.',
            ReservableType::User => 'Ce créneau est déjà réservé pour cette personne.',
        };
    }
}
