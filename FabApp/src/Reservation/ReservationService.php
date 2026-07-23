<?php

namespace App\Reservation;

use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Repository\MachineRepository;
use App\Repository\ReservationRepository;
use App\Repository\UtilisateurRepository;
use App\Reservation\Policy\BookingPolicyService;
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
 * booking is *allowed* — the future check, opening hours, overlap, the
 * per-resource access gate and the tier quotas — belongs here, so the remaining
 * permission work (access passes) has exactly one place to hook into.
 */
final class ReservationService
{
    private const MOTIF_MAX_LENGTH = 500;

    public function __construct(
        private readonly ReservationRepository $reservations,
        private readonly ReservableResolver $reservables,
        private readonly MachineRepository $machines,
        private readonly UtilisateurRepository $people,
        private readonly PersonAvailabilityService $personAvailability,
        private readonly OpeningHoursProvider $openingHours,
        private readonly MachineQualificationService $machineAccess,
        private readonly EntityManagerInterface $em,
        private readonly Security $security,
        private readonly ReservationMailer $mails,
        private readonly BookingPolicyService $policies,
    ) {
    }

    /**
     * Attempt a booking. Dates must already be parsed; $motif already trimmed
     * (null or '' both mean "no reason given").
     *
     * $asRequest is the "out of the blue" path for people: it drops the
     * requirement that the slot be one the person offered, and the booking lands
     * as pending until they accept it. It has no meaning for machines or spaces,
     * which have nobody to do the accepting.
     */
    public function book(
        ReservableType $type,
        int $id,
        Utilisateur $user,
        \DateTimeImmutable $start,
        \DateTimeImmutable $end,
        ?string $motif = null,
        bool $asRequest = false,
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

        // Quotas run after the sanity checks rather than alongside the access
        // gate: "your booking is too long" only makes sense once we know the end
        // is after the start, and telling someone they're over their weekly
        // limit for a slot the lab is closed on would be answering the wrong
        // question. Unconfigured labs return here immediately.
        $quotaRefusal = $this->policies->check($user, $type, $id, $start, $end, $now);
        if ($quotaRefusal !== null) {
            return $quotaRefusal;
        }

        $motif = $motif === null ? null : (trim($motif) ?: null);
        if ($motif !== null && mb_strlen($motif) > self::MOTIF_MAX_LENGTH) {
            return BookingResult::refused('MOTIF_TOO_LONG', 'Le motif ne doit pas dépasser 500 caractères.', 400);
        }

        $pending = false;
        if ($type === ReservableType::User) {
            $person = $this->people->find($id);
            $offered = $person !== null
                && $person->offersDuration((int) round(($end->getTimestamp() - $start->getTimestamp()) / 60))
                && $this->personAvailability->covers($person, $start, $end);

            // Anything outside the offered slots is only reachable through the
            // request form, and stays pending until the person accepts it.
            if (!$offered && !$asRequest) {
                return BookingResult::refused(
                    'SLOT_NOT_OFFERED',
                    'Ce créneau ne fait pas partie des disponibilités proposées. Envoyez une demande pour le proposer à cette personne.',
                    409,
                );
            }

            $pending = !$offered;
        }

        // A pending request holds its slot: two people can't both be waiting on
        // the same hour, and declining frees it again.
        if ($this->reservations->hasOverlap($type, $id, $start, $end)) {
            return BookingResult::refused('RESERVATION_OVERLAP', $this->overlapMessage($type), 409);
        }

        $reservation = (new Reservation())
            ->setUtilisateur($user)
            ->setDateDebut($start)
            ->setDateFin($end)
            ->setMotif($motif)
            ->setStatut($pending ? Reservation::STATUS_PENDING : Reservation::STATUS_CONFIRMED)
            ->setReservable($type, $id, $name);

        $this->em->persist($reservation);
        $this->em->flush();

        // After the flush, and never able to fail the booking: the reservation is
        // made whether or not anyone can be told about it.
        $this->mails->booked($reservation);

        return BookingResult::created($reservation);
    }

    /**
     * Per-resource-kind gate: *may this person use this resource at all?*
     * Machines require their training badges; people must have been made
     * bookable by an admin; spaces are open to any member today. Admins bypass
     * the training gate, as they did before — but not the bookable switch, since
     * a person who isn't offering their time isn't a resource at all.
     *
     * Quotas are **not** here. "How much may you book" is a separate question
     * with a separate answer (BookingPolicyService), and keeping them apart is
     * what stops a lab loosening its safety rules while loosening its limits.
     * Access passes, when they land, belong on the quota side.
     */
    private function checkAccess(ReservableType $type, int $id, Utilisateur $user): ?BookingResult
    {
        if ($type === ReservableType::User) {
            return $this->checkPersonAccess($id, $user);
        }

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

    private function checkPersonAccess(int $id, Utilisateur $user): ?BookingResult
    {
        $person = $this->people->find($id);
        if ($person === null || !$person->isBookable()) {
            return BookingResult::refused(
                'PERSON_NOT_BOOKABLE',
                'Cette personne ne peut pas être réservée.',
                403,
            );
        }

        if ($person->getId() === $user->getId()) {
            return BookingResult::refused(
                'PERSON_SELF_BOOKING',
                'Vous ne pouvez pas vous réserver vous-même. Bloquez plutôt le créneau dans vos disponibilités.',
                400,
            );
        }

        return null;
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
