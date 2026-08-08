<?php

namespace App\Reservation;

use App\Service\SiteSettingService;
use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Repository\MachineRepository;
use App\Repository\ReservationRepository;
use App\Repository\UtilisateurRepository;
use App\Reservation\Policy\AccessPassRepository;
use App\Reservation\Policy\BookingPolicyService;
use App\Reservation\Verb\BookingVerb;
use App\Reservation\Verb\BookingVerbService;
use App\Reservation\Verb\VerbContext;
use App\Service\MachineQualificationService;
use App\Feature\SiteFeatureService;
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
 * per-resource access gate, the tier quotas and the access passes that lift
 * them — belongs here, in one readable sequence.
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
        private readonly AccessPassRepository $passes,
        private readonly SiteFeatureService $modules,
        private readonly SiteSettingService $siteSettings,
        private readonly BookingVerbService $verbs,
        private readonly LabClock $clock,
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
        $slot = $this->validate($type, $id, $user, $start, $end, $asRequest);
        if ($slot instanceof BookingResult) {
            return $slot;
        }

        $motif = $motif === null ? null : (trim($motif) ?: null);
        if ($motif !== null && mb_strlen($motif) > self::MOTIF_MAX_LENGTH) {
            return BookingResult::refused('MOTIF_TOO_LONG', 'Le motif ne doit pas dépasser 500 caractères.', 400);
        }

        $reservation = (new Reservation())
            ->setUtilisateur($user)
            ->setDateDebut($start)
            ->setDateFin($end)
            ->setMotif($motif)
            ->setStatut($slot->pending ? Reservation::STATUS_PENDING : Reservation::STATUS_CONFIRMED)
            ->setReservable($type, $id, $slot->name);

        $this->em->persist($reservation);
        $this->em->flush();

        // The booking exists, so the pass has genuinely been used. A failure to
        // record that must not undo the booking — the worst case is a pass that
        // keeps one more use than it should, which staff can see and revoke.
        if ($slot->pass !== null) {
            $this->passes->consume($slot->pass, $reservation->getId(), $user->getId());
        }

        // After the flush, and never able to fail the booking: the reservation is
        // made whether or not anyone can be told about it.
        $this->mails->booked($reservation);

        return BookingResult::created($reservation);
    }

    /**
     * Give the slot back. The booking stays in the history, cancelled.
     *
     * ⚠️ Whether this is *permitted* is not decided here — `BookingVerbService`
     * owns that, and owns it alone, so the page that draws the button and the
     * endpoint that honours it cannot disagree about the rule or about the
     * sentence explaining it. They did before this session.
     */
    public function cancel(
        Reservation $reservation,
        Utilisateur $actor,
        ?\DateTimeImmutable $now = null,
        VerbContext $context = VerbContext::Member,
    ): BookingResult {
        $verdict = $this->verbs->verdict(BookingVerb::Cancel, $reservation, $actor, $now, $context);
        if (!$verdict->allowed) {
            return $verdict->toRefusal();
        }

        $reservation->cancel();
        $this->em->flush();
        $this->mails->cancelled($reservation);

        return BookingResult::updated($reservation);
    }

    /**
     * Undo a cancellation — S47's cancel-with-undo, folded into the same model
     * as every other verb rather than invented a second time.
     *
     * ⚠️ It is **not** a status flip. The slot was released the moment it was
     * cancelled and anyone could have taken it since, so this re-runs the whole
     * booking sequence against the old dates and refuses like a fresh booking if
     * the world moved on. Treating undo as free is how a member ends up
     * double-booked with somebody who did nothing wrong.
     */
    public function restore(Reservation $reservation, Utilisateur $actor, ?\DateTimeImmutable $now = null): BookingResult
    {
        $verdict = $this->verbs->verdict(BookingVerb::Restore, $reservation, $actor, $now);
        if (!$verdict->allowed) {
            return $verdict->toRefusal();
        }

        $type = $reservation->getReservableType();
        $id = $reservation->getReservableId();
        $owner = $reservation->getUtilisateur();
        if ($type === null || $id === null || $owner === null) {
            return BookingResult::refused('RESOURCE_NOT_FOUND', 'Cette réservation ne désigne plus de ressource.', 409);
        }

        // ⚠️ Re-labelled to real instants first. `validate()` compares its dates
        // against a live `now`, and every other caller reaches it from
        // `parseReservationDate()`, which builds the lab zone in. Handing it a
        // raw hydrated column instead would feed it lab digits under a UTC label
        // and shift every comparison by the offset — see LabClock.
        $slot = $this->validate(
            $type,
            $id,
            $owner,
            $this->clock->instantOf($reservation->getDateDebut()),
            $this->clock->instantOf($reservation->getDateFin()),
            // A request that was withdrawn comes back as a request, not as a
            // confirmed booking somebody never agreed to.
            asRequest: true,
            ignoreId: $reservation->getId(),
            consumePass: false,
        );

        if ($slot instanceof BookingResult) {
            return $slot;
        }

        $reservation->setStatut($slot->pending ? Reservation::STATUS_PENDING : Reservation::STATUS_CONFIRMED);
        $this->em->flush();
        $this->mails->booked($reservation);

        return BookingResult::updated($reservation);
    }

    /**
     * "I'm done, take the rest of my slot back." The booking keeps its owner and
     * still counts as honoured; only its end moves.
     *
     * ⚠️ Deliberately not a cancellation and deliberately not subject to any
     * lock window — see `BookingVerb`. It also sends no mail: the only person
     * who could receive one is the member who just pressed the button, standing
     * at the machine.
     */
    public function endNow(Reservation $reservation, Utilisateur $actor, ?\DateTimeImmutable $now = null): BookingResult
    {
        $verdict = $this->verbs->verdict(BookingVerb::EndNow, $reservation, $actor, $now);
        if (!$verdict->allowed) {
            return $verdict->toRefusal();
        }

        // ⚠️ Written in the storage convention, not as an instant. `dateFin` is
        // a convention-B column holding the lab's wall-clock; persisting a raw
        // `now` would write the box's UTC digits into a column every other row
        // states in lab time, and the booking would appear to end hours early.
        $reservation->setDateFin($this->clock->storedFormOf($now ?? $this->clock->now()));
        $this->em->flush();

        return BookingResult::updated($reservation);
    }

    /**
     * Move a booking to another time — the *whole* booking path re-run, then
     * applied.
     *
     * ⚠️ **Nothing is written until every rule has passed**, which is what makes
     * this a swap rather than an edit: if the new slot is taken, or the lab is
     * shut then, or the member is over their weekly cap at the new date, the old
     * booking is still standing untouched and they have lost nothing. The
     * alternative — release the old slot, then try to take the new one — loses
     * the member's slot to whoever books it first at the exact moment they were
     * trying to keep it, which is the cancel-and-rebook dance this replaces.
     *
     * ⚠️ It reuses `validate()` rather than checking "is the new time free". A
     * private path with a subset of the rules is how a member ends up moving a
     * booking onto a Sunday, past their quota, on a machine whose badge expired
     * last week — each of those a rule that exists and was simply not asked.
     */
    public function reschedule(
        Reservation $reservation,
        Utilisateur $actor,
        \DateTimeImmutable $start,
        \DateTimeImmutable $end,
        ?string $motif = null,
        ?\DateTimeImmutable $now = null,
    ): BookingResult {
        $verdict = $this->verbs->verdict(BookingVerb::Reschedule, $reservation, $actor, $now);
        if (!$verdict->allowed) {
            return $verdict->toRefusal();
        }

        $type = $reservation->getReservableType();
        $id = $reservation->getReservableId();
        $owner = $reservation->getUtilisateur();
        if ($type === null || $id === null || $owner === null) {
            return BookingResult::refused('RESOURCE_NOT_FOUND', 'Cette réservation ne désigne plus de ressource.', 409);
        }

        $motif = $motif === null ? null : (trim($motif) ?: null);
        if ($motif !== null && mb_strlen($motif) > self::MOTIF_MAX_LENGTH) {
            return BookingResult::refused('MOTIF_TOO_LONG', 'Le motif ne doit pas dépasser 500 caractères.', 400);
        }

        // ⚠️ Quotas and overlap are asked as if this row did not exist. Without
        // it a booking blocks its own move — the overlap query would find the
        // very slot being vacated, and a member sitting on their cap could never
        // move anything.
        $slot = $this->validate(
            $type,
            $id,
            $owner,
            $start,
            $end,
            // Moving a request keeps it a request; moving a confirmed booking
            // must not silently become one.
            asRequest: $reservation->isPending(),
            ignoreId: $reservation->getId(),
            consumePass: false,
        );

        if ($slot instanceof BookingResult) {
            return $slot;
        }

        $previousStart = $reservation->getDateDebut();
        $previousEnd = $reservation->getDateFin();

        $reservation
            ->setDateDebut($start)
            ->setDateFin($end)
            ->setMotif($motif)
            ->setStatut($slot->pending ? Reservation::STATUS_PENDING : Reservation::STATUS_CONFIRMED);

        $this->em->flush();
        $this->mails->moved($reservation, $previousStart, $previousEnd);

        return BookingResult::updated($reservation);
    }

    /**
     * Every rule that decides whether a slot may be held, in one sequence, run
     * identically by `book()` and by the verbs that move or restore a booking.
     *
     * ⚠️ **This is the whole point of S77's reschedule being "not a mutation".**
     * A move is re-validated from scratch, so it can never land somewhere a
     * fresh booking could not — outside opening hours, on a machine the member
     * has since lost their badge for, past their weekly cap. The only difference
     * a move gets is `$ignoreId`, which hides the row being moved from the
     * overlap and quota queries so it does not block itself.
     *
     * @return ValidatedSlot|BookingResult the refusal, or what the caller needs to commit
     */
    private function validate(
        ReservableType $type,
        int $id,
        Utilisateur $user,
        \DateTimeImmutable $start,
        \DateTimeImmutable $end,
        bool $asRequest,
        ?int $ignoreId = null,
        bool $consumePass = true,
    ): ValidatedSlot|BookingResult {
        // A resource layer that has been switched off must stop *taking* bookings,
        // not merely stop rendering. Its pages already 404, but /api/reservations
        // speaks the polymorphic payload directly and would otherwise keep
        // accepting bookings for a kind the deployment says it does not offer.
        // Here rather than in the controllers because this is the chokepoint every
        // path goes through — the calendar form, the person pages and the API alike.
        if (!$this->modules->allowsReservable($type)) {
            return BookingResult::refused(
                'RESOURCE_KIND_UNAVAILABLE',
                $this->notFoundMessage($type),
                404,
            );
        }

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

        $now = new \DateTimeImmutable('now', new \DateTimeZone($this->siteSettings->getTimezone()));
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

        // Resolved here but spent after the flush: a pass must not be consumed by
        // a booking that then fails on an overlap, and resolving it a second time
        // later would risk consuming it twice.
        $pass = $this->passes->findApplicable($user, $type, $id, $now);

        // Quotas run after the sanity checks rather than alongside the access
        // gate: "your booking is too long" only makes sense once we know the end
        // is after the start, and telling someone they're over their weekly
        // limit for a slot the lab is closed on would be answering the wrong
        // question. Unconfigured labs return here immediately.
        $quotaRefusal = $this->policies->check($user, $type, $id, $start, $end, $now, $pass !== null, $ignoreId);
        if ($quotaRefusal !== null) {
            return $quotaRefusal;
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
        if ($this->reservations->hasOverlap($type, $id, $start, $end, $ignoreId)) {
            return BookingResult::refused('RESERVATION_OVERLAP', $this->overlapMessage($type), 409);
        }

        // ⚠️ A move honours a pass but never spends one. The member is not
        // gaining a booking they haven't already paid for — the count is
        // unchanged — so charging a second pass to move the same slot by an hour
        // would be a fine for changing your mind. Refusing to look for one at all
        // would be worse: a booking made on a pass could then be unmovable
        // because the plain quota refuses its new time.
        return new ValidatedSlot($name, $pending, $consumePass ? $pass : null);
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
