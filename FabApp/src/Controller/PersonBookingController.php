<?php

namespace App\Controller;

use App\Service\SiteSettingService;
use App\Entity\UserAvailability;
use App\Entity\Utilisateur;
use App\Repository\ReservationRepository;
use App\Repository\UserAvailabilityRepository;
use App\Repository\UtilisateurRepository;
use App\Reservation\PersonAvailabilityService;
use App\Reservation\ReservableType;
use App\Reservation\ReservationMailer;
use App\Reservation\ReservationService;
use App\UsageRights\UsageRightsService;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * Booking a person — the last resource kind the polymorphic Reservation model
 * was built for. Two ways in, deliberately:
 *
 *  - the offered slots, computed from the person's weekly availability, which
 *    confirm immediately (they already said yes to that hour); and
 *  - a free-form request for any other time, which lands as pending and is the
 *    person's to accept or decline.
 *
 * Gated by `person_booking`, its **own** resource module — never by the staff or
 * trainers directories. A bookable person may be staff, a trainer, both or
 * neither, so which pages exist here stands on the person's own `bookable` flag
 * (admin-set) and on whether this deployment books people at all; publishing a
 * team directory is a separate question with a separate switch.
 *
 * Bookings themselves still go through ReservationService like every other kind,
 * which is also where the module is enforced for the API.
 */
final class PersonBookingController extends AbstractController
{
    #[Route('/personnes/{id}/reserver', name: 'app_person_booking', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function booking(
        int $id,
        Request $request,
        UtilisateurRepository $people,
        PersonAvailabilityService $availability,
        ReservationRepository $reservations,
        UsageRightsService $usageRights,
    ): Response {
        $person = $this->findBookablePerson($people, $id);
        $durations = $person->getBookingDurationsMinutes();

        $duration = (int) $request->query->get('duree', (string) $durations[0]);
        if (!in_array($duration, $durations, true)) {
            $duration = $durations[0];
        }

        return $this->render('site/person-booking.html.twig', [
            'person' => $person,
            'durations' => $durations,
            'selectedDuration' => $duration,
            'days' => $availability->dailySlots($person, $duration),
            'weeklyWindows' => $availability->weeklyWindows($person),
            'upcoming' => $reservations->findUpcomingForReservable(ReservableType::User, $id),
            'horizonDays' => PersonAvailabilityService::HORIZON_DAYS,
            'usageRight' => $usageRights->verdict($this->getUser() instanceof Utilisateur ? $this->getUser() : null, 'person_booking'),
        ]);
    }

    #[Route('/personnes/{id}/reserver', name: 'app_person_book', requirements: ['id' => '\d+'], methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function book(
        int $id,
        Request $request,
        UtilisateurRepository $people,
        ReservationService $booking,
        SiteSettingService $siteSettings,
    ): Response {
        $person = $this->findBookablePerson($people, $id);
        $user = $this->currentUser();

        if (!$this->isCsrfTokenValid('person_book_' . $id, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Réservation refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_person_booking', ['id' => $id]);
        }

        $start = $this->parseMoment($siteSettings, (string) $request->request->get('start'));
        $duration = (int) $request->request->get('duration');
        if ($start === null || $duration <= 0) {
            $this->addFlash('error', 'Créneau invalide.');

            return $this->redirectToRoute('app_person_booking', ['id' => $id]);
        }

        $result = $booking->book(
            ReservableType::User,
            $id,
            $user,
            $start,
            $start->modify(sprintf('+%d minutes', $duration)),
            (string) $request->request->get('motif'),
        );

        $this->addFlash(
            $result->ok ? 'success' : 'error',
            $result->ok ? sprintf('Rendez-vous confirmé avec %s.', $person->getDisplayName()) : $result->message,
        );

        return $this->redirectToRoute('app_person_booking', ['id' => $id, 'duree' => $duration]);
    }

    /**
     * The "out of the blue" path: any date and time, whether or not the person
     * offered it. Lands pending — this asks, it doesn't book.
     */
    #[Route('/personnes/{id}/demande', name: 'app_person_request', requirements: ['id' => '\d+'], methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function requestSlot(
        int $id,
        Request $request,
        UtilisateurRepository $people,
        ReservationService $booking,
        SiteSettingService $siteSettings,
    ): Response {
        $this->findBookablePerson($people, $id);
        $user = $this->currentUser();

        if (!$this->isCsrfTokenValid('person_request_' . $id, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Demande refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_person_booking', ['id' => $id]);
        }

        $start = $this->parseMoment($siteSettings, (string) $request->request->get('date') . ' ' . (string) $request->request->get('startTime')
        );
        $duration = (int) $request->request->get('duration');
        if ($start === null || $duration <= 0) {
            $this->addFlash('error', 'Date ou durée invalide.');

            return $this->redirectToRoute('app_person_booking', ['id' => $id]);
        }

        $result = $booking->book(
            ReservableType::User,
            $id,
            $user,
            $start,
            $start->modify(sprintf('+%d minutes', $duration)),
            (string) $request->request->get('motif'),
            asRequest: true,
        );

        $this->addFlash(
            $result->ok ? 'success' : 'error',
            $result->ok
                ? 'Demande envoyée. Elle est en attente de réponse.'
                : $result->message,
        );

        return $this->redirectToRoute('app_person_booking', ['id' => $id]);
    }

    /**
     * The person's own side: their weekly windows, the lengths they offer, and
     * the requests waiting on them.
     */
    #[Route('/mes-disponibilites', name: 'app_person_my_availability', methods: ['GET'])]
    #[IsGranted('ROLE_USER')]
    public function myAvailability(
        UserAvailabilityRepository $availabilities,
        ReservationRepository $reservations,
    ): Response {
        $user = $this->currentUser();

        return $this->render('site/my-availability.html.twig', [
            'person' => $user,
            'windowsByDay' => $availabilities->findForUserByDay($user),
            'durations' => $user->getBookingDurationsMinutes(),
            'pendingRequests' => $reservations->findPendingForReservable(ReservableType::User, (int) $user->getId()),
            'upcoming' => $reservations->findUpcomingForReservable(ReservableType::User, (int) $user->getId()),
        ]);
    }

    #[Route('/mes-disponibilites', name: 'app_person_my_availability_save', methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function saveAvailability(
        Request $request,
        UserAvailabilityRepository $availabilities,
        EntityManagerInterface $em,
    ): Response {
        $user = $this->currentUser();

        if (!$this->isCsrfTokenValid('my_availability', (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Enregistrement refusé : token CSRF invalide.');

            return $this->redirectToRoute('app_person_my_availability');
        }

        $user
            ->setBookingDurations((string) $request->request->get('durations'))
            ->setBookingNote((string) $request->request->get('note'));
        $em->flush();

        $windows = [];
        /** @var array<int, array{start?: string, end?: string}> $rows */
        $rows = $request->request->all('window');
        foreach ($rows as $dayOfWeek => $row) {
            $start = $this->parseTime((string) ($row['start'] ?? ''));
            $end = $this->parseTime((string) ($row['end'] ?? ''));
            if ($start === null || $end === null || $end <= $start) {
                continue;
            }

            $windows[] = (new UserAvailability())
                ->setDayOfWeek((int) $dayOfWeek)
                ->setStartTime($start)
                ->setEndTime($end);
        }

        $availabilities->replaceForUser($user, $windows);

        $this->addFlash('success', 'Disponibilités enregistrées.');

        return $this->redirectToRoute('app_person_my_availability');
    }

    #[Route('/mes-disponibilites/{id}/repondre', name: 'app_person_request_answer', requirements: ['id' => '\d+'], methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function answerRequest(
        int $id,
        Request $request,
        ReservationRepository $reservations,
        EntityManagerInterface $em,
        ReservationMailer $mails,
    ): Response {
        $user = $this->currentUser();
        $reservation = $reservations->find($id);

        // Only the person the request was addressed to answers it — being the
        // booked resource is what grants the right, not owning the row.
        if ($reservation === null || !$reservation->isFor(ReservableType::User, (int) $user->getId())) {
            throw $this->createNotFoundException('Demande introuvable');
        }

        if (!$this->isCsrfTokenValid('answer_request_' . $id, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Réponse refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_person_my_availability');
        }

        if (!$reservation->isPending()) {
            $this->addFlash('error', 'Cette demande a déjà été traitée.');

            return $this->redirectToRoute('app_person_my_availability');
        }

        $accepted = $request->request->get('answer') === 'accept';
        if ($accepted) {
            // The slot was held while pending, but a confirmed booking could have
            // landed on it through another path in the meantime.
            if ($reservations->hasOverlap(
                ReservableType::User,
                (int) $user->getId(),
                $reservation->getDateDebut(),
                $reservation->getDateFin(),
                $reservation->getId(),
            )) {
                $this->addFlash('error', 'Ce créneau est désormais occupé par une autre réservation.');

                return $this->redirectToRoute('app_person_my_availability');
            }

            $reservation->accept();
        } else {
            $reservation->decline();
        }

        $em->flush();
        $mails->answered($reservation, $accepted);
        $this->addFlash('success', $accepted ? 'Demande acceptée.' : 'Demande refusée.');

        return $this->redirectToRoute('app_person_my_availability');
    }

    private function findBookablePerson(UtilisateurRepository $people, int $id): Utilisateur
    {
        $person = $people->find($id);
        if ($person === null || !$person->isBookable()) {
            throw $this->createNotFoundException('Cette personne n’est pas réservable.');
        }

        return $person;
    }

    private function currentUser(): Utilisateur
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException('Authentification requise');
        }

        return $user;
    }

    /** Parses "Y-m-d H:i" (or the datetime-local "Y-m-d\TH:i") in the lab's timezone. */
    private function parseMoment(SiteSettingService $siteSettings, string $value): ?\DateTimeImmutable
    {
        $value = trim($value);
        if ($value === '') {
            return null;
        }

        try {
            return new \DateTimeImmutable($value, $this->labZone($siteSettings));
        } catch (\Throwable) {
            return null;
        }
    }

    private function parseTime(string $value): ?\DateTime
    {
        $time = \DateTime::createFromFormat('!H:i', trim($value));

        return $time === false ? null : $time;
    }

    /**
     * The lab's wall-clock zone, from the operator's setting.
     *
     * ⚠️ Human-entered times are parsed **and stored** in this zone, so the naive
     * string in the database keeps the time the person actually typed (the audit is
     * S38b in docs/HISTORY.md). Machine timestamps follow the opposite rule and are
     * stored UTC — those are converted at display time by the `|lab_date` filter,
     * never here.
     */
    private function labZone(SiteSettingService $siteSettings): \DateTimeZone
    {
        return new \DateTimeZone($siteSettings->getTimezone());
    }

}
