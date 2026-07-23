<?php

namespace App\Reservation;

use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Mail\Mailer;
use App\Mail\NotificationCategory;
use App\Repository\UtilisateurRepository;

/**
 * Every booking-related mail, in one place: which template goes to whom for each
 * transition, and the shared context they all render from.
 *
 * Nothing here is allowed to fail loudly — a booking that succeeded must not be
 * reported as an error because the mail queue hiccuped, and a person's request
 * must still be accepted if the requester's address bounces. Mailer already
 * refuses quietly when mail isn't set up; this swallows the rest.
 */
final class ReservationMailer
{
    public function __construct(
        private readonly Mailer $mailer,
        private readonly UtilisateurRepository $people,
    ) {
    }

    /**
     * A booking that was just created: confirmed bookings only tell the booker,
     * a request also has to reach the person being asked — they're the one who
     * has to do something about it.
     */
    public function booked(Reservation $reservation): void
    {
        $this->guard(function () use ($reservation): void {
            $booker = $reservation->getUtilisateur();
            $context = $this->context($reservation);

            if ($reservation->isPending()) {
                if ($booker !== null) {
                    $this->mailer->queueToUser($booker, 'booking_requested', $context, NotificationCategory::BOOKING);
                }

                $person = $this->bookedPerson($reservation);
                if ($person !== null) {
                    $this->mailer->queueToUser($person, 'booking_request_received', $context, NotificationCategory::BOOKING);
                }

                return;
            }

            if ($booker !== null) {
                $this->mailer->queueToUser($booker, 'booking_confirmed', $context, NotificationCategory::BOOKING);
            }
        });
    }

    /** The person answered a pending request — the requester is the one waiting on it. */
    public function answered(Reservation $reservation, bool $accepted): void
    {
        $this->guard(function () use ($reservation, $accepted): void {
            $booker = $reservation->getUtilisateur();
            if ($booker !== null) {
                $this->mailer->queueToUser($booker, $accepted ? 'booking_accepted' : 'booking_declined', $this->context($reservation), NotificationCategory::BOOKING);
            }
        });
    }

    /**
     * A booking was cancelled — by its owner, by an admin, or as a side effect of
     * the resource going away. The booker always hears about it; for a booking of
     * someone's time, so does the person whose slot just freed up.
     */
    public function cancelled(Reservation $reservation): void
    {
        $this->guard(function () use ($reservation): void {
            $context = $this->context($reservation);

            $booker = $reservation->getUtilisateur();
            if ($booker !== null) {
                $this->mailer->queueToUser($booker, 'booking_cancelled', $context, NotificationCategory::BOOKING);
            }

            $person = $this->bookedPerson($reservation);
            if ($person !== null) {
                $this->mailer->queueToUser($person, 'booking_cancelled', $context, NotificationCategory::BOOKING);
            }
        });
    }

    /** @param Reservation[] $reservations */
    public function cancelledBatch(array $reservations): void
    {
        foreach ($reservations as $reservation) {
            $this->cancelled($reservation);
        }
    }

    /** @return array<string, mixed> */
    private function context(Reservation $reservation): array
    {
        $booker = $reservation->getUtilisateur();

        return [
            // ISO dates, not formatted strings: the context is stored as JSON and
            // re-rendered by the worker, possibly in another recipient's language.
            'start' => $reservation->getDateDebut()->format(\DATE_ATOM),
            'end' => $reservation->getDateFin()->format(\DATE_ATOM),
            'resource' => $reservation->getReservableLabel() ?? '',
            'kind' => $reservation->getReservableType()?->value ?? '',
            'motif' => $reservation->getMotif(),
            'booker' => $booker !== null ? $this->displayName($booker) : '',
        ];
    }

    /** The person being booked, when the resource is somebody's time. */
    private function bookedPerson(Reservation $reservation): ?Utilisateur
    {
        if ($reservation->getReservableType() !== ReservableType::User) {
            return null;
        }

        $id = $reservation->getReservableId();

        return $id === null ? null : $this->people->find($id);
    }

    private function displayName(Utilisateur $user): string
    {
        $name = trim(($user->getFirstName() ?? '') . ' ' . ($user->getLastName() ?? ''));

        return $name !== '' ? $name : $user->getEmail();
    }

    private function guard(callable $send): void
    {
        try {
            $send();
        } catch (\Throwable) {
            // Notifying is never worth failing the action that triggered it.
        }
    }
}
