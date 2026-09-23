<?php

namespace App\Rfid;

use App\Entity\AccessPoint;
use App\Entity\Utilisateur;
use App\Reservation\ReservableType;
use App\Repository\ReservationRepository;
use App\Reservation\LabClock;

/**
 * Une personne peut-elle ouvrir CETTE porte, MAINTENANT ? (S178)
 *
 * 🔴 **Il n'y a rien à révoquer, et c'est tout le dessin.** La feuille de route
 * demandait « un accès temporaire lié à une réservation, révoqué à
 * l'annulation ». La façon évidente serait d'ACCORDER quelque chose à la
 * réservation — une ligne d'autorisation, une fenêtre stockée — puis de la
 * retirer à l'annulation. Ce serait une seconde source de vérité sur qui peut
 * entrer, et le jour où la révocation échoue (transaction, worker, bug), la
 * porte s'ouvre encore pour une réservation annulée.
 *
 * ✅ **Ici, rien n'est accordé : la question est reposée à chaque badge.** Une
 * réservation annulée n'est plus rendue par le dépôt, donc la porte se ferme au
 * scan suivant — **immédiatement, et sans qu'aucun code de révocation existe**.
 * « Révoqué à l'annulation » n'est pas une fonctionnalité à écrire, c'est une
 * conséquence à ne pas casser.
 *
 * ⚠️ **La marge est une MARGE, pas un cadeau.** Quinze minutes avant permettent
 * d'arriver un peu tôt sans se faire refuser à la porte ; quinze après, de
 * ranger. Les allonger revient à donner un accès hors réservation, et c'est
 * précisément le genre de valeur qu'on augmente « juste un peu » jusqu'à ce
 * qu'elle ne veuille plus rien dire.
 * 🅿️ Elle est ici en constante et pas en réglage : personne n'a encore fait
 * tourner une porte, donc personne ne sait quelle valeur convient. Un réglage
 * qu'on ne sait pas régler est un écran de plus, pas une réponse.
 *
 * 🔴 **Les heures de réservation sont des heures MURALES** (convention B, voir
 * `LabClock`) : `dateDebut`/`dateFin` stockent les DIGITS que quelqu'un a saisis,
 * sans zone. Les comparer directement à un instant réel déplace chaque décision
 * du décalage du labo — deux heures en été, largement de quoi ouvrir une porte
 * pour la mauvaise réservation, ou la refuser à la bonne. `instantOf()` les
 * ré-étiquette ; c'est la seule conversion correcte, et l'omettre est
 * SILENCIEUX. Voir [[feedback-fabos-timezone-hazard]].
 */
final class DoorAccessDecision
{
    /** Quinze minutes de chaque côté du créneau. */
    public const MARGIN_MINUTES = 15;

    public function __construct(
        private readonly ReservationRepository $reservations,
        private readonly LabClock $clock,
    ) {
    }

    /**
     * @return array{allowed: bool, status: string, reservationId: ?int}
     *         `status` est une CLÉ, jamais une phrase : elle part dans le
     *         journal d'accès et dans `AccessIncident`, qui décident chacun de
     *         ce qu'ils en montrent.
     */
    public function decide(AccessPoint $point, ?Utilisateur $user, ?\DateTimeImmutable $at = null): array
    {
        if ($user === null) {
            return ['allowed' => false, 'status' => 'unknown_rfid', 'reservationId' => null];
        }
        if ($user->getStatut() !== 'actif') {
            // S190 — même règle qu'aux machines : une réservation encore au
            // calendrier ne rouvre pas la porte d'un compte désactivé.
            return ['allowed' => false, 'status' => 'account_inactive', 'reservationId' => null];
        }
        if ($point->isArchived()) {
            // ⚠️ Une porte archivée n'ouvre plus, même pour qui a une
            // réservation : l'exploitant l'a retirée du service, et c'est une
            // décision qui doit gagner sur toutes les autres.
            return ['allowed' => false, 'status' => 'access_point_archived', 'reservationId' => null];
        }

        $place = $point->getPlace();
        if ($place === null) {
            /*
             * 🔴 **Une porte qui n'ouvre AUCUN espace nommé ne peut pas être
             * décidée par réservation** — il n'y a rien à réserver. C'est le cas
             * du portail d'entrée, et il est légitime : sa règle viendra des
             * axes lieu/jours/horaires d'un forfait, pas d'ici. Rendre `true`
             * « parce que c'est l'entrée » ouvrirait le bâtiment à quiconque
             * badge.
             */
            return ['allowed' => false, 'status' => 'no_place_bound', 'reservationId' => null];
        }

        $now = $at ?? $this->clock->now();
        $from = $now->modify('+' . self::MARGIN_MINUTES . ' minutes');
        $until = $now->modify('-' . self::MARGIN_MINUTES . ' minutes');

        foreach ($this->reservations->findActiveForReservable(ReservableType::Place, (int) $place->getId()) as $reservation) {
            if ($reservation->getUtilisateur()?->getId() !== $user->getId()) {
                continue;
            }
            // ⚠️ `instantOf()` des DEUX côtés — sans lui on compare des digits
            // sans zone à un instant réel.
            $start = $this->clock->instantOf($reservation->getDateDebut());
            $end = $this->clock->instantOf($reservation->getDateFin());
            // Le créneau, élargi de la marge des deux côtés : commencé avant
            // `now + marge`, pas fini avant `now - marge`.
            if ($start <= $from && $end >= $until) {
                return ['allowed' => true, 'status' => 'booking_in_progress', 'reservationId' => $reservation->getId()];
            }
        }

        return ['allowed' => false, 'status' => 'no_booking_now', 'reservationId' => null];
    }
}
