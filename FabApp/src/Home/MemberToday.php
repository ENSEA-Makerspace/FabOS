<?php

declare(strict_types=1);

namespace App\Home;

use App\Entity\Utilisateur;
use App\Repository\LoanRepository;
use App\Repository\ProgressionRepository;
use App\Repository\ReservationRepository;
use App\Reservation\LabClock;
use App\UsageRights\RightsExplainer;
use Doctrine\DBAL\Connection;

/**
 * « Ce qui compte aujourd'hui » pour un membre connecté (2026-10-01, d'après la
 * planche `users/04-accueil-membre.png`) : ce qu'il lui reste à faire, avec le
 * verbe qui le fait ; sa prochaine réservation ; ce que ses badges ouvrent.
 *
 * ⚠️ Ne calcule RIEN de neuf : chaque ligne vient d'un service ou d'un dépôt que
 * le profil lit déjà. Ce n'est qu'un tri, dans l'ordre de l'urgence.
 */
final class MemberToday
{
    public function __construct(
        private readonly ProgressionRepository $progressions,
        private readonly LoanRepository $loans,
        private readonly ReservationRepository $reservations,
        private readonly RightsExplainer $explainer,
        private readonly LabClock $clock,
        private readonly Connection $db,
    ) {
    }

    /** @return array{todo: list<array<string, mixed>>, next: ?\App\Entity\Reservation, nextIsNow: bool, explained: array<string, mixed>} */
    public function for(Utilisateur $user): array
    {
        $todo = [];

        // 1. Ce qui est en retard d'abord : un objet à rendre.
        foreach ($this->loans->findForBorrower($user) as $loan) {
            $status = $loan->getEffectiveStatus();
            if ($status === 'returned') {
                continue;
            }
            $todo[] = [
                'kind' => 'loan', 'tone' => $status === 'overdue' ? 'stop' : 'caution', 'icon' => 'box',
                'loan' => $loan, 'overdue' => $status === 'overdue',
            ];
        }

        // 2. Les formations commencées et pas finies : « Continuer ».
        foreach ($this->progressions->findVisibleByUser($user) as $progression) {
            if (!$progression->isCompleted() && $progression->getFormation() !== null) {
                $todo[] = ['kind' => 'training', 'tone' => 'wait', 'icon' => 'tool', 'progression' => $progression];
            }
        }

        // 3. Les demandes de réservation encore en attente.
        $next = null;
        $nextIsNow = false;
        $now = $this->clock->now();
        foreach ($this->reservations->findForUser($user, ['dateDebut' => 'ASC']) as $reservation) {
            if (!$reservation->isActive() || $this->clock->instantOf($reservation->getDateFin()) < $now) {
                continue;
            }
            if ($reservation->isPending()) {
                $todo[] = ['kind' => 'pending', 'tone' => 'wait', 'icon' => 'hourglass', 'reservation' => $reservation];
            }
            if ($next === null) {
                $next = $reservation;
                $nextIsNow = $this->clock->instantOf($reservation->getDateDebut()) <= $now;
            }
        }

        // 4. Aucun badge : la première chose à faire est une formation.
        $badges = (int) $this->db->fetchOne('SELECT COUNT(*) FROM UTILISATEUR_BADGE WHERE utilisateurId = ?', [$user->getId()]);
        if ($badges === 0) {
            $todo[] = ['kind' => 'first_badge', 'tone' => 'caution', 'icon' => 'key'];
        }

        // 5. Des droits, mais aucune carte enregistrée : le lecteur ne la connaît pas.
        $explained = $this->explainer->explain($user);
        if ($badges > 0 && !($explained['badge']['registered'] ?? false)) {
            $todo[] = ['kind' => 'card', 'tone' => 'caution', 'icon' => 'key'];
        }

        return [
            'todo' => $todo,
            'next' => $next,
            'nextIsNow' => $nextIsNow,
            'explained' => $explained,
        ];
    }
}
