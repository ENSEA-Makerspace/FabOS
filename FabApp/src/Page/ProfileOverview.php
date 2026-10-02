<?php

declare(strict_types=1);

namespace App\Page;

use App\Entity\Utilisateur;
use App\Home\MemberToday;
use App\Repository\EventRegistrationRepository;
use App\Repository\LoanRepository;
use App\Repository\ReservationRepository;

/**
 * « Mon compte » — proposition du 2026-10-02 (demande de l'opérateur : « il faut
 * scroller pour voir les infos utiles, trop de petit texte, pas assez utile »).
 *
 * ⚠️ Ne calcule rien de neuf : tout vient des services que `/profil` et l'accueil
 * lisent déjà (`MemberToday`, `MyTrainings`, `AccountSecurityEmails`, l'explicateur
 * de droits). Ce service ne fait que RÉSUMER : un droit répété quatre fois devient
 * une ligne, un tableau vide devient une absence.
 */
final class ProfileOverview
{
    public function __construct(
        private readonly MemberToday $today,
        private readonly MyTrainings $trainings,
        private readonly AccountSecurityEmails $account,
        private readonly LoanRepository $loans,
        private readonly EventRegistrationRepository $registrations,
        private readonly ReservationRepository $reservations,
    ) {
    }

    /** @return array<string, mixed> */
    public function for(Utilisateur $user): array
    {
        $today = $this->today->for($user);
        $explained = $today['explained'];

        // Les droits : ce qui est permis, en UNE ligne, et d'où ça vient, une fois.
        $allowed = [];
        $denied = [];
        $sources = [];
        foreach ($explained['capabilities'] as $row) {
            if ($row['verdict']->allowed) {
                $allowed[] = $row['capability']->labelKey;
            } else {
                $denied[] = ['labelKey' => $row['capability']->labelKey, 'reason' => $row['verdict']->reason];
            }
            foreach ($row['paths'] as $path) {
                $key = (string) $path['package'] . '|' . (string) ($path['group'] ?? '');
                $sources[$key] ??= ['package' => $path['package'], 'group' => $path['group'] ?? null, 'until' => $path['until'] ?? null];
            }
        }

        $loans = array_values(array_filter(
            $this->loans->findForBorrower($user),
            static fn ($loan): bool => $loan->getEffectiveStatus() !== 'returned',
        ));
        $now = new \DateTimeImmutable();
        $events = array_values(array_filter(
            $this->registrations->findForUser($user),
            static fn ($registration): bool => $registration->getEvent() !== null && $registration->getEvent()->getDateDebut() >= $now,
        ));

        $account = $this->account->for($user);
        $optional = 0;
        $optionalOn = 0;
        foreach ($account['groups'] as $group) {
            foreach ($group['lines'] as $line) {
                if (!$line['essential']) {
                    ++$optional;
                    $optionalOn += ($line['received'] && $account['masterOn']) ? 1 : 0;
                }
            }
        }

        return [
            'today' => $today,
            'trainings' => $this->trainings->for($user),
            'account' => $account,
            'emails' => ['optional' => $optional, 'optionalOn' => $optionalOn],
            'rights' => ['allowed' => $allowed, 'denied' => $denied, 'sources' => array_values($sources)],
            'loans' => $loans,
            'events' => $events,
            'reservationCount' => $this->reservations->count(['utilisateur' => $user]),
        ];
    }
}
