<?php

declare(strict_types=1);

namespace App\Service;

use App\Mail\Mailer;
use App\Mail\NotificationCategory;
use App\Repository\UtilisateurRepository;

/**
 * S205 — prévient l'équipe d'un nouveau signalement de panne, par le système
 * d'e-mail existant (gabarit `machine_report_received`, modifiable dans l'admin
 * comme les autres ; catégorie MAINTENANCE, que chaque membre du personnel peut
 * couper dans ses préférences).
 *
 * ⚠️ Un e-mail qui échoue ne doit JAMAIS faire perdre le signalement : tout est
 * déjà en base quand on arrive ici, et chaque erreur est avalée.
 */
final class MachineReportNotifier
{
    public function __construct(
        private readonly Mailer $mailer,
        private readonly UtilisateurRepository $people,
    ) {
    }

    /** @return int le nombre d'e-mails mis en file */
    public function notify(string $machineName, string $description, ?string $contact, string $link): int
    {
        $sent = 0;
        try {
            foreach ($this->people->findStaff() as $member) {
                if ($this->mailer->queueToUser($member, 'machine_report_received', [
                    'machine' => $machineName,
                    'description' => $description,
                    'contact' => (string) $contact,
                    'link' => $link,
                ], NotificationCategory::MAINTENANCE, false)) {
                    ++$sent;
                }
            }
        } catch (\Throwable) {
            // voir la note de classe
        }

        return $sent;
    }
}
