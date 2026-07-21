<?php

namespace App\Controller;

use App\Repository\AccessRfidLogRepository;
use App\Repository\MachineRepository;
use App\Service\OpeningHoursProvider;
use Doctrine\DBAL\Connection;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * Unlisted full-screen pages meant to be shown on a wall monitor in the lab.
 * They are intentionally not linked from the navigation and auto-refresh.
 */
final class KioskController extends AbstractController
{
    #[Route('/kiosk/entries', name: 'app_kiosk_entries', methods: ['GET'])]
    public function entries(AccessRfidLogRepository $logs): Response
    {
        return $this->render('site/kiosk-entries.html.twig', [
            'entries' => $logs->findRecentWithUser(12),
        ]);
    }

    #[Route('/kiosk/stats', name: 'app_kiosk_stats', methods: ['GET'])]
    public function stats(Connection $db): Response
    {
        $count = static function (Connection $db, string $sql) : int {
            try {
                return (int) $db->fetchOne($sql);
            } catch (\Throwable) {
                return 0;
            }
        };

        return $this->render('site/kiosk-stats.html.twig', [
            'stats' => [
                'members' => $count($db, 'SELECT COUNT(*) FROM UTILISATEUR'),
                'machines' => $count($db, 'SELECT COUNT(*) FROM MACHINE'),
                'projects' => $count($db, 'SELECT COUNT(*) FROM CREATION WHERE isPublished = 1'),
                'formations' => $count($db, 'SELECT COUNT(*) FROM FORMATION'),
                'badges' => $count($db, 'SELECT COUNT(*) FROM UTILISATEUR_BADGE'),
                'visits_today' => $count($db, 'SELECT COUNT(*) FROM ACCESS_RFID_LOG WHERE DATE(createdAt) = CURDATE()'),
            ],
        ]);
    }

    #[Route('/kiosk/machine/{id}', name: 'app_kiosk_machine', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function machineStation(int $id, MachineRepository $machines, OpeningHoursProvider $hours): Response
    {
        $machine = $machines->find($id);
        if ($machine === null) {
            throw $this->createNotFoundException('Machine introuvable');
        }

        return $this->render('site/kiosk-machine.html.twig', [
            'machine' => $machine,
            'openingHours' => $hours->getOpeningHoursForJson(),
            'todayIndex' => (int) (new \DateTimeImmutable())->format('N') - 1,
        ]);
    }
}
