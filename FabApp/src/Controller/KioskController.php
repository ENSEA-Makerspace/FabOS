<?php

namespace App\Controller;

use App\Repository\AccessRfidLogRepository;
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
}
