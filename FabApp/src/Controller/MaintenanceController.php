<?php

namespace App\Controller;

use App\Repository\MaintenanceTaskRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * Public maintenance backlog — open (due/overdue) tasks across the lab, for
 * transparency (the HSE/safety team values this). Read-only; creation is
 * admin-only. Gated by the admin-toggleable `maintenance` module (app_maintenance
 * route prefix → ModuleAccessSubscriber).
 */
final class MaintenanceController extends AbstractController
{
    #[Route('/maintenance', name: 'app_maintenance', methods: ['GET'])]
    public function backlog(MaintenanceTaskRepository $tasks): Response
    {
        return $this->render('site/maintenance.html.twig', [
            'tasks' => $tasks->findOpenSafe(),
        ]);
    }
}
