<?php

namespace App\Controller;

use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

#[IsGranted('ROLE_ADMIN')]
final class LegacyAdminController extends AbstractController
{
    #[Route('/admin-dashboard.html', name: 'app_admin_dashboard_html', methods: ['GET'])]
    public function dashboard(): Response { return $this->redirectToRoute('app_admin_dashboard'); }

    #[Route('/admin-machines.html', name: 'app_admin_machines_html', methods: ['GET'])]
    public function machines(): Response { return $this->redirectToRoute('app_admin_machines'); }

    #[Route('/admin-formations.html', name: 'app_admin_formations_html', methods: ['GET'])]
    public function formations(): Response { return $this->redirectToRoute('app_admin_formations'); }

    #[Route('/admin-reservations.html', name: 'app_admin_reservations_html', methods: ['GET'])]
    public function reservations(): Response { return $this->redirectToRoute('app_admin_reservations'); }

    #[Route('/admin-utilisateurs.html', name: 'app_admin_users_html', methods: ['GET'])]
    public function users(): Response { return $this->redirectToRoute('app_admin_users'); }

    #[Route('/admin-badges.html', name: 'app_admin_badges_html', methods: ['GET'])]
    public function badges(): Response { return $this->redirectToRoute('app_admin_badges'); }
}
