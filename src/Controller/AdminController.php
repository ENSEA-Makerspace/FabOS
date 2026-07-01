<?php

namespace App\Controller;

use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

#[Route('/admin')]
final class AdminController extends AbstractController
{
    #[Route('', name: 'app_admin_dashboard', methods: ['GET'])]
    #[Route('/dashboard', name: 'app_admin_dashboard_alt', methods: ['GET'])]
    #[Route('/dashboard.html', name: 'app_admin_dashboard_scoped_html', methods: ['GET'])]
    public function dashboard(): Response
    {
        return $this->render('site/admin-dashboard.html.twig');
    }

    #[Route('/machines', name: 'app_admin_machines', methods: ['GET'])]
    #[Route('/machines.html', name: 'app_admin_machines_scoped_html', methods: ['GET'])]
    #[Route('/admin-machines.html', name: 'app_admin_machines_double_legacy_html', methods: ['GET'])]
    public function machines(): Response
    {
        return $this->render('site/admin-machines.html.twig');
    }

    #[Route('/formations', name: 'app_admin_formations', methods: ['GET'])]
    #[Route('/formations.html', name: 'app_admin_formations_scoped_html', methods: ['GET'])]
    #[Route('/admin-formations.html', name: 'app_admin_formations_double_legacy_html', methods: ['GET'])]
    public function formations(): Response
    {
        return $this->render('site/admin-formations.html.twig');
    }

    #[Route('/reservations', name: 'app_admin_reservations', methods: ['GET'])]
    #[Route('/reservations.html', name: 'app_admin_reservations_scoped_html', methods: ['GET'])]
    #[Route('/admin-reservations.html', name: 'app_admin_reservations_double_legacy_html', methods: ['GET'])]
    public function reservations(): Response
    {
        return $this->render('site/admin-reservations.html.twig');
    }

    #[Route('/utilisateurs', name: 'app_admin_users', methods: ['GET'])]
    #[Route('/utilisateurs.html', name: 'app_admin_users_scoped_html', methods: ['GET'])]
    #[Route('/admin-utilisateurs.html', name: 'app_admin_users_double_legacy_html', methods: ['GET'])]
    public function users(): Response
    {
        return $this->render('site/admin-utilisateurs.html.twig');
    }

    #[Route('/badges', name: 'app_admin_badges', methods: ['GET'])]
    #[Route('/badges.html', name: 'app_admin_badges_scoped_html', methods: ['GET'])]
    #[Route('/admin-badges.html', name: 'app_admin_badges_double_legacy_html', methods: ['GET'])]
    public function badges(): Response
    {
        return $this->render('site/admin-badges.html.twig');
    }

    #[Route('/admin-dashboard.html', name: 'app_admin_dashboard_legacy_html', methods: ['GET'])]
    public function legacyDashboard(): Response { return $this->dashboard(); }
}
