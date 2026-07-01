<?php

namespace App\Controller;

use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

final class LegacyAdminController extends AbstractController
{
    #[Route('/admin-dashboard.html', name: 'app_admin_dashboard_html', methods: ['GET'])]
    public function dashboard(): Response { return $this->render('site/admin-dashboard.html.twig'); }

    #[Route('/admin-machines.html', name: 'app_admin_machines_html', methods: ['GET'])]
    public function machines(): Response { return $this->render('site/admin-machines.html.twig'); }

    #[Route('/admin-formations.html', name: 'app_admin_formations_html', methods: ['GET'])]
    public function formations(): Response { return $this->render('site/admin-formations.html.twig'); }

    #[Route('/admin-reservations.html', name: 'app_admin_reservations_html', methods: ['GET'])]
    public function reservations(): Response { return $this->render('site/admin-reservations.html.twig'); }

    #[Route('/admin-utilisateurs.html', name: 'app_admin_users_html', methods: ['GET'])]
    public function users(): Response { return $this->render('site/admin-utilisateurs.html.twig'); }

    #[Route('/admin-badges.html', name: 'app_admin_badges_html', methods: ['GET'])]
    public function badges(): Response { return $this->render('site/admin-badges.html.twig'); }
}
