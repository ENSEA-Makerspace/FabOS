<?php

namespace App\Controller;

use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

final class SiteController extends AbstractController
{
    #[Route('/', name: 'app_home', methods: ['GET'])]
    #[Route('/index.html', name: 'app_home_html', methods: ['GET'])]
    public function home(): Response
    {
        return $this->render('site/index.html.twig');
    }

    #[Route('/calendrier', name: 'app_calendar', methods: ['GET'])]
    #[Route('/calendrier.html', name: 'app_calendar_html', methods: ['GET'])]
    #[Route('/calendar', name: 'app_calendar_legacy_en', methods: ['GET'])]
    #[Route('/calendar.html', name: 'app_calendar_legacy_en_html', methods: ['GET'])]
    public function calendar(): Response
    {
        return $this->render('site/calendrier.html.twig');
    }

    #[Route('/machines', name: 'app_machines', methods: ['GET'])]
    #[Route('/machines.html', name: 'app_machines_html', methods: ['GET'])]
    #[Route('/machine', name: 'app_machine_legacy_singular', methods: ['GET'])]
    #[Route('/machine.html', name: 'app_machine_legacy_singular_html', methods: ['GET'])]
    public function machines(): Response
    {
        return $this->render('site/machines.html.twig');
    }


    #[Route('/machine/new', name: 'app_machine_new_legacy', methods: ['GET'])]
    #[Route('/machines/new', name: 'app_machines_new_legacy', methods: ['GET'])]
    public function machineNewLegacy(): Response
    {
        return $this->render('site/admin-machines.html.twig');
    }

    #[Route('/machines/{id}', name: 'app_machine_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}', name: 'app_machine_detail_legacy_singular', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine-detail.html', name: 'app_machine_detail_html', methods: ['GET'])]
    public function machineDetail(Request $request, ?int $id = null): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        return $this->render('site/machine-detail.html.twig', ['machine_id' => $id]);
    }

    #[Route('/machines/{id}/calendrier', name: 'app_machine_calendar', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machines/{id}/calendar', name: 'app_machine_calendar_legacy_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/calendrier', name: 'app_machine_calendar_legacy_singular', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/calendar', name: 'app_machine_calendar_legacy_singular_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine-calendrier.html', name: 'app_machine_calendar_html', methods: ['GET'])]
    public function machineCalendar(Request $request, ?int $id = null): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        return $this->render('site/machine-calendrier.html.twig', ['machine_id' => $id]);
    }

    #[Route('/machines/{id}/historique', name: 'app_machine_history', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machines/{id}/history', name: 'app_machine_history_legacy_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/historique', name: 'app_machine_history_legacy_singular', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/history', name: 'app_machine_history_legacy_singular_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine-historique.html', name: 'app_machine_history_html', methods: ['GET'])]
    public function machineHistory(Request $request, ?int $id = null): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        return $this->render('site/machine-historique.html.twig', ['machine_id' => $id]);
    }

    #[Route('/formations', name: 'app_formations', methods: ['GET'])]
    #[Route('/formations.html', name: 'app_formations_html', methods: ['GET'])]
    public function formations(): Response
    {
        return $this->render('site/formations.html.twig');
    }

    #[Route('/formations/{id}', name: 'app_formation_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/formation-detail.html', name: 'app_formation_detail_html', methods: ['GET'])]
    public function formationDetail(Request $request, ?int $id = null): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        return $this->render('site/formation-detail.html.twig', ['formation_id' => $id]);
    }

    #[Route('/formations/{id}/suivi', name: 'app_formation_follow', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/formation-suivi.html', name: 'app_formation_follow_html', methods: ['GET'])]
    public function formationFollow(Request $request, ?int $id = null): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        return $this->render('site/formation-suivi.html.twig', ['formation_id' => $id]);
    }

    #[Route('/leaderboard', name: 'app_leaderboard', methods: ['GET'])]
    #[Route('/leaderboard.html', name: 'app_leaderboard_html', methods: ['GET'])]
    public function leaderboard(): Response
    {
        return $this->render('site/leaderboard.html.twig');
    }

    #[Route('/login', name: 'app_login', methods: ['GET'])]
    #[Route('/login.html', name: 'app_login_html', methods: ['GET'])]
    public function login(): Response
    {
        return $this->render('site/login.html.twig');
    }

    #[Route('/register', name: 'app_register', methods: ['GET'])]
    #[Route('/register.html', name: 'app_register_html', methods: ['GET'])]
    public function register(): Response
    {
        return $this->render('site/register.html.twig');
    }

    #[Route('/forgot-password', name: 'app_forgot_password', methods: ['GET'])]
    #[Route('/forgot-password.html', name: 'app_forgot_password_html', methods: ['GET'])]
    public function forgotPassword(): Response
    {
        return $this->render('site/forgot-password.html.twig');
    }

    #[Route('/profil', name: 'app_profile', methods: ['GET'])]
    #[Route('/profil.html', name: 'app_profile_html', methods: ['GET'])]
    public function profile(): Response
    {
        return $this->render('site/profil.html.twig');
    }

    #[Route('/search', name: 'app_search', methods: ['GET'])]
    #[Route('/search.html', name: 'app_search_html', methods: ['GET'])]
    public function search(): Response
    {
        return $this->render('site/search.html.twig');
    }

    #[Route('/recherche', name: 'app_recherche', methods: ['GET'])]
    #[Route('/recherche.html', name: 'app_recherche_html', methods: ['GET'])]
    public function recherche(): Response
    {
        return $this->render('site/recherche.html.twig');
    }
}
