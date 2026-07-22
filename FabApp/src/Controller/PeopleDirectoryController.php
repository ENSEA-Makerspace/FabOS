<?php

namespace App\Controller;

use App\Repository\UtilisateurRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * Public directory pages listing the lab's staff and trainers. Membership is a
 * person-type flag on the user (isStaff / isTrainer), set by an admin. Both
 * pages are gated by their own admin-toggleable module (app_staff / app_trainer
 * route prefixes → ModuleAccessSubscriber). Display-only for now — the "book a
 * person" flow rides the later polymorphic-Reservation refactor.
 */
final class PeopleDirectoryController extends AbstractController
{
    #[Route('/equipe', name: 'app_staff', methods: ['GET'])]
    public function staff(UtilisateurRepository $users): Response
    {
        return $this->render('site/people-directory.html.twig', [
            'people' => $users->findStaff(),
            'personType' => 'staff',
        ]);
    }

    #[Route('/formateurs', name: 'app_trainers', methods: ['GET'])]
    public function trainers(UtilisateurRepository $users): Response
    {
        return $this->render('site/people-directory.html.twig', [
            'people' => $users->findTrainers(),
            'personType' => 'trainers',
        ]);
    }
}
