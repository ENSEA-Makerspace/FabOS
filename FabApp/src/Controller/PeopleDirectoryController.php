<?php

namespace App\Controller;

use App\Repository\UtilisateurRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * Public directory pages listing the lab's staff and trainers.
 *
 * These modules are **surfaces and nothing else**: each owns one page and one
 * menu entry. Three separate things share the word "staff" and must not be
 * confused —
 *
 *  - the **people and their roles**, which are kernel: `ROLE_STAFF` /
 *    `ROLE_TRAINER` authorisation and the staff desk (pass issuing, ticket
 *    scanning) keep working with both these modules off, and nothing here may
 *    ever become a way to revoke someone's authorisation;
 *  - the **directory page**, which is what this controller and these two
 *    modules are; and
 *  - whether someone's **time is bookable**, which is the `person_booking`
 *    resource module plus the person's own `bookable` flag.
 *
 * An events venue may well want an "our team" page with nobody bookable, and a
 * workshop may book its trainers' time while publishing no directory at all.
 * The route gate matches `app_staff` and `app_trainers` **exactly** for this
 * reason — see ModuleAccessSubscriber.
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
