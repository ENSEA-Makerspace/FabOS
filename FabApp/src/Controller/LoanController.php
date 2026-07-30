<?php

namespace App\Controller;

use App\Repository\LoanableItemRepository;
use App\Repository\LoanRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * Public Loans page — browse what the lab lends out and how many of each are
 * currently available. Gated by the admin-toggleable `loans` module (app_loans
 * route prefix → FeatureAccessSubscriber). Checkouts themselves are managed by
 * staff in the admin; members track their own from their profile.
 */
final class LoanController extends AbstractController
{
    #[Route('/prets', name: 'app_loans', methods: ['GET'])]
    public function catalogue(LoanableItemRepository $items, LoanRepository $loans): Response
    {
        return $this->render('site/loans.html.twig', [
            'items' => $items->findAllSafe(),
            'activeCounts' => $loans->activeCountsByItem(),
        ]);
    }
}
