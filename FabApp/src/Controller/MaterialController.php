<?php

namespace App\Controller;

use App\Repository\MaterialRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * Public Materials catalogue — browse the shared catalogue of significant lab
 * materials. Gated by the admin-toggleable `materials` module (app_materials
 * route prefix → FeatureAccessSubscriber). The per-machine / per-training
 * "materials used" lists land in a follow-up slice.
 */
final class MaterialController extends AbstractController
{
    #[Route('/materiaux', name: 'app_materials', methods: ['GET'])]
    public function catalogue(MaterialRepository $materials): Response
    {
        return $this->render('site/materials.html.twig', [
            'materials' => $materials->findAllSafe(),
        ]);
    }
}
