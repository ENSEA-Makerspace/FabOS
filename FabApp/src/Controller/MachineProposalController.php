<?php

declare(strict_types=1);

namespace App\Controller;

use App\Page\MachineCreationHints;
use App\Repository\BadgeRepository;
use App\Repository\VenueRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * PROPOSITION (2026-10-06) — « Créer une machine » revu, en sous-page du menu
 * Développement. Une maquette : elle lit les vraies données et n'enregistre rien.
 * 🅿️ À la décision : retirer ce contrôleur, son entrée de menu (`NavBuilder`, clé
 * `admin_nav.entry.app_admin_proposal_machine`) et `site/proposals/machine-new.html.twig`.
 * `MachineCreationHints` reste s'il sert le vrai écran.
 */
#[IsGranted('ROLE_ADMIN')]
final class MachineProposalController extends AbstractController
{
    #[Route('/admin/propositions/machine', name: 'app_admin_proposal_machine', methods: ['GET'])]
    public function show(MachineCreationHints $hints, BadgeRepository $badges, VenueRepository $venues): Response
    {
        return $this->render('site/proposals/machine-new.html.twig', [
            'hints' => $hints->all(),
            'venues' => $venues->findBy(['active' => true], ['name' => 'ASC']),
            'badges' => $badges->findBy([], ['nom' => 'ASC']),
        ]);
    }
}
