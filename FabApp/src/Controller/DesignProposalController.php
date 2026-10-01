<?php

declare(strict_types=1);

namespace App\Controller;

use App\Catalogue\PlaceCatalogue;
use App\Design\PageProposals;
use App\Entity\Utilisateur;
use App\Service\MarkdownDocService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * Les pages revues d'après les planches, en sous-pages du menu Développement
 * (demande de l'opérateur, 2026-10-01). Voir {@see PageProposals}.
 *
 * ⚠️ Comme `/admin/references` : non traduit (outillage de développement), aucun
 * formulaire qui enregistre. Une proposition se regarde, elle ne fait rien.
 */
#[Route('/admin/propositions')]
#[IsGranted('ROLE_ADMIN')]
final class DesignProposalController extends AbstractController
{
    #[Route('', name: 'app_admin_proposals', methods: ['GET'])]
    public function index(PageProposals $proposals, MarkdownDocService $docs): Response
    {
        return $this->render('site/admin-proposals.html.twig', [
            'proposals' => $proposals->all(),
            'triage' => $docs->render('proposals'),
        ]);
    }

    #[Route('/{slug}', name: 'app_admin_proposal', requirements: ['slug' => '[a-z0-9-]+'], methods: ['GET'])]
    public function show(string $slug, Request $request, PageProposals $proposals, PlaceCatalogue $places): Response
    {
        $proposal = $proposals->find($slug) ?? throw $this->createNotFoundException();
        $user = $this->getUser();
        $user = $user instanceof Utilisateur ? $user : null;

        // Chaque proposition lit les MÊMES données que la page qu'elle remplacerait.
        $data = match ($slug) {
            'espaces-catalogue' => $places->build($request, $user),
            default => [],
        };

        return $this->render('site/proposals/' . $slug . '.html.twig', $data + [
            'proposal' => $proposal,
            'all_proposals' => $proposals->all(),
        ]);
    }
}
