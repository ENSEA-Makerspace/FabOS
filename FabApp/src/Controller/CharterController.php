<?php

declare(strict_types=1);

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Service\CharterAcceptances;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * S209 — la charte de sécurité : le texte (le règlement du lab) et « J'ai lu et
 * j'accepte ». Une fois acceptée, la page dit quand. Si le texte change, la
 * version change et l'accord est redemandé.
 *
 * ⚠️ 404 tant que la fonction `charter` est éteinte, que la migration manque ou
 * qu'aucun règlement n'est écrit. Elle ne bloque PAS la réservation.
 */
#[IsGranted('ROLE_USER')]
final class CharterController extends AbstractController
{
    #[Route('/charte', name: 'app_charter', methods: ['GET', 'POST'])]
    public function index(Request $request, CharterAcceptances $charter): Response
    {
        if (!$charter->isAvailable()) {
            throw $this->createNotFoundException();
        }
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException();
        }

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('charter_accept', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');

                return $this->redirectToRoute('app_charter', [], Response::HTTP_SEE_OTHER);
            }
            $charter->accept((int) $user->getId());
            $this->addFlash('success', 'charter.accepted_flash');

            // Accepté : on rentre à l'accueil, dont la zone de messages dit merci.
            return $this->redirectToRoute('app_home', [], Response::HTTP_SEE_OTHER);
        }

        return $this->render('site/charter.html.twig', [
            'charterHtml' => $charter->html(),
            'acceptedAt' => $charter->acceptedAt((int) $user->getId()),
        ]);
    }
}
