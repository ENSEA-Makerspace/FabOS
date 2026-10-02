<?php

declare(strict_types=1);

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Page\ProfileOverview;
use App\Repository\UtilisateurRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * PROPOSITION (2026-10-02) — « Mon compte », une refonte de `/profil`, en sous-page
 * du menu Développement. `?membre=<id>` regarde la page avec un compte de test,
 * `?onglet=` choisit l'onglet. Lecture seule.
 * 🅿️ À la décision : retirer ce contrôleur, son entrée de menu, le gabarit
 * `site/proposals/profil.html.twig` et `page-profil-proposition.css`.
 */
#[IsGranted('ROLE_ADMIN')]
final class ProfileProposalController extends AbstractController
{
    #[Route('/admin/propositions/profil', name: 'app_admin_proposal_profile', methods: ['GET'])]
    public function show(Request $request, UtilisateurRepository $users, ProfileOverview $overview): Response
    {
        $member = $this->getUser();
        if ($request->query->getInt('membre') > 0) {
            $member = $users->find($request->query->getInt('membre')) ?? $member;
        }
        if (!$member instanceof Utilisateur) {
            throw $this->createAccessDeniedException();
        }
        $tab = (string) $request->query->get('onglet', 'apercu');

        return $this->render('site/proposals/profil.html.twig', [
            'member' => $member,
            'tab' => \in_array($tab, ['apercu', 'acces', 'activite', 'reglages'], true) ? $tab : 'apercu',
            'p' => $overview->for($member),
            'demo' => $request->query->getInt('membre') ?: null,
        ]);
    }
}
