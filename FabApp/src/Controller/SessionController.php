<?php

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use App\Security\SessionRegistry;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * S191a — « Sessions ouvertes » : où ce compte est connecté, et fermer.
 *
 * ⚠️ Une page à part, pas une section du profil : la section « Sécurité » vit
 * dans le formulaire des réglages, et un bouton « Fermer » ne peut pas y
 * être un formulaire imbriqué.
 */
final class SessionController extends AbstractController
{
    #[Route('/profil/sessions', name: 'app_profile_sessions', methods: ['GET'])]
    public function index(Request $request, SessionRegistry $registry): Response
    {
        $user = $this->currentUser();
        if (!$registry->isReady()) {
            return $this->redirectToRoute('app_profile');
        }

        return $this->render('site/profile-sessions.html.twig', [
            'sessions' => $registry->aliveFor($user, $request->getSession()),
        ]);
    }

    #[Route('/profil/sessions/{id}/fermer', name: 'app_profile_session_revoke', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function revoke(int $id, Request $request, SessionRegistry $registry): Response
    {
        $user = $this->currentUser();
        if ($this->isCsrfTokenValid('session_revoke_' . $id, (string) $request->request->get('_token'))) {
            // Fermer SA propre session depuis cette page, c'est se déconnecter :
            // l'écouteur coupe à la requête suivante, c'est-à-dire tout de suite.
            $this->addFlash('success', $registry->revoke($user, $id) ? 'sessions.closed_one' : 'sessions.already_closed');
        }

        return $this->redirectToRoute('app_profile_sessions', [], Response::HTTP_SEE_OTHER);
    }

    #[Route('/profil/sessions/fermer-les-autres', name: 'app_profile_sessions_revoke_others', methods: ['POST'])]
    public function revokeOthers(Request $request, SessionRegistry $registry): Response
    {
        $user = $this->currentUser();
        if ($this->isCsrfTokenValid('sessions_revoke_others', (string) $request->request->get('_token'))) {
            $registry->revokeOthers($user, $request->getSession());
            $this->addFlash('success', 'sessions.closed_others');
        }

        return $this->redirectToRoute('app_profile_sessions', [], Response::HTTP_SEE_OTHER);
    }

    /** Compte volé, ordinateur du labo resté ouvert : l'équipe ferme tout. */
    #[Route('/admin/utilisateurs/{id}/sessions/fermer', name: 'app_admin_user_sessions_revoke', requirements: ['id' => '\d+'], methods: ['POST'])]
    #[IsGranted('ROLE_ADMIN')]
    public function adminRevokeAll(int $id, Request $request, SessionRegistry $registry, UtilisateurRepository $users): Response
    {
        $user = $users->find($id);
        if (!$user instanceof Utilisateur) {
            throw $this->createNotFoundException();
        }
        if ($this->isCsrfTokenValid('admin_sessions_revoke_' . $id, (string) $request->request->get('_token'))) {
            $this->addFlash('success', $registry->revokeAll($user) > 0 ? 'sessions.admin_closed' : 'sessions.admin_none');
        }

        return $this->redirectToRoute('app_admin_user_detail', ['id' => $id], Response::HTTP_SEE_OTHER);
    }

    private function currentUser(): Utilisateur
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException();
        }

        return $user;
    }
}
