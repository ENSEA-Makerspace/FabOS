<?php

namespace App\Controller;

use App\Entity\Badge;
use App\Entity\Utilisateur;
use App\Repository\BadgeRepository;
use App\Repository\UtilisateurRepository;
use App\Training\BadgeGrants;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * S202 — « Attribuer un badge » / « Retirer un badge », depuis la fiche admin
 * d'une personne. Le motif est OBLIGATOIRE dans les deux sens : c'est lui qui
 * répond plus tard à « pourquoi a-t-elle (ou n'a-t-elle plus) ce badge ? ».
 */
#[IsGranted('ROLE_ADMIN')]
final class BadgeGrantController extends AbstractController
{
    #[Route('/admin/utilisateurs/{id}/badges/attribuer', name: 'app_admin_user_badge_grant', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function grant(int $id, Request $request, UtilisateurRepository $users, BadgeRepository $badges, BadgeGrants $grants): Response
    {
        return $this->act($id, $request, $users, $badges, 'admin_badge_grant_', function (Utilisateur $user, Badge $badge, Utilisateur $actor, string $reason) use ($grants): string {
            return $grants->grant($user, $badge, $actor, $reason) ? 'badge_grants.granted' : 'badge_grants.already_held';
        });
    }

    #[Route('/admin/utilisateurs/{id}/badges/retirer', name: 'app_admin_user_badge_revoke', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function revoke(int $id, Request $request, UtilisateurRepository $users, BadgeRepository $badges, BadgeGrants $grants): Response
    {
        return $this->act($id, $request, $users, $badges, 'admin_badge_revoke_', function (Utilisateur $user, Badge $badge, Utilisateur $actor, string $reason) use ($grants): string {
            return $grants->revoke($user, $badge, $actor, $reason) ? 'badge_grants.revoked' : 'badge_grants.not_held';
        });
    }

    /** @param callable(Utilisateur, Badge, Utilisateur, string): string $do rend la clé du message */
    private function act(int $id, Request $request, UtilisateurRepository $users, BadgeRepository $badges, string $tokenPrefix, callable $do): Response
    {
        $user = $users->find($id);
        if (!$user instanceof Utilisateur) {
            throw $this->createNotFoundException();
        }
        $actor = $this->getUser();
        $badge = $badges->find((int) $request->request->get('badge'));
        $reason = trim((string) $request->request->get('reason'));

        if (!$this->isCsrfTokenValid($tokenPrefix . $id, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');
        } elseif (!$badge instanceof Badge || !$actor instanceof Utilisateur) {
            $this->addFlash('error', 'badge_grants.pick_badge');
        } elseif ($reason === '') {
            $this->addFlash('error', 'badge_grants.reason_required');
        } else {
            $this->addFlash('success', [$do($user, $badge, $actor, mb_substr($reason, 0, 500)), ['%badge%' => $badge->getNom()]]);
        }

        return $this->redirectToRoute('app_admin_user_detail', ['id' => $id, '_fragment' => 'badge-grants'], Response::HTTP_SEE_OTHER);
    }
}
