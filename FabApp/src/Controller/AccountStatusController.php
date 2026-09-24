<?php

namespace App\Controller;

use App\Account\AccountAnonymiser;
use App\Account\AccountDeactivation;
use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S190d — « Désactiver ce compte » / « Réactiver », depuis la fiche admin.
 *
 * ⚠️ Le refus est rejoué ici : la fiche l'affiche, mais c'est ce POST qui
 * protège (soi-même, dernier administrateur, compte anonymisé).
 */
#[IsGranted('ROLE_ADMIN')]
final class AccountStatusController extends AbstractController
{
    #[Route('/admin/utilisateurs/{id}/desactiver', name: 'app_admin_user_deactivate', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function deactivate(int $id, Request $request, UtilisateurRepository $users, AccountDeactivation $deactivation, TranslatorInterface $translator): Response
    {
        $user = $this->find($users, $id);
        if (!$this->isCsrfTokenValid('admin_deactivate_' . $id, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');
        } elseif (AccountDeactivation::isInactive($user)) {
            $this->addFlash('success', 'account_status.already_inactive');
        } elseif (($refusal = $deactivation->refusalFor($user, $this->actor())) !== null) {
            $this->addFlash('error', 'account_status.refused_' . $refusal);
        } else {
            $cancelled = $deactivation->deactivate($user);
            $this->addFlash('success', $translator->trans('account_status.deactivated', ['count' => $cancelled]));
        }

        return $this->redirectToRoute('app_admin_user_detail', ['id' => $id], Response::HTTP_SEE_OTHER);
    }

    #[Route('/admin/utilisateurs/{id}/reactiver', name: 'app_admin_user_reactivate', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function reactivate(int $id, Request $request, UtilisateurRepository $users, AccountDeactivation $deactivation): Response
    {
        $user = $this->find($users, $id);
        if (!$this->isCsrfTokenValid('admin_reactivate_' . $id, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');
        } elseif (AccountAnonymiser::isAnonymised($user)) {
            $this->addFlash('error', 'account_status.refused_already_anonymised');
        } else {
            $deactivation->reactivate($user);
            $this->addFlash('success', 'account_status.reactivated');
        }

        return $this->redirectToRoute('app_admin_user_detail', ['id' => $id], Response::HTTP_SEE_OTHER);
    }

    private function actor(): ?Utilisateur
    {
        $actor = $this->getUser();

        return $actor instanceof Utilisateur ? $actor : null;
    }

    private function find(UtilisateurRepository $users, int $id): Utilisateur
    {
        $user = $users->find($id);
        if (!$user instanceof Utilisateur) {
            throw $this->createNotFoundException();
        }

        return $user;
    }
}
