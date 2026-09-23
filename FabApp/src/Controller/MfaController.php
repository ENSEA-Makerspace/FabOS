<?php

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use App\Security\MfaGateListener;
use App\Security\MfaService;
use App\Security\Totp;
use App\Service\SiteSettingService;
use Endroid\QrCode\Builder\Builder;
use Endroid\QrCode\ErrorCorrectionLevel;
use Endroid\QrCode\Writer\SvgWriter;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Bundle\SecurityBundle\Security;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S191b — la double authentification : la saisir à la connexion, l'activer,
 * la désactiver, et l'équipe qui la retire (téléphone perdu).
 *
 * ⚠️ Les codes de secours n'existent en clair qu'une fois : ils passent par un
 * flash de session, s'affichent au rechargement suivant, puis disparaissent.
 */
final class MfaController extends AbstractController
{
    private const MAX_FAILURES = 5;
    private const CODES_FLASH = 'mfa_codes';

    #[Route('/connexion/verification', name: 'app_mfa_challenge', methods: ['GET', 'POST'])]
    public function challenge(Request $request, MfaService $mfa, Security $security, TranslatorInterface $translator): Response
    {
        $session = $request->getSession();
        $user = $this->getUser();
        if (!$user instanceof Utilisateur || $session->get(MfaGateListener::PENDING) !== $user->getId()) {
            return $this->redirectToRoute($user instanceof Utilisateur ? 'app_profile' : 'app_login');
        }

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('mfa_challenge', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'forgot.csrf');

                return $this->redirectToRoute('app_mfa_challenge', [], Response::HTTP_SEE_OTHER);
            }
            $via = $mfa->verify($user, $request->request->getString('code'));
            if ($via !== null) {
                $session->remove(MfaGateListener::PENDING);
                $session->remove(MfaGateListener::FAILURES);
                if ($via === 'recovery') {
                    $this->addFlash('success', $translator->trans('mfa.used_recovery', ['count' => $mfa->recoveryLeft($user)]));
                }
                $target = $session->get('_security.main.target_path');
                $session->remove('_security.main.target_path');

                return is_string($target) && $target !== '' ? $this->redirect($target) : $this->redirectToRoute('app_profile');
            }

            $failures = (int) $session->get(MfaGateListener::FAILURES, 0) + 1;
            if ($failures >= self::MAX_FAILURES) {
                // Cinq erreurs : on referme la session. Recommencer exige le mot
                // de passe — et la limite d'essais de S196a s'applique à lui.
                $security->logout(false);
                $request->getSession()->getFlashBag()->add('error', 'mfa.too_many');

                return $this->redirectToRoute('app_login', [], Response::HTTP_SEE_OTHER);
            }
            $session->set(MfaGateListener::FAILURES, $failures);
            $this->addFlash('error', 'mfa.wrong_code');

            return $this->redirectToRoute('app_mfa_challenge', [], Response::HTTP_SEE_OTHER);
        }

        return $this->render('site/login-mfa.html.twig');
    }

    #[Route('/profil/double-authentification', name: 'app_profile_mfa', methods: ['GET'])]
    public function manage(Request $request, MfaService $mfa, SiteSettingService $settings): Response
    {
        $user = $this->currentUser();
        if (!$mfa->isReady()) {
            return $this->redirectToRoute('app_profile');
        }
        $status = $mfa->status($user);
        $secret = $status === MfaService::PENDING ? $mfa->pendingSecret($user) : null;
        $qr = null;
        if ($secret !== null) {
            try {
                $qr = (new Builder(
                    writer: new SvgWriter(),
                    data: Totp::provisioningUri($secret, $settings->getOrgName() ?: 'FabOS', $user->getEmail()),
                    errorCorrectionLevel: ErrorCorrectionLevel::Medium,
                    size: 220,
                    margin: 8,
                ))->build()->getDataUri();
            } catch (\Throwable) {
                $qr = null;
            }
        }
        $codes = $request->getSession()->getFlashBag()->get(self::CODES_FLASH);

        return $this->render('site/profile-mfa.html.twig', [
            'status' => $status,
            'secretGroups' => $secret !== null ? str_split($secret, 4) : [],
            'qr' => $qr,
            'recoveryLeft' => $status === MfaService::ENABLED ? $mfa->recoveryLeft($user) : 0,
            'codes' => $codes[0] ?? [],
        ]);
    }

    #[Route('/profil/double-authentification/{action}', name: 'app_profile_mfa_action', requirements: ['action' => 'activer|confirmer|annuler|regenerer|desactiver'], methods: ['POST'])]
    public function act(string $action, Request $request, MfaService $mfa): Response
    {
        $user = $this->currentUser();
        if (!$mfa->isReady() || !$this->isCsrfTokenValid('mfa_' . $action, (string) $request->request->get('_token'))) {
            return $this->redirectToRoute('app_profile_mfa', [], Response::HTTP_SEE_OTHER);
        }
        $code = $request->request->getString('code');

        switch ($action) {
            case 'activer':
                if ($mfa->status($user) !== MfaService::ENABLED) {
                    $mfa->start($user);
                }
                break;
            case 'annuler':
                if ($mfa->status($user) === MfaService::PENDING) {
                    $mfa->reset($user);
                }
                break;
            case 'confirmer':
                $codes = $mfa->confirm($user, $code);
                if ($codes === null) {
                    $this->addFlash('error', 'mfa.wrong_code');
                } else {
                    $this->addFlash('success', 'mfa.enabled');
                    $request->getSession()->getFlashBag()->add(self::CODES_FLASH, $codes);
                }
                break;
            case 'regenerer':
                $codes = $mfa->regenerate($user, $code);
                if ($codes === null) {
                    $this->addFlash('error', 'mfa.wrong_code');
                } else {
                    $request->getSession()->getFlashBag()->add(self::CODES_FLASH, $codes);
                }
                break;
            case 'desactiver':
                $this->addFlash($mfa->disable($user, $code) ? 'success' : 'error', $mfa->status($user) === MfaService::NONE ? 'mfa.disabled' : 'mfa.wrong_code');
                break;
        }

        return $this->redirectToRoute('app_profile_mfa', [], Response::HTTP_SEE_OTHER);
    }

    /** Téléphone perdu ET codes de secours perdus : l'équipe retire le second facteur. */
    #[Route('/admin/utilisateurs/{id}/double-authentification/retirer', name: 'app_admin_user_mfa_reset', requirements: ['id' => '\d+'], methods: ['POST'])]
    #[IsGranted('ROLE_ADMIN')]
    public function adminReset(int $id, Request $request, MfaService $mfa, UtilisateurRepository $users): Response
    {
        $user = $users->find($id);
        if (!$user instanceof Utilisateur) {
            throw $this->createNotFoundException();
        }
        if ($this->isCsrfTokenValid('admin_mfa_reset_' . $id, (string) $request->request->get('_token'))) {
            $mfa->reset($user);
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
