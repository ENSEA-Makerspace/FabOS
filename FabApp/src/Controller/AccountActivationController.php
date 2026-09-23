<?php

namespace App\Controller;

use App\Security\AccountActivation;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\SecurityRequestAttributes;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S189 — après l'inscription : « vérifiez votre boîte », renvoyer, corriger,
 * activer. Aucune impasse : chaque état a une sortie.
 *
 * 🔴 **Cet écran ne sait pas s'il parle à un nouveau compte.** Il ne reçoit que
 * l'adresse tapée et le délai de renvoi ; le compte en attente, s'il existe,
 * reste côté service. C'est ce qui rend la page identique pour une adresse libre
 * et une adresse déjà membre — le gabarit ne PEUT pas trahir la différence.
 */
final class AccountActivationController extends AbstractController
{
    /** Assez pour deux fautes de frappe et un changement d'avis. */
    private const MAX_CORRECTIONS = 5;

    #[Route('/inscription/verifier', name: 'app_register_check', methods: ['GET'])]
    public function check(Request $request, AccountActivation $activation): Response
    {
        $session = $request->getSession();
        $email = $session->get(AccountActivation::SESSION_EMAIL);
        if (!is_string($email) || $email === '') {
            return $this->redirectToRoute('app_register');
        }

        return $this->render('site/register-check.html.twig', [
            'email' => $email,
            'cooldown' => $activation->cooldownLeft($session),
        ]);
    }

    #[Route('/inscription/renvoyer', name: 'app_register_resend', methods: ['POST'])]
    public function resend(Request $request, AccountActivation $activation, TranslatorInterface $translator): Response
    {
        $session = $request->getSession();
        if (!$this->isCsrfTokenValid('register_resend', (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'forgot.csrf');
        } elseif (($wait = $activation->cooldownLeft($session)) > 0) {
            $this->addFlash('error', $translator->trans('register_check.wait', ['%seconds%' => $wait]));
        } else {
            $activation->resend($session);
            $this->addFlash('success', 'register_check.resent');
        }

        return $this->redirectToRoute('app_register_check', [], Response::HTTP_SEE_OTHER);
    }

    #[Route('/inscription/corriger', name: 'app_register_correct', methods: ['POST'])]
    public function correct(Request $request, AccountActivation $activation): Response
    {
        $session = $request->getSession();
        $email = mb_strtolower(trim($request->request->getString('email')));
        $count = (int) $session->get('activation.corrections', 0);

        if (!$this->isCsrfTokenValid('register_correct', (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'forgot.csrf');
        } elseif ($email === '' || filter_var($email, FILTER_VALIDATE_EMAIL) === false) {
            $this->addFlash('error', 'register.err_email');
        } elseif ($count >= self::MAX_CORRECTIONS) {
            $this->addFlash('error', 'register_check.too_many');
        } else {
            $session->set('activation.corrections', $count + 1);
            $activation->correct($session, $email);
            $this->addFlash('success', 'register_check.corrected');
        }

        return $this->redirectToRoute('app_register_check', [], Response::HTTP_SEE_OTHER);
    }

    /**
     * ⚠️ Un GET qui écrit, assumé : c'est un lien dans un courrier. Un scanner de
     * messagerie qui le suit active le compte — il a lu la boîte, ce qui est
     * précisément ce que le lien prouve.
     */
    #[Route('/inscription/activer/{token}', name: 'app_register_activate', methods: ['GET'], requirements: ['token' => '[A-Za-z0-9_\-\.]+'])]
    public function activate(string $token, Request $request, AccountActivation $activation): Response
    {
        $user = $activation->activate($token);
        if ($user === null) {
            // Faux, expiré, déjà servi, ou parti vers une adresse corrigée depuis :
            // la page dit quoi faire dans CHAQUE cas, sans dire lequel.
            return $this->render('site/register-activate-failed.html.twig', [], new Response(status: Response::HTTP_GONE));
        }

        $session = $request->getSession();
        foreach ([AccountActivation::SESSION_EMAIL, AccountActivation::SESSION_USER, AccountActivation::SESSION_SENT_AT, 'activation.corrections'] as $key) {
            $session->remove($key);
        }
        // L'adresse est pré-remplie à la connexion : il ne reste que le mot de passe.
        $session->set(SecurityRequestAttributes::LAST_USERNAME, $user->getEmail());
        $this->addFlash('success', 'register_check.activated');

        return $this->redirectToRoute('app_login', [], Response::HTTP_SEE_OTHER);
    }
}
