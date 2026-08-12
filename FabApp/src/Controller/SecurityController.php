<?php

namespace App\Controller;

use App\Identity\ProviderRegistry;
use App\Mail\Mailer;
use App\Mail\NotificationCategory;
use App\Repository\UtilisateurRepository;
use App\Security\PasswordResetTokenizer;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Authentication\AuthenticationUtils;

final class SecurityController extends AbstractController
{
    #[Route('/login', name: 'app_login', methods: ['GET', 'POST'])]
    #[Route('/login.html', name: 'app_login_html', methods: ['GET'])]
    public function login(AuthenticationUtils $authenticationUtils, ProviderRegistry $providers): Response
    {
        if ($this->getUser()) {
            return $this->redirectToRoute('app_profile');
        }

        return $this->render('site/login.html.twig', [
            'last_username' => $authenticationUtils->getLastUsername(),
            'error' => $authenticationUtils->getLastAuthenticationError(),
            'oidcProviders' => $providers->enabled(),
        ]);
    }


    /**
     * ⚠️ **The answer is the same whether the address exists or not.** Telling a
     * stranger "no account with that e-mail" turns this form into a membership
     * oracle: point it at a list of addresses and it reports which of them belong
     * to this lab. That is a real disclosure for a makerspace whose members did
     * not agree to be listed. One sentence, one response time, one behaviour.
     */
    #[Route('/forgot-password', name: 'app_forgot_password_submit', methods: ['POST'])]
    public function forgotPasswordSubmit(
        Request $request,
        UtilisateurRepository $users,
        PasswordResetTokenizer $tokenizer,
        Mailer $mailer,
    ): Response {
        if (!$this->isCsrfTokenValid('forgot_password', (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'forgot.csrf');

            return $this->redirectToRoute('app_forgot_password');
        }

        $email = trim(mb_strtolower($request->request->getString('email')));
        $user = $email === '' ? null : $users->findOneBy(['email' => $email]);

        if ($user !== null) {
            $token = $tokenizer->create($user, new \DateTimeImmutable());
            // ⚠️ `transactional: true`. A password reset is not a notification and
            // must not be suppressed by a member's e-mail preferences — locking
            // someone out of their account because they opted out of announcements
            // would be an own goal.
            $mailer->queueToUser($user, 'password_reset', [
                'resetUrl' => $this->generateUrl(
                    'app_reset_password',
                    ['token' => $token],
                    \Symfony\Component\Routing\Generator\UrlGeneratorInterface::ABSOLUTE_URL,
                ),
                'validHours' => (int) (PasswordResetTokenizer::TTL_SECONDS / 3600),
            ], NotificationCategory::GENERAL, true);
        }

        $this->addFlash('success', 'forgot.sent_if_exists');

        return $this->redirectToRoute('app_forgot_password');
    }

    /**
     * ⚠️ The token is verified TWICE: once to decide whether to show the form,
     * and again on submit before anything is written. Without the second check a
     * form left open past the expiry — or after the password was changed by some
     * other route — would still write.
     */
    #[Route('/reset-password/{token}', name: 'app_reset_password', methods: ['GET', 'POST'], requirements: ['token' => '[A-Za-z0-9_\-\.]+'])]
    public function resetPassword(
        string $token,
        Request $request,
        UtilisateurRepository $users,
        PasswordResetTokenizer $tokenizer,
        UserPasswordHasherInterface $hasher,
        EntityManagerInterface $entityManager,
    ): Response {
        $now = new \DateTimeImmutable();
        $userId = $tokenizer->userIdIfValid($token, $now);
        $user = $userId === null ? null : $users->find($userId);

        if ($user === null || !$tokenizer->matchesAccount($token, $user)) {
            return $this->render('site/reset-password.html.twig', ['expired' => true, 'token' => $token], new Response(status: Response::HTTP_GONE));
        }

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('reset_password', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'forgot.csrf');

                return $this->redirectToRoute('app_reset_password', ['token' => $token]);
            }

            $password = $request->request->getString('password');
            $confirm = $request->request->getString('password_confirm');

            if (mb_strlen($password) < 8) {
                $this->addFlash('error', 'reset.too_short');
            } elseif ($password !== $confirm) {
                $this->addFlash('error', 'reset.mismatch');
            } else {
                $user->setPassword($hasher->hashPassword($user, $password));
                $entityManager->flush();

                // The token committed to the OLD hash, so it — and every other
                // outstanding link for this account — is now dead. Nothing to
                // delete, nothing that can be forgotten.
                $this->addFlash('success', 'reset.done');

                return $this->redirectToRoute('app_login');
            }
        }

        return $this->render('site/reset-password.html.twig', ['expired' => false, 'token' => $token]);
    }

    #[Route('/logout', name: 'app_logout', methods: ['GET'])]
    public function logout(): never
    {
        throw new \LogicException('Logout is handled by Symfony Security.');
    }
}
