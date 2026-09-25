<?php

declare(strict_types=1);

namespace App\Controller;

use App\Identity\ExternalIdentityService;
use App\Identity\IdentityRefusal;
use App\Identity\OidcModule;
use App\Identity\ProviderRegistry;
use App\Identity\IdentityTestReport;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Bundle\SecurityBundle\Security;
use Symfony\Component\HttpFoundation\RedirectResponse;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Component\Security\Core\Exception\AccountStatusException;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S196 — se connecter avec un fournisseur OIDC, ou le TESTER.
 *
 * Toute la validation vit dans `OidcModule`, toute la décision d'identité dans
 * `ExternalIdentityService` : ce contrôleur ne fait qu'aiguiller.
 *
 * ⚠️ Un refus ramène à la connexion avec une PHRASE (`identity.refused.*`),
 * jamais une page 500 : un membre doit savoir s'il peut réessayer.
 * « Tester » (`?test=1`, administrateurs seulement) fait le même aller-retour
 * mais ne connecte personne et n'écrit rien : il range ce qu'il a vu pour
 * l'écran « Connexion & annuaires ».
 */
final class OidcController extends AbstractController
{
    #[Route('/login/oidc/{provider}', name: 'app_oidc_start', requirements: ['provider' => '[a-z][a-z0-9_]{1,79}'], methods: ['GET'])]
    public function start(string $provider, Request $request, ProviderRegistry $registry, OidcModule $oidc, IdentityTestReport $reports, TranslatorInterface $translator): RedirectResponse
    {
        $test = $request->query->getBoolean('test');
        if ($test) {
            $this->denyAccessUnlessGranted('ROLE_ADMIN');
        }
        $config = $registry->find($provider);
        if ($config === null || (!$config->enabled && !$test)) {
            throw $this->createNotFoundException();
        }
        if ($test) {
            $oidc->forget($config);
        }

        try {
            return new RedirectResponse($oidc->begin($config, $request->getSession(), $this->callbackUrl(), $test));
        } catch (IdentityRefusal $refusal) {
            if ($test) {
                $reports->store($request->getSession(), $config->key, refusal: $refusal);

                return $this->redirectToRoute('app_admin_identity_test', ['key' => $config->key]);
            }
            $this->addFlash('error', ['identity.login_refused', ['%provider%' => $config->label, '%reason%' => $this->reason($refusal, $translator)]]);

            return $this->redirectToRoute('app_login');
        }
    }

    #[Route('/login/oidc/callback', name: 'app_oidc_callback', methods: ['GET'], priority: 10)]
    public function callback(Request $request, ProviderRegistry $registry, OidcModule $oidc, ExternalIdentityService $identities, Security $security, IdentityTestReport $reports, TranslatorInterface $translator): RedirectResponse
    {
        $session = $request->getSession();
        $state = $request->query->getString('state');
        $testing = $oidc->testFlow($session, $state);
        try {
            $result = $oidc->complete(
                $session,
                $state,
                $request->query->getString('code'),
                $request->query->has('error') ? $request->query->getString('error') : null,
                $this->callbackUrl(),
                $registry->find(...),
            );
        } catch (IdentityRefusal $refusal) {
            // « Tester » montre AUSSI l'échec, détail technique compris : c'est
            // là qu'un réglage faux se voit (mauvaise audience, secret absent…).
            if ($testing !== null && $this->isGranted('ROLE_ADMIN')) {
                $reports->store($session, $testing, refusal: $refusal);

                return $this->redirectToRoute('app_admin_identity_test', ['key' => $testing]);
            }
            $this->addFlash('error', ['identity.login_refused', ['%provider%' => '—', '%reason%' => $this->reason($refusal, $translator)]]);

            return $this->redirectToRoute('app_login');
        }

        if ($result->test) {
            $reports->store($session, $result->provider->key, result: $result, decision: $identities->decide($result->profile));

            return $this->redirectToRoute('app_admin_identity_test', ['key' => $result->provider->key]);
        }

        try {
            $user = $identities->apply($result->profile);
            $security->login($user, 'form_login', 'main');
        } catch (IdentityRefusal $refusal) {
            $this->addFlash('error', ['identity.login_refused', ['%provider%' => $result->provider->label, '%reason%' => $this->reason($refusal, $translator)]]);

            return $this->redirectToRoute('app_login');
        } catch (AccountStatusException $e) {
            // Le statut local gagne : même message que la connexion par mot de passe.
            $this->addFlash('error', $e->getMessageKey());

            return $this->redirectToRoute('app_login');
        }

        return $this->redirectToRoute('app_profile');
    }

    private function callbackUrl(): string
    {
        return $this->generateUrl('app_oidc_callback', [], UrlGeneratorInterface::ABSOLUTE_URL);
    }

    private function reason(IdentityRefusal $refusal, TranslatorInterface $translator): string
    {
        return $translator->trans($refusal->reasonKey, $refusal->params);
    }
}
