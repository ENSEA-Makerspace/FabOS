<?php

declare(strict_types=1);

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Identity\ExternalIdentityService;
use App\Identity\IdentityDecision;
use App\Identity\IdentityRefusal;
use App\Identity\PendingExternalLogin;
use App\Repository\UtilisateurRepository;
use App\Security\AccountActivation;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Bundle\SecurityBundle\Security;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * S197 — la première connexion par un fournisseur, sans impasse.
 *
 * « Complétez votre compte » ne demande QUE ce qui manque (`needs`) :
 *   - une ADRESSE — absente, non garantie par le fournisseur, ou déjà prise
 *     ici. 🔴 Celle qu'on tape suit exactement S189 : lien de confirmation si
 *     elle est libre, « vous avez déjà un compte » à sa propriétaire si elle
 *     est prise — et le MÊME écran dans les deux cas. Taper l'adresse d'un
 *     autre ne révèle rien et ne lie rien ;
 *   - un NOM, si le fournisseur n'en donne aucun.
 *
 * « J'ai déjà un compte FabOS » : on se connecte à ce compte (mot de passe,
 * second facteur compris), PUIS on confirme la liaison sur une page qui montre
 * l'identité externe en jeu. ⚠️ La confirmation explicite après connexion est ce
 * qui protège un poste partagé : quelqu'un qui se connecterait ensuite dans le
 * même navigateur verrait le nom d'un autre, et annulerait.
 */
final class ExternalAccountController extends AbstractController
{
    #[Route('/connexion/completer', name: 'app_external_complete', methods: ['GET', 'POST'])]
    public function complete(Request $request, PendingExternalLogin $pending, ExternalIdentityService $identities, AccountActivation $activation, UtilisateurRepository $users, Security $security): Response
    {
        $session = $request->getSession();
        $state = $pending->get($session);
        $profile = $pending->profile($session);
        if ($state === null || $profile === null) {
            $this->addFlash('error', 'identity.pending_expired');

            return $this->redirectToRoute('app_login');
        }

        $decision = $identities->decide($profile);
        if ($decision->outcome !== IdentityDecision::COMPLETE) {
            // Lié entre-temps, ou plus rien ne manque : la connexion normale reprend.
            $pending->clear($session);
            $this->addFlash('success', 'identity.pending_resolved');

            return $this->redirectToRoute('app_login');
        }
        $needsEmail = \in_array(IdentityDecision::NEED_EMAIL, $decision->needs, true);
        $needsName = \in_array(IdentityDecision::NEED_NAME, $decision->needs, true);

        $errors = [];
        $values = [
            // Une adresse que le fournisseur n'a pas garantie est PROPOSÉE (on la
            // confirmera) ; une adresse déjà prise ici ne l'est pas.
            'email' => $decision->emailTaken ? '' : (string) $profile->email,
            'firstName' => (string) $decision->firstName,
            'lastName' => (string) $decision->lastName,
        ];

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('external_complete', (string) $request->request->get('_token'))) {
                $errors[] = 'forgot.csrf';
            }
            if ($needsEmail) {
                $values['email'] = mb_strtolower(trim($request->request->getString('email')));
                if (!filter_var($values['email'], FILTER_VALIDATE_EMAIL)) {
                    $errors[] = 'identity.complete.email_invalid';
                }
            }
            if ($needsName) {
                $values['firstName'] = trim($request->request->getString('firstName'));
                $values['lastName'] = trim($request->request->getString('lastName'));
                if ($values['firstName'] === '' && $values['lastName'] === '') {
                    $errors[] = 'identity.complete.name_required';
                }
            }

            if ($errors === []) {
                $first = $values['firstName'] !== '' ? mb_substr($values['firstName'], 0, 100) : null;
                $last = $values['lastName'] !== '' ? mb_substr($values['lastName'], 0, 100) : null;
                try {
                    if (!$needsEmail) {
                        // Seul le nom manquait : l'adresse est garantie et libre.
                        $user = $identities->provision($profile, (string) $profile->email, $first, $last, verified: true);
                        $pending->clear($session);
                        $security->login($user, 'form_login', 'main');

                        return $this->redirectToRoute('app_profile');
                    }

                    $owner = $users->findOneBy(['email' => $values['email']]);
                    if ($activation->isRequired()) {
                        // 🔴 S189 à l'identique : un courrier part dans les deux cas,
                        // l'écran est le même.
                        if ($owner instanceof Utilisateur) {
                            $activation->sendAlreadyRegistered($owner);
                            $activation->remember($session, $values['email'], null);
                        } else {
                            $user = $identities->provision($profile, $values['email'], $first, $last, verified: false);
                            $activation->sendLink($user);
                            $activation->remember($session, $values['email'], $user);
                        }
                        $pending->clear($session);

                        return $this->redirectToRoute('app_register_check');
                    }

                    // Sans courrier opérationnel, S189 ouvre le compte tout de
                    // suite ; on fait pareil — mais une adresse prise est refusée.
                    if ($owner instanceof Utilisateur) {
                        $errors[] = 'identity.complete.email_taken_nomail';
                    } else {
                        $user = $identities->provision($profile, $values['email'], $first, $last, verified: true);
                        $pending->clear($session);
                        $security->login($user, 'form_login', 'main');

                        return $this->redirectToRoute('app_profile');
                    }
                } catch (IdentityRefusal $refusal) {
                    $errors[] = $refusal->reasonKey;
                }
            }
        }

        return $this->render('site/external-complete.html.twig', [
            'provider' => $state['providerLabel'],
            'needsEmail' => $needsEmail,
            'needsName' => $needsName,
            'emailTaken' => $decision->emailTaken,
            'emailUnverified' => $profile->email !== null && !$profile->emailVerified,
            'values' => $values,
            'errors' => $errors,
        ], $errors !== [] ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    /** « J'ai déjà un compte FabOS » : se connecter d'abord, lier ensuite. */
    #[Route('/connexion/completer/lier', name: 'app_external_link_start', methods: ['POST'])]
    public function startLink(Request $request, PendingExternalLogin $pending): Response
    {
        $session = $request->getSession();
        $state = $pending->get($session);
        if ($state === null || !$this->isCsrfTokenValid('external_complete', (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'identity.pending_expired');

            return $this->redirectToRoute('app_login');
        }
        $pending->markWantsLink($session);
        $this->addFlash('success', ['identity.link_login', ['%provider%' => $state['providerLabel']]]);

        return $this->redirectToRoute('app_login');
    }

    /**
     * Connecté à son compte local : confirmer (ou non) la liaison. La page
     * montre l'identité externe — nom et adresse — pour qu'on sache QUI on lie.
     */
    #[Route('/connexion/lier', name: 'app_external_link_confirm', methods: ['GET', 'POST'])]
    public function confirmLink(Request $request, PendingExternalLogin $pending, ExternalIdentityService $identities): Response
    {
        $user = $this->getUser();
        $session = $request->getSession();
        $state = $pending->get($session);
        $profile = $pending->profile($session);
        if (!$user instanceof Utilisateur || $state === null || $profile === null || !$state['wantsLink']) {
            $pending->clear($session);

            return $this->redirectToRoute('app_profile');
        }

        if ($request->isMethod('POST') && $this->isCsrfTokenValid('external_link', (string) $request->request->get('_token'))) {
            $pending->clear($session);
            if ($request->request->getString('answer') !== 'link') {
                $this->addFlash('success', 'identity.link_cancelled');

                return $this->redirectToRoute('app_profile');
            }
            try {
                $identities->linkExisting($profile, $user);
                $this->addFlash('success', ['identity.linked', ['%provider%' => $state['providerLabel']]]);
            } catch (IdentityRefusal $refusal) {
                $this->addFlash('error', $refusal->reasonKey);
            }

            return $this->redirectToRoute('app_profile');
        }

        return $this->render('site/external-link.html.twig', [
            'provider' => $state['providerLabel'],
            'external' => $profile,
        ]);
    }
}
