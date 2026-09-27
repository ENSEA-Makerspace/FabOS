<?php

declare(strict_types=1);

namespace App\Controller;

use App\Form\Admin\AuthProviderType;
use App\Identity\AttributeMapping;
use App\Identity\AuthProvider;
use App\Identity\IdentityTestReport;
use App\Identity\ProviderRegistry;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\Form\FormError;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Component\Security\Http\Attribute\IsGranted;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S196 — « Connexion & annuaires » : les fournisseurs de connexion, là où un
 * exploitant les cherche (Configuration), et non plus dans l'écran de la
 * fédération FabOS (`/admin/network`), qui n'a rien à voir.
 *
 * 🔴 **« Tester » est ce qui rend une « myriade de solutions » configurable
 * sans lire de code** : un vrai aller-retour chez le fournisseur, puis les
 * attributs reçus et ce que FabOS en ferait — sans connecter personne ni rien
 * écrire. Il marche sur un fournisseur pas encore activé.
 */
#[IsGranted('ROLE_ADMIN')]
#[Route('/admin/connexion')]
final class IdentityAdminController extends AbstractController
{
    #[Route('', name: 'app_admin_identity', methods: ['GET'])]
    public function index(ProviderRegistry $registry): Response
    {
        return $this->render('site/admin-identity.html.twig', [
            'providers' => $registry->all(),
            'linked' => $registry->linkedCounts(),
            'ready' => $registry->isExtended(),
        ]);
    }

    #[Route('/nouveau', name: 'app_admin_identity_new', methods: ['GET', 'POST'])]
    #[Route('/{key}', name: 'app_admin_identity_edit', requirements: ['key' => '[a-z][a-z0-9_]{1,79}'], methods: ['GET', 'POST'])]
    public function edit(Request $request, ProviderRegistry $registry, TranslatorInterface $translator, ?string $key = null): Response
    {
        $provider = $key !== null ? $registry->find($key) : null;
        if ($key !== null && $provider === null) {
            throw $this->createNotFoundException();
        }
        $data = $provider !== null ? self::toForm($provider) : ['scopes' => 'openid profile email', 'preset' => AttributeMapping::DEFAULT_PRESET, 'enabled' => false];
        $form = $this->createForm(AuthProviderType::class, $data, [
            'editing' => $provider !== null,
            'preset' => (string) ($request->request->all('auth_provider')['preset'] ?? $data['preset']),
        ]);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $values = $form->getData();
            $saveKey = $provider?->key ?? (string) $values['key'];
            if ($provider === null && $registry->find($saveKey) !== null) {
                $form->get('key')->addError(new FormError($translator->trans('identity.invalid.key_taken')));
            } else {
                $mapping = [];
                foreach (AttributeMapping::FIELDS as $field) {
                    $value = trim((string) ($values['map_' . $field] ?? ''));
                    if ($value !== '') {
                        $mapping[$field] = $value;
                    }
                }
                try {
                    $registry->save(
                        $saveKey, (string) $values['label'], AuthProvider::KIND_OIDC, (string) $values['issuer'],
                        (string) $values['clientId'], (string) $values['secretEnv'], (string) $values['scopes'], (bool) $values['enabled'],
                        ['preset' => (string) $values['preset'], 'trustEmail' => (bool) $values['trustEmail'], 'passwordUrl' => trim((string) ($values['passwordUrl'] ?? '')), 'mapping' => $mapping],
                    );
                    $this->addFlash('success', ['identity.saved', ['%label%' => (string) $values['label']]]);

                    return $this->redirectToRoute('app_admin_identity', [], Response::HTTP_SEE_OTHER);
                } catch (\InvalidArgumentException $e) {
                    $form->addError(new FormError($translator->trans($e->getMessage())));
                } catch (\Doctrine\DBAL\Exception\UniqueConstraintViolationException) {
                    $form->get('issuer')->addError(new FormError($translator->trans('identity.invalid.issuer_taken')));
                }
            }
        }

        return $this->render('site/admin-identity-form.html.twig', [
            'provider' => $provider,
            'form' => $form->createView(),
            'callbackUrl' => $this->generateUrl('app_oidc_callback', [], UrlGeneratorInterface::ABSOLUTE_URL),
            'ready' => $registry->isExtended(),
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/{key}/etat', name: 'app_admin_identity_toggle', requirements: ['key' => '[a-z][a-z0-9_]{1,79}'], methods: ['POST'])]
    public function toggle(string $key, Request $request, ProviderRegistry $registry): Response
    {
        $provider = $registry->find($key) ?? throw $this->createNotFoundException();
        if ($this->isCsrfTokenValid('identity_toggle_' . $key, (string) $request->request->get('_token'))) {
            $registry->setEnabled($key, !$provider->enabled);
            $this->addFlash('success', [$provider->enabled ? 'identity.disabled_flash' : 'identity.enabled_flash', ['%label%' => $provider->label]]);
        }

        return $this->redirectToRoute('app_admin_identity', [], Response::HTTP_SEE_OTHER);
    }

    #[Route('/{key}/test', name: 'app_admin_identity_test', requirements: ['key' => '[a-z][a-z0-9_]{1,79}'], methods: ['GET'])]
    public function test(string $key, Request $request, ProviderRegistry $registry, IdentityTestReport $reports): Response
    {
        $provider = $registry->find($key) ?? throw $this->createNotFoundException();

        return $this->render('site/admin-identity-test.html.twig', [
            'provider' => $provider,
            'report' => $reports->get($request->getSession(), $key),
            'mapping' => $provider->mapping(),
        ]);
    }

    /** @return array<string, mixed> */
    private static function toForm(AuthProvider $provider): array
    {
        $data = [
            'label' => $provider->label, 'key' => $provider->key, 'issuer' => $provider->issuer,
            'clientId' => $provider->clientId, 'secretEnv' => $provider->secretEnv, 'scopes' => implode(' ', $provider->scopes),
            'preset' => $provider->preset(), 'trustEmail' => $provider->trustsEmail(), 'enabled' => $provider->enabled,
            'passwordUrl' => (string) $provider->passwordUrl(),
        ];
        $explicit = \is_array($provider->settings['mapping'] ?? null) ? $provider->settings['mapping'] : [];
        foreach (AttributeMapping::FIELDS as $field) {
            $data['map_' . $field] = (string) ($explicit[$field] ?? '');
        }

        return $data;
    }
}
