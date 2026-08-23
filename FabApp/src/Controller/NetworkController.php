<?php
namespace App\Controller;
use App\Form\Admin\NetworkIdentityType;
use App\Form\Admin\NetworkPeerType;
use App\Form\Admin\NetworkProviderType;
use App\Identity\ProviderRegistry;
use App\Network\InstanceIdentity;
use App\Network\OriginPolicy;
use Doctrine\DBAL\Connection;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\Form\FormError;
use Symfony\Component\Form\FormInterface;
use Symfony\Component\HttpFoundation\JsonResponse;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

final class NetworkController extends AbstractController
{
    #[Route('/.well-known/fabos', name:'app_fabos_discovery', methods:['GET'])]
    public function discovery(InstanceIdentity $identity): JsonResponse { try { $i=$identity->get(); } catch(\LogicException) { return $this->json(['status'=>'unconfigured'],503); } return $this->json(['api'=>'fabos-network','version'=>'v1','instance'=>$i]); }

    /**
     * ⚠️ **S148, J-22 — les trois formulaires de l'écran passent au thème.**
     *
     * 🔴 **Ce que la conversion change, c'est le refus.** Les trois branches
     * étaient enveloppées dans un `try { … } catch(\Throwable $e)` qui posait
     * `$e->getMessage()` en flash puis **redirigeait**. Une identité de pair
     * invalide — un UUID mal collé, une clé publique tronquée — renvoyait donc
     * l'opérateur sur une page vierge : les quatre champs, dont une clé base64,
     * étaient à recoller. Chaque formulaire se re-rend maintenant avec ce qui a
     * été tapé, et l'erreur est sur le champ.
     *
     * ⚠️ Le `catch` reste, pour ce qu'aucune contrainte ne peut prévoir (une
     * origine que `OriginPolicy` rejette, une écriture qui échoue). Il pose alors
     * l'erreur sur le formulaire concerné plutôt qu'en haut de page.
     */
    #[Route('/admin/network', name:'app_admin_network', methods:['GET','POST'])]
    #[IsGranted('ROLE_ADMIN')]
    public function admin(Request $request,ProviderRegistry $providers,InstanceIdentity $identity,Connection $db,OriginPolicy $origins): Response
    {
        $forms = [
            'identity' => $this->createForm(NetworkIdentityType::class),
            'provider' => $this->createForm(NetworkProviderType::class, ['enabled' => true]),
            'peer' => $this->createForm(NetworkPeerType::class),
        ];
        foreach ($forms as $form) {
            $form->handleRequest($request);
        }

        $submitted = null;
        foreach ($forms as $key => $form) {
            if ($form->isSubmitted()) {
                $submitted = $key;
                break;
            }
        }

        if ($submitted !== null && $forms[$submitted]->isValid()) {
            $data = $forms[$submitted]->getData();
            try {
                match ($submitted) {
                    'identity' => $identity->initialize((string) $data['name'], $origins->normalize((string) $data['origin'])),
                    'provider' => $providers->save((string) $data['key'], (string) $data['label'], (string) $data['issuer'], (string) $data['clientId'], (string) $data['secretEnv'], (bool) $data['enabled']),
                    'peer' => $this->trustPeer($db, $origins, $data),
                };
                $this->addFlash('success', 'flash.configuration_reseau_enregistree');

                return $this->redirectToRoute('app_admin_network');
            } catch (\Throwable $e) {
                $forms[$submitted]->addError(new FormError($e->getMessage()));
            }
        }

        try{$instance=$identity->get();}catch(\LogicException){$instance=null;}

        return $this->render('site/admin-network.html.twig',[
            'instance'=>$instance,
            'providers'=>$db->fetchAllAssociative('SELECT providerKey,label,issuer,enabled FROM AUTH_PROVIDER ORDER BY label'),
            'peers'=>$db->fetchAllAssociative('SELECT p.*,i.nom institution FROM FABOS_PEER p LEFT JOIN INSTITUTION i ON i.id=p.institutionId ORDER BY p.createdAt DESC'),
            'workspaceKey'=>'network',
            'forms'=>array_map(static fn (FormInterface $form) => $form->createView(), $forms),
            // ⚠️ Le pair vit dans un `<details>` replié : refusé, il doit s'ouvrir,
            // sinon l'opérateur ne voit ni ce qu'il a tapé ni pourquoi c'est refusé.
            'peerRefused'=>$submitted === 'peer',
        ], $submitted !== null ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    /** @param array<string, mixed> $data */
    private function trustPeer(Connection $db, OriginPolicy $origins, array $data): void
    {
        $db->executeStatement(
            "INSERT INTO FABOS_PEER (instanceUuid,origin,keyId,publicKey,trustState,createdAt) VALUES (:uuid,:origin,:key,:public,'trusted',NOW()) ON DUPLICATE KEY UPDATE origin=VALUES(origin),keyId=VALUES(keyId),publicKey=VALUES(publicKey),trustState='trusted',suspendedAt=NULL",
            [
                'uuid' => (string) $data['instanceUuid'],
                'origin' => $origins->normalize((string) $data['peerOrigin']),
                'key' => (string) $data['keyId'],
                'public' => (string) $data['publicKey'],
            ],
        );
    }
}
