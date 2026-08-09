<?php
namespace App\Controller;
use App\Identity\ProviderRegistry;
use App\Network\InstanceIdentity;
use App\Network\OriginPolicy;
use Doctrine\DBAL\Connection;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\JsonResponse;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

final class NetworkController extends AbstractController
{
    #[Route('/.well-known/fabos', name:'app_fabos_discovery', methods:['GET'])]
    public function discovery(InstanceIdentity $identity): JsonResponse { try { $i=$identity->get(); } catch(\LogicException) { return $this->json(['status'=>'unconfigured'],503); } return $this->json(['api'=>'fabos-network','version'=>'v1','instance'=>$i]); }

    #[Route('/admin/network', name:'app_admin_network', methods:['GET','POST'])]
    #[IsGranted('ROLE_ADMIN')]
    public function admin(Request $request,ProviderRegistry $providers,InstanceIdentity $identity,Connection $db,OriginPolicy $origins): Response
    {
        if($request->isMethod('POST')) {
            if(!$this->isCsrfTokenValid('network_settings',$request->request->getString('_token'))) throw $this->createAccessDeniedException();
            $action=$request->request->getString('action');
            try {
                if($action==='initialize') $identity->initialize($request->request->getString('name'),$origins->normalize($request->request->getString('origin')));
                if($action==='provider') $providers->save($request->request->getString('key'),$request->request->getString('label'),$request->request->getString('issuer'),$request->request->getString('clientId'),$request->request->getString('secretEnv'),$request->request->getBoolean('enabled'));
                if($action==='peer') {
                    $origin=$origins->normalize($request->request->getString('peerOrigin')); $public=base64_decode($request->request->getString('publicKey'),true);
                    if(!preg_match('/^[0-9a-f-]{36}$/i',$request->request->getString('instanceUuid')) || $public===false || strlen($public)!==SODIUM_CRYPTO_SIGN_PUBLICKEYBYTES) throw new \InvalidArgumentException('Identité du pair invalide.');
                    $db->executeStatement("INSERT INTO FABOS_PEER (instanceUuid,origin,keyId,publicKey,trustState,createdAt) VALUES (:uuid,:origin,:key,:public,'trusted',NOW()) ON DUPLICATE KEY UPDATE origin=VALUES(origin),keyId=VALUES(keyId),publicKey=VALUES(publicKey),trustState='trusted',suspendedAt=NULL",['uuid'=>$request->request->getString('instanceUuid'),'origin'=>$origin,'key'=>$request->request->getString('keyId'),'public'=>$request->request->getString('publicKey')]);
                }
                $this->addFlash('success','Configuration réseau enregistrée.');
            } catch(\Throwable $e) { $this->addFlash('error',$e->getMessage()); }
            return $this->redirectToRoute('app_admin_network');
        }
        try{$instance=$identity->get();}catch(\LogicException){$instance=null;}
        return $this->render('site/admin-network.html.twig',['instance'=>$instance,'providers'=>$db->fetchAllAssociative('SELECT providerKey,label,issuer,enabled FROM AUTH_PROVIDER ORDER BY label'),'peers'=>$db->fetchAllAssociative('SELECT p.*,i.nom institution FROM FABOS_PEER p LEFT JOIN INSTITUTION i ON i.id=p.institutionId ORDER BY p.createdAt DESC'),'workspaceKey'=>'network']);
    }
}
