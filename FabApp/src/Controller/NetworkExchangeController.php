<?php
namespace App\Controller;
use App\Entity\Utilisateur;
use App\Network\ExchangeService;
use App\Network\InstanceIdentity;
use App\Repository\UtilisateurBadgeRepository;
use Endroid\QrCode\Builder\Builder;
use Endroid\QrCode\ErrorCorrectionLevel;
use Endroid\QrCode\Writer\PngWriter;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;
final class NetworkExchangeController extends AbstractController
{
    #[Route('/profil/reseau/qr',name:'app_profile_network_qr',methods:['POST'])]
    #[IsGranted('ROLE_USER')]
    public function qr(Request $request,ExchangeService $exchange,InstanceIdentity $identity,UtilisateurBadgeRepository $badges): Response
    {
        if(!$this->isCsrfTokenValid('network_exchange',$request->request->getString('_token')) || !$request->request->getBoolean('consent')) throw $this->createAccessDeniedException('Consentement requis.');
        $user=$this->getUser(); if(!$user instanceof Utilisateur || $user->getId()===null) throw $this->createAccessDeniedException(); $audience=$request->request->getString('audienceUuid'); if(!preg_match('/^[0-9a-f-]{36}$/i',$audience)) throw new \InvalidArgumentException('Instance destinataire invalide.');
        $fields=$user->getPublicFields(); $claims=[]; if(in_array('name',$fields,true))$claims['name']=$user->getDisplayName(); if(in_array('bio',$fields,true))$claims['bio']=$user->getPublicBio(); if(in_array('badges',$fields,true))$claims['credentials']=array_values(array_filter(array_map(static fn($award)=>$award->getBadge()?->isArchived()?null:['kind'=>'badge','sourceObjectUuid'=>sprintf('00000000-0000-4000-8000-%012d',$award->getBadge()?->getId()??0),'label'=>$award->getBadge()?->getNom(),'issuedAt'=>$award->getDateObtention()->format(DATE_ATOM)],$badges->findBy(['utilisateur'=>$user]))));
        $token=$exchange->issue($user->getId(),$audience,$claims,new \DateTimeImmutable('+10 minutes')); $instance=$identity->get(); $payload=json_encode(['version'=>'v1','source'=>$instance['origin'],'sourceUuid'=>$instance['uuid'],'token'=>$token],JSON_THROW_ON_ERROR); $png=(new Builder(writer:new PngWriter(),data:$payload,errorCorrectionLevel:ErrorCorrectionLevel::High,size:800,margin:20))->build()->getString();
        return new Response($png,200,['Content-Type'=>'image/png','Content-Disposition'=>'attachment; filename="fabos-exchange.png"','Referrer-Policy'=>'no-referrer','Cache-Control'=>'no-store']);
    }
}
