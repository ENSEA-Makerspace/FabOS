<?php
namespace App\Controller;
use App\Network\ExchangeService;
use App\Network\InstanceIdentity;
use App\Network\SignedRequestVerifier;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\JsonResponse;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\Routing\Attribute\Route;
final class NetworkApiController extends AbstractController
{
    #[Route('/api/fabos/v1/exchanges/{token}',name:'app_fabos_exchange_consume',requirements:['token'=>'[A-Za-z0-9_-]{43}'],methods:['POST'])]
    public function consume(string $token,Request $request,SignedRequestVerifier $signed,ExchangeService $exchanges,InstanceIdentity $identity): JsonResponse
    {
        try{$peer=$signed->verify($request);$claims=$exchanges->consume($token,$peer['instanceUuid']);if($claims===null)return $this->json(['error'=>'expired_or_consumed'],410);$payload=json_encode(['version'=>'v1','source'=>$identity->get()['uuid'],'claims'=>$claims,'issuedAt'=>(new \DateTimeImmutable())->format(DATE_ATOM)],JSON_THROW_ON_ERROR);return $this->json(['payload'=>base64_encode($payload),'signature'=>$identity->sign($payload),'keyId'=>$identity->get()['keyId']]);}catch(\RuntimeException $e){return $this->json(['error'=>$e->getMessage()],403);}
    }
}
