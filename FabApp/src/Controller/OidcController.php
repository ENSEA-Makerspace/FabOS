<?php
namespace App\Controller;
use App\Identity\ExternalIdentityService;
use App\Identity\ProviderRegistry;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Bundle\SecurityBundle\Security;
use Symfony\Component\HttpFoundation\RedirectResponse;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Contracts\HttpClient\HttpClientInterface;

/** OIDC authorization-code + PKCE. External claims never create local roles. */
final class OidcController extends AbstractController
{
    #[Route('/login/oidc/{provider}',name:'app_oidc_start',requirements:['provider'=>'[a-z][a-z0-9_]{1,79}'],methods:['GET'])]
    public function start(string $provider,Request $request,ProviderRegistry $registry,HttpClientInterface $http): RedirectResponse
    {
        $config=$registry->find($provider); if(!$config?->enabled) throw $this->createNotFoundException();
        $discovery=$http->request('GET',$config->issuer.'/.well-known/openid-configuration',['timeout'=>5,'max_redirects'=>0])->toArray();
        if(($discovery['issuer']??null)!==$config->issuer) throw new \RuntimeException('Issuer OIDC incohérent.');
        foreach(['authorization_endpoint','token_endpoint','userinfo_endpoint'] as $key) if(!isset($discovery[$key]) || !str_starts_with($discovery[$key],'https://')) throw new \RuntimeException('Découverte OIDC incomplète.');
        $state=bin2hex(random_bytes(24)); $nonce=bin2hex(random_bytes(24)); $verifier=rtrim(strtr(base64_encode(random_bytes(48)),'+/','-_'),'=');
        $request->getSession()->set('oidc_'.$state,['provider'=>$provider,'nonce'=>$nonce,'verifier'=>$verifier,'discovery'=>$discovery,'created'=>time()]);
        $query=http_build_query(['response_type'=>'code','client_id'=>$config->clientId,'redirect_uri'=>$this->generateUrl('app_oidc_callback',[],0),'scope'=>implode(' ',$config->scopes),'state'=>$state,'nonce'=>$nonce,'code_challenge'=>rtrim(strtr(base64_encode(hash('sha256',$verifier,true)),'+/','-_'),'='),'code_challenge_method'=>'S256']);
        return new RedirectResponse($discovery['authorization_endpoint'].'?'.$query);
    }
    #[Route('/login/oidc/callback',name:'app_oidc_callback',methods:['GET'],priority:10)]
    public function callback(Request $request,ProviderRegistry $registry,ExternalIdentityService $identities,HttpClientInterface $http,Security $security): RedirectResponse
    {
        $state=$request->query->getString('state'); $flow=$request->getSession()->remove('oidc_'.$state); if(!is_array($flow) || time()-(int)$flow['created']>600) throw $this->createAccessDeniedException('État OIDC invalide ou expiré.');
        $config=$registry->find($flow['provider']); if(!$config?->enabled) throw $this->createAccessDeniedException(); $secret=getenv($config->secretEnv); if(!is_string($secret)||$secret==='') throw new \RuntimeException('Secret OIDC indisponible.');
        $tokens=$http->request('POST',$flow['discovery']['token_endpoint'],['body'=>['grant_type'=>'authorization_code','code'=>$request->query->getString('code'),'redirect_uri'=>$this->generateUrl('app_oidc_callback',[],0),'client_id'=>$config->clientId,'client_secret'=>$secret,'code_verifier'=>$flow['verifier']],'timeout'=>8,'max_redirects'=>0])->toArray();
        $claims=$http->request('GET',$flow['discovery']['userinfo_endpoint'],['auth_bearer'=>$tokens['access_token']??'','timeout'=>8,'max_redirects'=>0])->toArray();
        $user=$identities->resolveOrProvision($config,(string)($claims['sub']??''),isset($claims['email'])?(string)$claims['email']:null,isset($claims['name'])?(string)$claims['name']:null);
        $security->login($user,'form_login','main'); return $this->redirectToRoute('app_profile');
    }
}
