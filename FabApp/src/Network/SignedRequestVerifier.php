<?php
namespace App\Network;
use Doctrine\DBAL\Connection;
use Symfony\Component\HttpFoundation\Request;
final class SignedRequestVerifier
{
    public function __construct(private readonly Connection $db) {}
    /** @return array<string,mixed> trusted peer row */
    public function verify(Request $request): array
    {
        $uuid=$request->headers->get('X-FabOS-Instance',''); $timestamp=$request->headers->get('X-FabOS-Timestamp',''); $nonce=$request->headers->get('X-FabOS-Nonce',''); $signature=base64_decode($request->headers->get('X-FabOS-Signature',''),true);
        if(abs(time()-(int)$timestamp)>300 || strlen($nonce)<24 || $signature===false) throw new \RuntimeException('Signature réseau invalide ou expirée.');
        $peer=$this->db->fetchAssociative("SELECT * FROM FABOS_PEER WHERE instanceUuid=:uuid AND trustState='trusted' AND suspendedAt IS NULL",['uuid'=>$uuid]); if(!$peer) throw new \RuntimeException('Pair non approuvé.');
        $canonical=$request->getMethod()."\n".$request->getPathInfo()."\n".$timestamp."\n".$nonce."\n".hash('sha256',$request->getContent()); $public=base64_decode($peer['publicKey'],true);
        if($public===false || !sodium_crypto_sign_verify_detached($signature,$canonical,$public)) throw new \RuntimeException('Signature réseau incorrecte.');
        try{$this->db->insert('FABOS_REPLAY',['nonceHash'=>hash('sha256',$uuid."\0".$nonce),'peerId'=>$peer['id'],'expiresAt'=>(new \DateTimeImmutable('+10 minutes'))->format('Y-m-d H:i:s'),'consumedAt'=>(new \DateTimeImmutable())->format('Y-m-d H:i:s')]);}catch(\Throwable){throw new \RuntimeException('Rejeu réseau refusé.');}
        return $peer;
    }
}
