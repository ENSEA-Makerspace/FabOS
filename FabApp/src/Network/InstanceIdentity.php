<?php
namespace App\Network;
use Doctrine\DBAL\Connection;
use Symfony\Component\DependencyInjection\Attribute\Autowire;
final class InstanceIdentity
{
    public function __construct(private readonly Connection $db, #[Autowire('%kernel.secret%')] private readonly string $secret) {}
    /** @return array{uuid:string,name:string,origin:string,keyId:string,publicKey:string} */
    public function get(): array
    {
        $row=$this->db->fetchAssociative('SELECT * FROM FABOS_INSTANCE WHERE id=1'); if (!$row) { throw new \LogicException('Identité FabOS non initialisée.'); }
        return ['uuid'=>$row['instanceUuid'],'name'=>$row['displayName'],'origin'=>$row['canonicalOrigin'],'keyId'=>$row['activeKeyId'],'publicKey'=>$row['publicKey']];
    }
    public function initialize(string $name,string $origin): void
    {
        if ($this->db->fetchOne('SELECT 1 FROM FABOS_INSTANCE WHERE id=1')) { return; }
        $pair=sodium_crypto_sign_keypair(); $public=base64_encode(sodium_crypto_sign_publickey($pair)); $private=sodium_crypto_sign_secretkey($pair); $nonce=random_bytes(SODIUM_CRYPTO_SECRETBOX_NONCEBYTES); $key=hash('sha256',$this->secret,true); $encrypted=base64_encode($nonce.sodium_crypto_secretbox($private,$nonce,$key));
        $this->db->insert('FABOS_INSTANCE',['id'=>1,'instanceUuid'=>$this->uuid(),'displayName'=>trim($name),'canonicalOrigin'=>(new OriginPolicy())->normalize($origin),'activeKeyId'=>'key-'.bin2hex(random_bytes(6)),'publicKey'=>$public,'privateKeyEncrypted'=>$encrypted,'createdAt'=>(new \DateTimeImmutable())->format('Y-m-d H:i:s')]);
    }
    public function sign(string $payload): string { $row=$this->db->fetchAssociative('SELECT privateKeyEncrypted FROM FABOS_INSTANCE WHERE id=1'); if(!$row) throw new \LogicException('Identité absente.'); $raw=base64_decode($row['privateKeyEncrypted'],true); $nonce=substr($raw,0,SODIUM_CRYPTO_SECRETBOX_NONCEBYTES); $private=sodium_crypto_secretbox_open(substr($raw,SODIUM_CRYPTO_SECRETBOX_NONCEBYTES),$nonce,hash('sha256',$this->secret,true)); if($private===false) throw new \RuntimeException('Clé illisible.'); return base64_encode(sodium_crypto_sign_detached($payload,$private)); }
    private function uuid(): string { $h=bin2hex(random_bytes(16)); return sprintf('%s-%s-4%s-%s%s-%s',substr($h,0,8),substr($h,8,4),substr($h,13,3),dechex((hexdec($h[16])&3)|8),substr($h,17,3),substr($h,20)); }
}
