<?php
namespace App\Network;
use Doctrine\DBAL\Connection;
final class ExchangeService
{
    public function __construct(private readonly Connection $db) {}
    public function issue(int $userId,string $audienceUuid,array $claims,\DateTimeImmutable $expires): string
    {
        if ($expires <= new \DateTimeImmutable() || $expires > new \DateTimeImmutable('+15 minutes')) throw new \InvalidArgumentException('Expiration invalide.');
        $token=rtrim(strtr(base64_encode(random_bytes(32)),'+/','-_'),'=');
        $this->db->insert('FABOS_EXCHANGE',['tokenHash'=>hash('sha256',$token),'userId'=>$userId,'audienceUuid'=>$audienceUuid,'claimsJson'=>json_encode($claims,JSON_THROW_ON_ERROR),'expiresAt'=>$expires->format('Y-m-d H:i:s'),'createdAt'=>(new \DateTimeImmutable())->format('Y-m-d H:i:s')]); return $token;
    }
    public function consume(string $token,string $audienceUuid): ?array
    {
        return $this->db->transactional(function() use($token,$audienceUuid): ?array { $row=$this->db->fetchAssociative('SELECT * FROM FABOS_EXCHANGE WHERE tokenHash=:hash FOR UPDATE',['hash'=>hash('sha256',$token)]); if(!$row || $row['audienceUuid']!==$audienceUuid || $row['consumedAt']!==null || $row['revokedAt']!==null || new \DateTimeImmutable($row['expiresAt'])<=new \DateTimeImmutable()) return null; $this->db->update('FABOS_EXCHANGE',['consumedAt'=>(new \DateTimeImmutable())->format('Y-m-d H:i:s')],['id'=>$row['id']]); return json_decode($row['claimsJson'],true,512,JSON_THROW_ON_ERROR); });
    }
}
