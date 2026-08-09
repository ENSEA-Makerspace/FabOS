<?php
namespace App\Network;
use Doctrine\DBAL\Connection;
final class FederatedCatalogImporter
{
    public function __construct(private readonly Connection $db) {}
    public function credential(int $userId,string $source,string $uuid,string $kind,string $label,array $payload,\DateTimeImmutable $issuedAt,?\DateTimeImmutable $expiresAt,?\DateTimeImmutable $revokedAt): void
    { if(!in_array($kind,['badge','training'],true)) throw new \InvalidArgumentException('Credential inconnu.'); $this->db->executeStatement('INSERT INTO FEDERATED_CREDENTIAL (userId,sourceInstanceUuid,sourceObjectUuid,kind,label,payloadJson,issuedAt,expiresAt,revokedAt,importedAt) VALUES (:user,:source,:uuid,:kind,:label,:payload,:issued,:expires,:revoked,NOW()) ON DUPLICATE KEY UPDATE label=VALUES(label),payloadJson=VALUES(payloadJson),expiresAt=VALUES(expiresAt),revokedAt=VALUES(revokedAt),importedAt=NOW()',['user'=>$userId,'source'=>$source,'uuid'=>$uuid,'kind'=>$kind,'label'=>$label,'payload'=>json_encode($payload,JSON_THROW_ON_ERROR),'issued'=>$issuedAt->format('Y-m-d H:i:s'),'expires'=>$expiresAt?->format('Y-m-d H:i:s'),'revoked'=>$revokedAt?->format('Y-m-d H:i:s')]); }
    public function machine(string $source,string $uuid,string $brand,string $model,array $payload,?\DateTimeImmutable $archivedAt): void
    { $forbidden=['rfid','token','status','safety','availability','venue','stock']; foreach($forbidden as $key) unset($payload[$key]); $this->db->executeStatement('INSERT INTO FEDERATED_MACHINE_CATALOG (sourceInstanceUuid,sourceObjectUuid,brand,model,payloadJson,archivedAt,importedAt) VALUES (:source,:uuid,:brand,:model,:payload,:archived,NOW()) ON DUPLICATE KEY UPDATE brand=VALUES(brand),model=VALUES(model),payloadJson=VALUES(payloadJson),archivedAt=VALUES(archivedAt),importedAt=NOW()',['source'=>$source,'uuid'=>$uuid,'brand'=>mb_substr(trim($brand),0,140),'model'=>mb_substr(trim($model),0,180),'payload'=>json_encode($payload,JSON_THROW_ON_ERROR),'archived'=>$archivedAt?->format('Y-m-d H:i:s')]); }
}
