<?php
namespace App\Network;
final class FederatedCredentialPolicy
{
    /** Imported proof can be displayed, but qualifies only while issued, unexpired and unrevoked. */
    public function isActive(array $row,?\DateTimeImmutable $now=null): bool
    {
        $now??=new \DateTimeImmutable(); if(($row['revokedAt']??null)!==null) return false;
        if(isset($row['issuedAt']) && new \DateTimeImmutable((string)$row['issuedAt'])>$now) return false;
        return !isset($row['expiresAt']) || $row['expiresAt']===null || new \DateTimeImmutable((string)$row['expiresAt'])>$now;
    }
}
