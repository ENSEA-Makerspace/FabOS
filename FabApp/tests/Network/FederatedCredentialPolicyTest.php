<?php
namespace App\Tests\Network;
use App\Network\FederatedCredentialPolicy;
use PHPUnit\Framework\TestCase;
final class FederatedCredentialPolicyTest extends TestCase
{
    public function testRevocationAndExpiryCanNeverReactivateCredential(): void
    {
        $policy=new FederatedCredentialPolicy(); $now=new \DateTimeImmutable('2026-08-10 12:00:00');
        self::assertTrue($policy->isActive(['issuedAt'=>'2026-08-01','expiresAt'=>'2026-09-01','revokedAt'=>null],$now));
        self::assertFalse($policy->isActive(['issuedAt'=>'2026-08-01','expiresAt'=>'2026-08-09','revokedAt'=>null],$now));
        self::assertFalse($policy->isActive(['issuedAt'=>'2026-08-01','expiresAt'=>'2026-09-01','revokedAt'=>'2026-08-08'],$now));
    }
}
