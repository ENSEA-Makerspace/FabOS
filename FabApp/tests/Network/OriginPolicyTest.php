<?php
namespace App\Tests\Network;
use App\Network\OriginPolicy;
use PHPUnit\Framework\Attributes\DataProvider;
use PHPUnit\Framework\TestCase;
final class OriginPolicyTest extends TestCase
{
    public function testCanonicalHttpsOriginIsAccepted(): void { self::assertSame('https://fab.example.org:8443',(new OriginPolicy())->normalize(' https://FAB.EXAMPLE.ORG:8443 ')); }
    #[DataProvider('invalidOrigins')]
    public function testUnsafeOriginsAreRejected(string $origin): void { $this->expectException(\InvalidArgumentException::class); (new OriginPolicy())->normalize($origin); }
    public static function invalidOrigins(): iterable { yield ['http://fab.example.org']; yield ['https://localhost']; yield ['https://127.0.0.1']; yield ['https://fab.example.org/path']; yield ['https://user@fab.example.org']; }
}
