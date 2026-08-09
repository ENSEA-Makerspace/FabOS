<?php
namespace App\Tests\Identity;
use App\Entity\Utilisateur;
use PHPUnit\Framework\TestCase;
final class PublicProfilePrivacyTest extends TestCase
{
    public function testProfileIsPrivateAndFieldsAreAllowListed(): void
    {
        $user=new Utilisateur(); self::assertFalse($user->isPublicProfileEnabled()); self::assertSame([], $user->getPublicFields());
        $user->setPublicFields(['name','email','rfid','badges','name']); self::assertSame(['name','badges'],$user->getPublicFields());
    }
}
