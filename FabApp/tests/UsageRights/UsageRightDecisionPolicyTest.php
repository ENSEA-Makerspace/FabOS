<?php

namespace App\Tests\UsageRights;

use App\UsageRights\UsageRightDecisionPolicy;
use PHPUnit\Framework\Attributes\DataProvider;
use PHPUnit\Framework\TestCase;

final class UsageRightDecisionPolicyTest extends TestCase
{
    /** @return iterable<string, array{bool,bool,bool,bool,bool,list<string>,bool,string}> */
    public static function decisions(): iterable
    {
        yield 'unsupported stays closed' => [false, false, true, true, false, [], false, 'unsupported'];
        yield 'disabled enforcement preserves existing behaviour' => [true, false, true, false, false, [], true, 'not_enforced'];
        yield 'site feature off cannot be overridden' => [true, true, false, true, true, ['Full'], false, 'feature_disabled'];
        yield 'anonymous user is invited to sign in' => [true, true, true, false, false, [], false, 'signin_required'];
        yield 'subject admin has recovery access' => [true, true, true, true, true, [], true, 'admin_bypass'];
        yield 'member without package is refused' => [true, true, true, true, false, [], false, 'missing_package'];
        yield 'one package grants the capability' => [true, true, true, true, false, ['Standard'], true, 'granted'];
    }

    #[DataProvider('decisions')]
    public function testDecisionPrecedence(bool $supported, bool $enforced, bool $featureEnabled, bool $present, bool $admin, array $packages, bool $allowed, string $reason): void
    {
        $verdict = (new UsageRightDecisionPolicy())->decide('machines', $supported, $enforced, $featureEnabled, $present, $admin, $packages);

        self::assertSame($allowed, $verdict->allowed);
        self::assertSame($reason, $verdict->reason);
        // Package sources belong only to a positive grant. Earlier refusal
        // states intentionally discard them so the UI cannot imply that a
        // disabled feature is currently usable through a package.
        self::assertSame($reason === 'granted' ? $packages : [], $verdict->packages);
    }
}
