<?php

namespace App\Tests\Account;

use App\Account\AccountAnonymiser;
use App\Account\AccountGuard;
use App\Entity\Role;
use App\Entity\Utilisateur;
use App\Entity\UtilisateurRole;
use App\Repository\UtilisateurRepository;
use PHPUnit\Framework\TestCase;

/**
 * The lockout invariant, exercised rather than described.
 *
 * ⚠️ The sibling contract test reads declarations, which is the right altitude
 * for "did someone forget to scrub a column". This one is different: whether the
 * *last administrator* is refused is a decision with branches, and a branch is
 * worth running. 🔴 It is also the one mistake nobody can repair afterwards —
 * anonymisation is irreversible by design, so an operator who erases their own
 * last admin account has no way back in and no one to ask.
 */
final class AccountGuardTest extends TestCase
{
    private function user(int $id, string $email, string $statut = 'actif', ?string $role = null): Utilisateur
    {
        $user = new Utilisateur();

        // No setter for the id — it is generated. Reflection is honest here:
        // the guard compares ids, so the test has to give them.
        $ref = new \ReflectionProperty(Utilisateur::class, 'id');
        $ref->setValue($user, $id);

        $user->setEmail($email);
        $user->setUsername('u' . $id);
        $user->setStatut($statut);

        if ($role !== null) {
            $roleEntity = (new Role())->setNom($role);
            $link = (new UtilisateurRole())->setUtilisateur($user)->setRole($roleEntity);
            $user->getUtilisateurRoles()->add($link);
        }

        return $user;
    }

    private function guardOver(Utilisateur ...$all): AccountGuard
    {
        // ⚠️ A stub, not a mock: the guard's collaborator only has to answer, and
        // PHPUnit rightly warns that a mock with no configured expectations is a
        // stub wearing the wrong name.
        $repo = $this->createStub(UtilisateurRepository::class);
        $repo->method('findAll')->willReturn($all);

        return new AccountGuard($repo);
    }

    public function testTheOnlyAdministratorCannotBeErased(): void
    {
        $admin = $this->user(1, 'admin@example.test', 'actif', 'ADMIN');
        $member = $this->user(2, 'member@example.test');

        $guard = $this->guardOver($admin, $member);

        self::assertSame(AccountGuard::REFUSED_LAST_ADMIN, $guard->refusalFor($admin));
        self::assertFalse($guard->allows($admin));
    }

    public function testAnOrdinaryMemberIsAlwaysErasable(): void
    {
        $admin = $this->user(1, 'admin@example.test', 'actif', 'ADMIN');
        $member = $this->user(2, 'member@example.test');

        self::assertNull($this->guardOver($admin, $member)->refusalFor($member));
    }

    public function testAnAdministratorIsErasableOnceASecondOneExists(): void
    {
        $first = $this->user(1, 'a@example.test', 'actif', 'ADMIN');
        $second = $this->user(2, 'b@example.test', 'actif', 'ADMIN');

        self::assertNull($this->guardOver($first, $second)->refusalFor($first));
    }

    /**
     * 🔴 The subtle one. A second admin row exists, so a naive count says "fine"
     * — but that account is already erased and can never sign in again. Counting
     * rows instead of usable administrators IS the lockout.
     */
    public function testAnAlreadyErasedAdministratorDoesNotCountAsTheSecondOne(): void
    {
        $live = $this->user(1, 'a@example.test', 'actif', 'ADMIN');
        $erased = $this->user(2, 'anonymised-2@' . AccountAnonymiser::SENTINEL_DOMAIN, 'inactif', 'ADMIN');

        self::assertSame(AccountGuard::REFUSED_LAST_ADMIN, $this->guardOver($live, $erased)->refusalFor($live));
    }

    /** ⚠️ Same trap, other spelling: a suspended admin cannot let anyone back in either. */
    public function testAnInactiveAdministratorDoesNotCountAsTheSecondOne(): void
    {
        $live = $this->user(1, 'a@example.test', 'actif', 'ADMIN');
        $suspended = $this->user(2, 'b@example.test', 'inactif', 'ADMIN');

        self::assertSame(AccountGuard::REFUSED_LAST_ADMIN, $this->guardOver($live, $suspended)->refusalFor($live));
    }

    public function testAnAlreadyErasedAccountIsNotErasedTwice(): void
    {
        $admin = $this->user(1, 'a@example.test', 'actif', 'ADMIN');
        $second = $this->user(2, 'b@example.test', 'actif', 'ADMIN');
        $erased = $this->user(3, 'anonymised-3@' . AccountAnonymiser::SENTINEL_DOMAIN);

        self::assertSame(AccountGuard::REFUSED_ALREADY, $this->guardOver($admin, $second, $erased)->refusalFor($erased));
    }

    public function testTheSentinelRecognisesAnErasedAccountAndOnlyThat(): void
    {
        self::assertTrue(AccountAnonymiser::isAnonymised(
            $this->user(9, 'anonymised-9@' . AccountAnonymiser::SENTINEL_DOMAIN),
        ));

        // ⚠️ A real address that merely mentions the word must not match.
        self::assertFalse(AccountAnonymiser::isAnonymised(
            $this->user(10, 'anonymised.person@example.test'),
        ));
    }
}
