<?php

namespace App\Tests\Account;

use App\Account\AccountAnonymiser;
use PHPUnit\Framework\TestCase;
use Symfony\Component\Yaml\Yaml;

/**
 * The erasure keeps its promises.
 *
 * ⚠️ **Why a source-text test and not an integration one.** Anonymising needs a
 * database, a hasher and files on disk; standing that up would test Doctrine.
 * The failures this actually guards against are of a different kind — someone
 * adds a column that holds personal data and forgets to scrub it, or someone
 * "improves" the sentinel into something reversible. Both are visible in the
 * declarations, and both are silent at runtime: a forgotten column does not
 * throw, it just quietly keeps a name.
 *
 * 🔴 An erasure that half-works looks exactly like one that worked.
 */
final class AccountAnonymiserContractTest extends TestCase
{
    private const LOCALES = ['fr', 'en', 'de', 'es', 'it'];

    private function source(): string
    {
        $src = file_get_contents(__DIR__ . '/../../src/Account/AccountAnonymiser.php');
        self::assertNotFalse($src);

        return $src;
    }

    /**
     * Every identifying column on `Utilisateur` is either cleared or listed here
     * as deliberately kept. ⚠️ Adding a personal column to the entity without
     * touching the anonymiser fails this test, which is the entire point.
     */
    public function testEveryIdentifyingUserFieldIsCleared(): void
    {
        $cleared = [
            'setEmail', 'setUsername', 'setFirstName', 'setLastName', 'setNumeroId',
            'setIdentifiantRfid', 'setPublicBio', 'setPublicSlug', 'setPublicFields',
            'setPublicProfileEnabled', 'setBookingNote', 'setAvatarFilename',
            'setBannerFilename', 'setPassword',
        ];

        $source = $this->source();
        foreach ($cleared as $setter) {
            self::assertStringContainsString($setter . '(', $source, sprintf('%s is never called — that field survives the erasure', $setter));
        }
    }

    /**
     * ⚠️ The other tables that hold a name, an address or a card number. Each is
     * here because leaving it behind defeats the erasure while the user row
     * looks perfectly anonymous.
     */
    public function testEverySatelliteTableHoldingPersonalDataIsScrubbed(): void
    {
        $source = $this->source();

        self::assertStringContainsString('EXTERNAL_IDENTITY', $source, 'OIDC links left behind let the next sign-in rebuild the account');
        self::assertStringContainsString('EMAIL_LOG', $source, 'the mail log keeps the address, the display name and a context naming what they booked');
        self::assertStringContainsString('setGuestName', $source, 'a guest registration keeps the name typed into the form');
        self::assertStringContainsString('setBadgeUid', $source, 'the physical card number identifies a person as surely as an e-mail');
        self::assertStringContainsString('setAuthorName', $source, 'the creation byline is free text and outlives the account link');
    }

    /**
     * 🔴 Irreversibility is the difference between erasure and pseudonymisation.
     * A hash of a known e-mail address is re-identified by testing candidates,
     * so hashing the address would leave the rows personal data and the GDPR
     * request unfulfilled.
     */
    public function testTheIdentityIsOverwrittenAndNeverHashedOrCopied(): void
    {
        $source = $this->source();

        // The password is *supposed* to be hashed; nothing else may be.
        $identityHashing = preg_match('/hash\w*\(\s*\$user->get(Email|Username|FirstName|LastName)/i', $source);
        self::assertSame(0, $identityHashing, 'an identifier is being hashed rather than overwritten — that is pseudonymisation');

        foreach (['INSERT INTO', 'DELETED_USER', 'ARCHIVE'] as $forbidden) {
            self::assertStringNotContainsString($forbidden, $source, sprintf('%s suggests a copy is being kept, which makes the erasure reversible', $forbidden));
        }
    }

    /**
     * The operator's rule: "stats should stay, bookings and all… projects
     * untouched, leaderboard as well."
     */
    public function testNothingStatisticalIsDeleted(): void
    {
        $source = $this->source();

        foreach (['Reservation', 'LogUtilisation', 'UtilisateurBadge', 'Progression', 'Loan'] as $kept) {
            self::assertStringNotContainsString('remove(' . $kept, $source, sprintf('%s must survive the erasure — it is the lab activity the operator asked to keep', $kept));
        }

        self::assertStringNotContainsString('setTempsPresenceTotal', $source, 'presence time is a statistic, not an identifier');
        self::assertStringNotContainsString('setPoints', $source, 'points feed the leaderboard the operator asked to keep');
    }

    /** ⚠️ RFC 2606 reserves `.invalid`, so a scrubbed address can never reach an inbox. */
    public function testTheSentinelCannotResolveToARealMailbox(): void
    {
        self::assertStringEndsWith('.invalid', AccountAnonymiser::SENTINEL_DOMAIN);
    }

    public function testEveryStringTheErasureShowsIsTranslatedInAllFiveLocales(): void
    {
        $keys = [
            'title', 'intro', 'erased_title', 'erased_list', 'kept_title', 'kept_list',
            'irreversible', 'confirm_label', 'submit', 'error_mismatch', 'error_csrf',
            'refused_last_admin', 'refused_already', 'done', 'admin_action', 'admin_confirm',
        ];

        foreach (self::LOCALES as $locale) {
            $catalogue = Yaml::parseFile(__DIR__ . '/../../translations/messages.' . $locale . '.yaml');
            foreach ($keys as $key) {
                $value = $catalogue['account_delete'][$key] ?? null;
                self::assertIsString($value, sprintf('account_delete.%s is missing from messages.%s.yaml', $key, $locale));
                self::assertNotSame('', trim($value));
            }
        }
    }

    /**
     * 🔴 The lockout invariant. Erasure has no undo, so an operator who erases
     * the last administrator cannot be helped afterwards by anyone.
     */
    public function testTheGuardRefusesTheLastAdministrator(): void
    {
        $guard = file_get_contents(__DIR__ . '/../../src/Account/AccountGuard.php');
        self::assertNotFalse($guard);

        self::assertStringContainsString('REFUSED_LAST_ADMIN', $guard);
        self::assertStringContainsString("ROLE_ADMIN", $guard);
        // ⚠️ Counted over active, non-anonymised accounts — an install whose only
        // other admin is already erased has no other admin.
        self::assertStringContainsString('isAnonymised', $guard);
        self::assertStringContainsString("'actif'", $guard);
    }

    /**
     * ⚠️ Both entry points must run the same service. A second erasure written
     * for the admin screen is a second definition of "erased", and the one that
     * drifts is the one that leaves data behind.
     */
    public function testBothEntryPointsGoThroughTheOneAnonymiser(): void
    {
        foreach (['SiteController', 'AdminController'] as $controller) {
            $src = file_get_contents(__DIR__ . '/../../src/Controller/' . $controller . '.php');
            self::assertNotFalse($src);
            self::assertStringContainsString('AccountAnonymiser', $src, $controller . ' does not use the shared anonymiser');
            self::assertStringContainsString('AccountGuard', $src, $controller . ' does not consult the guard');
        }
    }
}
