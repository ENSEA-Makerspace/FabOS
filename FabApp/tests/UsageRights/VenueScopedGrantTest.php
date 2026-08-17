<?php

namespace App\Tests\UsageRights;

use PHPUnit\Framework\TestCase;

/**
 * A venue-scoped grant has to be able to refuse something.
 *
 * 🔴 **It could not, for the whole of S133b and S134.** The column existed, the
 * migration wrote it, the query read it — and every caller passed `null` for the
 * venue, so the `:venue IS NULL` branch matched every grant and a restriction to
 * one location behaved exactly like no restriction at all. It was found by an
 * operator asking how to scope a grant to a location, which is not a question a
 * feature should still be answering with "you can't, and it wouldn't work".
 *
 * ⚠️ **Why a source-text test rather than an integration one.** Exercising the
 * real path needs a database with two locations, a package, an assignment and a
 * member, and the thing that broke would survive all of it: the code was correct
 * everywhere except that one argument was never supplied. What has to be pinned
 * is the *wiring* — that the venue reaches the query — and the wiring is exactly
 * what source text shows. The behaviour of the SQL itself is pinned by
 * `UsageRightDecisionPolicyTest` and by the query's own NULL handling.
 */
final class VenueScopedGrantTest extends TestCase
{
    private const SERVICE = __DIR__ . '/../../src/UsageRights/UsageRightsService.php';
    private const REPOSITORY = __DIR__ . '/../../src/UsageRights/UsageGrantRepository.php';
    private const RESERVATIONS = __DIR__ . '/../../src/Reservation/ReservationService.php';
    private const RESOLVER = __DIR__ . '/../../src/Reservation/ReservableResolver.php';

    /**
     * ⚠️ **Rewritten in S144b, and the guard has to be as tight as it was.** The
     * five positional nullables became a `UsageScope` when the resource and the
     * interval joined the venue — that is exactly the refactor during which a
     * dimension quietly stops being passed, so this asserts the venue survives the
     * whole path: caller → scope → query parameter.
     */
    public function testVerdictAcceptsAScopeAndHandsItsVenueToTheGrantQuery(): void
    {
        $source = file_get_contents(self::SERVICE);

        self::assertStringContainsString(
            '?UsageScope $scope = null): UsageRightVerdict',
            $source,
            'verdict() must take a scope, or no caller can ever ask a location- or resource-specific question.',
        );

        // 🔴 The exact regression this file was written for: the venue argument
        // was the literal `null` in every caller for two sessions.
        self::assertStringContainsString(
            '$scope?->venueId,',
            $source,
            'The scope handed to paths() must carry the caller venue through.',
        );
        self::assertStringContainsString(
            'UsageGrantAction::Use, $askedScope',
            $source,
            'verdict() must pass the scope it built into paths().',
        );

        self::assertStringContainsString(
            "'venue' => \$scope->venueId,",
            file_get_contents(self::REPOSITORY),
            'And the query parameter must read the venue off the scope, not default it to null.',
        );
    }

    /**
     * The booking chokepoint has to supply every dimension it knows, because each
     * one is permissive when absent — a restriction that cannot be evaluated is
     * not enforced, so an omission here reads as a working feature and behaves as
     * nothing at all.
     */
    public function testTheBookingChokepointSuppliesTheResourceAndTheInterval(): void
    {
        $source = file_get_contents(self::RESERVATIONS);

        self::assertStringContainsString('categoryLabelFor($type, $id)', $source);
        self::assertStringContainsString('public function categoryLabelFor(', file_get_contents(self::RESOLVER));

        self::assertStringContainsString(
            'new UsageScope($venueId, $type->value, $reservableId, $categoryLabel, $from, $until)',
            file_get_contents(self::SERVICE),
            'A booking knows its location, its kind, its resource, its category and its hours; all five belong in the scope.',
        );
    }

    public function testTheBookingChokepointSuppliesTheResourcesVenue(): void
    {
        self::assertStringContainsString(
            'venueIdFor($type, $id)',
            file_get_contents(self::RESERVATIONS),
            'The booking path must tell the rights service which location is being booked.',
        );

        self::assertStringContainsString(
            'public function venueIdFor(',
            file_get_contents(self::RESOLVER),
            'The resolver is what knows a machine or a space belongs to a location.',
        );
    }

    /**
     * ⚠️ Null must stay permissive, and that is a decision rather than an
     * accident. A caller with no location — an overview, an appointment with a
     * person — cannot evaluate a location restriction, and refusing on a question
     * you did not ask is worse than answering it broadly.
     */
    public function testAnUnaskedVenueQuestionStaysPermissive(): void
    {
        self::assertStringContainsString(
            'g.venueId IS NULL OR :venue IS NULL OR g.venueId = :venue',
            file_get_contents(self::REPOSITORY),
            'Both NULLs matter: an unrestricted grant, and a caller not asking about a location.',
        );
    }

    /**
     * 🔴 S133b created `USAGE_GRANT` beside the `USAGE_PACKAGE_GRANT` that S111
     * had already shipped — two tables for one concept. S134b converged them, and
     * this is what stops the duplicate coming back the next time somebody reads
     * "S111 persists it" as a plan rather than as a record.
     */
    public function testThereIsOneGrantTable(): void
    {
        $source = file_get_contents(self::REPOSITORY);

        self::assertStringContainsString('USAGE_PACKAGE_GRANT', $source);
        self::assertDoesNotMatchRegularExpression(
            '/\bUSAGE_GRANT\b(?!_)/',
            preg_replace('/^\s*(\*|\/\/|--).*$/m', '', $source) ?? '',
            'USAGE_GRANT was the duplicate; the reader must consult USAGE_PACKAGE_GRANT only.',
        );
    }
}
