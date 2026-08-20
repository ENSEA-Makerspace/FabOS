<?php

declare(strict_types=1);

namespace App\Tests\Calendar;

use PHPUnit\Framework\TestCase;

/**
 * Taking a place at a session enrols you — and never certifies you (S146e).
 *
 * 🔴 **The rule this protects is a SAFETY rule.** A badge says somebody may use a
 * machine unsupervised. Turning up to a session is not evidence of that, and a
 * trainer validates it. Anything in the enrolment path that sets `completed`,
 * touches a score or awards a badge is not a bigger version of this feature, it is
 * a different and dangerous one.
 */
final class SessionEnrolmentContractTest extends TestCase
{
    private const ROOT = __DIR__ . '/../..';
    private const SERVICE = self::ROOT . '/src/Calendar/SessionEnrolment.php';

    public function testEnrollingRecordsAStartAndNothingMore(): void
    {
        $code = preg_replace('#^\s*(//|\*|/\*).*$#m', '', file_get_contents(self::SERVICE)) ?? '';

        self::assertStringContainsString('->setCompleted(false)', $code);
        self::assertStringContainsString('->setScore(0)', $code);

        // 🔴 Nothing in here may certify.
        self::assertStringNotContainsString('setCompleted(true)', $code);
        self::assertStringNotContainsString('Badge', $code, 'Enrolment must never touch badges.');
        self::assertStringNotContainsString('setDateEnd', $code, 'A finished date is a claim this cannot make.');
    }

    /**
     * 🔴 An existing progression is returned untouched: somebody with three quizzes
     * behind them must not lose their score by signing up for a session. The unique
     * key on (user, formation) means a blind insert would throw, too.
     */
    public function testAnExistingProgressionIsNeverOverwritten(): void
    {
        $code = file_get_contents(self::SERVICE);

        self::assertStringContainsString('$existing = $this->progressions->findOneBy(', $code);
        self::assertStringContainsString("if (\$existing !== null) {\n            return \$existing;\n        }", $code);
    }

    /** ⚠️ Guests have no account, and `PROGRESSION.userId` is NOT NULL. */
    public function testGuestsAndNonSessionsAreSkipped(): void
    {
        $code = file_get_contents(self::SERVICE);

        self::assertStringContainsString('if ($formation === null || $user === null) {', $code);
        // Only a held seat enrols; the waiting list is not the room.
        self::assertStringContainsString('if (!$registration->isRegistered()) {', $code);
    }

    /**
     * ⚠️ **Both ways into a seat enrol.** Registering is one; being promoted off the
     * waiting list is the other, and enrolling only on the first would leave everyone
     * who arrived through the second un-enrolled.
     */
    public function testBothDoorsIntoASeatEnrol(): void
    {
        $service = file_get_contents(self::ROOT . '/src/Event/EventRegistrationService.php');

        $register = substr($service, (int) strpos($service, 'public function register('));
        $register = substr($register, 0, (int) strpos($register, 'public function cancel('));
        self::assertStringContainsString('$this->sessionEnrolment->enrolIfSession($registration);', $register);

        $promote = substr($service, (int) strpos($service, 'private function promoteNextIfSeatFree('));
        self::assertStringContainsString('$this->sessionEnrolment->enrolIfSession($next);', $promote);
    }

    /**
     * ⚠️ **The enrolment lands in the SAME transaction as the seat.** A seat without
     * its enrolment, or an enrolment without its seat, is a state nobody can explain.
     * The service must therefore not flush on its own.
     */
    public function testTheEnrolmentDoesNotFlushOnItsOwn(): void
    {
        self::assertStringNotContainsString('->flush()', file_get_contents(self::SERVICE));
    }

    /** ⚠️ The member is told BEFORE they click, not by the confirmation. */
    public function testTheEventPageWarnsBeforeTheButton(): void
    {
        $page = file_get_contents(self::ROOT . '/templates/site/event-detail.html.twig');

        $notice = strpos($page, "'event.registration.session_enrols'|trans");
        $button = strpos($page, "path('app_event_register'", (int) $notice);

        self::assertNotFalse($notice, 'The page must say that a place also enrols.');
        self::assertNotFalse($button);
        self::assertLessThan($button, $notice, 'The warning belongs above the button it qualifies.');
    }
}
