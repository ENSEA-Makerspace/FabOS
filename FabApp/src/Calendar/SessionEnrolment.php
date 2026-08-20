<?php

declare(strict_types=1);

namespace App\Calendar;

use App\Entity\EventRegistration;
use App\Entity\Progression;
use App\Repository\ProgressionRepository;
use Doctrine\ORM\EntityManagerInterface;

/**
 * Taking a place at a session enrols you in the training (S146e).
 *
 * 🔴 **ATTENDING NEVER QUALIFIES, and this class is where that line is drawn.**
 * Certification is a safety question: a trainer validates it. Enrolling records
 * that somebody has started — `completed = false`, `score = 0` — and nothing else.
 * Nothing here may ever set `completed`, touch a score, or award a badge, and a
 * future change that does is not an extension of this feature, it is a different
 * and much more dangerous one.
 *
 * 🔴 **An existing progression is returned UNTOUCHED.** Somebody who already did
 * three quizzes and then signs up for a session must not have their score reset or
 * their start date moved. `unique_user_formation` also means a blind insert would
 * throw, so the read is load-bearing twice over.
 *
 * ⚠️ **Guests cannot be enrolled, and that is a schema fact, not a policy choice.**
 * `PROGRESSION.userId` is `NOT NULL`: there is no account to attach progress to. A
 * guest at a session is a guest at a session.
 *
 * ⚠️ **Cancelling a place does NOT un-enrol.** The progression may by then hold real
 * work — quizzes taken, sections read — and deleting it to mirror a cancelled seat
 * would destroy it. Giving up a seat is a statement about one evening; the training
 * is a longer thing.
 */
final class SessionEnrolment
{
    public function __construct(
        private readonly ProgressionRepository $progressions,
        private readonly EntityManagerInterface $em,
    ) {}

    /**
     * Enrol the holder of this place, if it is a place at a session.
     *
     * ⚠️ Does NOT flush: the caller owns the transaction, and a place and its
     * enrolment must land together or not at all.
     *
     * @return Progression|null the progression from now on, or null when there is
     *                          nothing to enrol in — no training, or no account
     */
    public function enrolIfSession(EventRegistration $registration): ?Progression
    {
        // ⚠️ Only a held seat enrols. Somebody on the waiting list has not got into
        // the room yet; they are enrolled by the promotion path when a seat frees up.
        if (!$registration->isRegistered()) {
            return null;
        }

        $formation = $registration->getEvent()?->getFormation();
        $user = $registration->getUtilisateur();

        if ($formation === null || $user === null) {
            return null;
        }

        $existing = $this->progressions->findOneBy(['utilisateur' => $user, 'formation' => $formation]);
        if ($existing !== null) {
            return $existing;
        }

        $progression = (new Progression())
            ->setUtilisateur($user)
            ->setFormation($formation)
            // 🔴 Started, not finished. Said explicitly rather than left to the
            // column defaults, because this is the line the whole class exists for.
            ->setCompleted(false)
            ->setScore(0);

        $this->em->persist($progression);

        return $progression;
    }
}
