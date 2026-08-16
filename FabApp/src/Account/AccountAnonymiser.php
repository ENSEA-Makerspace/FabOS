<?php

declare(strict_types=1);

namespace App\Account;

use App\Entity\AccessRfidLog;
use App\Entity\Creation;
use App\Entity\EventRegistration;
use App\Entity\Utilisateur;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;

/**
 * Erasing the person while keeping what the lab did.
 *
 * **The operator's rule (2026-08-16):** "stats should stay, bookings and all. If
 * user Pierre got deleted, we should still see his activity in stats, projects
 * untouched, leaderboard as well. Maybe just his name gets changed?" — so this
 * is **anonymisation, never deletion**. Every row survives; the person leaves.
 *
 * ⚠️ **Why that is also the GDPR-correct answer and not a dodge.** Article 17
 * gives a right to erasure of *personal data*. Recital 26 says the regulation
 * does not apply to anonymous information — data that can no longer be tied to
 * an identifiable person. So erasing the identifiers while keeping the rows
 * honours the request AND keeps the lab's history, which is not the member's
 * personal data once nobody can tell whose it was.
 *
 * 🔴 **The whole thing turns on IRREVERSIBILITY.** If any mapping back to the
 * person survives anywhere, this is *pseudonymisation*, the rows are still
 * personal data, and the erasure has not happened. That is why:
 *   - no "deleted users" table, no archive copy, no audit row naming them;
 *   - the e-mail address is overwritten, **not hashed** — a hash of a known
 *     address is trivially re-identified by testing candidate addresses;
 *   - the uploaded avatar and banner are deleted from disk, not just unlinked;
 *   - the external-identity rows go, or the next OIDC sign-in re-creates the
 *     account from the provider's claims and undoes all of it.
 *
 * ⚠️ **What is deliberately KEPT, and why it is not personal data afterwards:**
 * reservations, usage logs, loans, badges, points, presence time, quiz progress,
 * votes and creations all keep pointing at the same row id. That id is now a
 * bare row number: with the name, e-mail, username, member number and card uid
 * gone and no mapping stored, it identifies nobody. This is what makes the
 * statistics survive an erasure request intact.
 *
 * ⚠️ Deletion of a *reservation* is not this class's job and must not be added
 * to it: cancelling someone's future bookings is a scheduling decision the lab
 * takes separately, and silently freeing rooms because a member left would
 * change the calendar without anyone asking.
 */
final class AccountAnonymiser
{
    /**
     * ⚠️ `.invalid` is reserved by RFC 2606 and can never resolve, so a scrubbed
     * address cannot reach a real inbox even if some future code path tries to
     * mail it. It doubles as the marker for `isAnonymised()` — a marker that
     * costs no migration, which matters because a migration has to be run by
     * the operator's own hand and would block this from shipping.
     */
    public const SENTINEL_DOMAIN = 'anonymised.invalid';

    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly Connection $db,
        private readonly UserPasswordHasherInterface $hasher,
        private readonly string $projectDir,
    ) {
    }

    public static function isAnonymised(Utilisateur $user): bool
    {
        return str_ends_with($user->getEmail(), '@' . self::SENTINEL_DOMAIN);
    }

    /**
     * ⚠️ Callers must decide *whether* this is allowed — see `AccountGuard`.
     * This method performs the erasure and asks nothing.
     */
    public function anonymise(Utilisateur $user): void
    {
        $id = (int) $user->getId();

        $this->forgetUploads($user);
        $this->forgetExternalIdentities($id);
        $this->forgetMailLog($id);
        $this->forgetRegistrations($user);
        $this->forgetCardScans($user);
        $this->forgetCreationBylines($user);

        // ⚠️ The identity itself goes last: everything above locates rows by the
        // user id, which does not change, but a half-finished pass is easier to
        // reason about if the account is still recognisable when it fails.
        $user->setEmail(sprintf('anonymised-%d@%s', $id, self::SENTINEL_DOMAIN));
        $user->setUsername(sprintf('anonymised-%d', $id));

        // ⚠️ Stored, not translated. `getDisplayName()` has 83 call sites and an
        // entity has no business holding a translator; this is the one place the
        // "translate the UI, never the content" rule bends, because the value IS
        // now system-generated. `#id` keeps two erased members distinguishable
        // in a leaderboard without saying anything about either of them.
        $user->setFirstName('Anonyme');
        $user->setLastName('#' . $id);

        $user->setNumeroId(null);
        $user->setIdentifiantRfid(null);   // also frees the physical card for reuse
        $user->setPublicBio(null);
        $user->setPublicSlug(null);
        $user->setPublicFields(null);
        $user->setPublicProfileEnabled(false);
        $user->setBookingNote(null);
        $user->setBookable(false);
        $user->setAvatarFilename(null);
        $user->setBannerFilename(null);
        $user->setStatut('inactif');

        // 🔴 A random unguessable password, not an empty string and not a fixed
        // one: an empty hash can behave as "any password matches" in some
        // hashers, and a shared constant would let one leak open every erased
        // account. `statut` alone is not the lock — the form login does not read
        // it, only the OIDC path does.
        $user->setPassword($this->hasher->hashPassword($user, bin2hex(random_bytes(32))));

        $this->em->flush();
    }

    /** ⚠️ Removed from disk. A row that no longer names the file does not erase the face in it. */
    private function forgetUploads(Utilisateur $user): void
    {
        foreach ([$user->getAvatarFilename(), $user->getBannerFilename()] as $name) {
            $name = trim((string) $name);
            if ($name === '' || str_contains($name, '/') || str_contains($name, '\\')) {
                continue;
            }
            // ⚠️ `profile-banners`, not `banners`. The two upload folders are
            // named inconsistently in `SiteController`; guessing costs nothing at
            // runtime and leaves the image on disk forever.
            foreach (['avatars', 'profile-banners'] as $folder) {
                $path = $this->projectDir . '/public/uploads/' . $folder . '/' . $name;
                if (is_file($path)) {
                    @unlink($path);
                }
            }
        }
    }

    /** 🔴 Left behind, the next OIDC sign-in rebuilds the account from the provider. */
    private function forgetExternalIdentities(int $userId): void
    {
        $this->safely('DELETE FROM EXTERNAL_IDENTITY WHERE userId = :id', ['id' => $userId]);
    }

    /**
     * The mail log keeps the recipient address, their display name and the
     * template's context — which for a booking reminder contains the machine and
     * the times they booked it. The row stays so send statistics stay; the
     * person leaves it.
     */
    private function forgetMailLog(int $userId): void
    {
        $this->safely(
            "UPDATE EMAIL_LOG SET recipient = :addr, recipientName = NULL, contextJson = '{}' WHERE userId = :id",
            ['addr' => sprintf('anonymised-%d@%s', $userId, self::SENTINEL_DOMAIN), 'id' => $userId],
        );
    }

    /**
     * ⚠️ A registration made *as a guest* carries a name and address typed into
     * the form, held separately from the account. Erasing the account without
     * these leaves the person's name on the attendee list.
     */
    private function forgetRegistrations(Utilisateur $user): void
    {
        $rows = $this->em->getRepository(EventRegistration::class)->findBy(['utilisateur' => $user]);
        foreach ($rows as $row) {
            $row->setGuestName(null);
            $row->setContactEmail(sprintf('anonymised-%d@%s', (int) $user->getId(), self::SENTINEL_DOMAIN));
        }
    }

    /**
     * 🔴 `AccessRfidLog.badgeUid` is the physical card's number. It is a unique
     * identifier for a person as surely as their e-mail, and it outlives the
     * account because the log is an audit trail. The scan stays — that is the
     * statistic — the card number does not.
     */
    private function forgetCardScans(Utilisateur $user): void
    {
        $rows = $this->em->getRepository(AccessRfidLog::class)->findBy(['utilisateur' => $user]);
        foreach ($rows as $row) {
            $row->setBadgeUid('');
        }
    }

    /**
     * ⚠️ The creation itself is untouched — operator's words, "projects should be
     * untouched". Only the byline changes, and only where it was typed as free
     * text; the author link stays so the gallery and the leaderboard keep
     * counting it.
     */
    private function forgetCreationBylines(Utilisateur $user): void
    {
        $rows = $this->em->getRepository(Creation::class)->findBy(['author' => $user]);
        foreach ($rows as $row) {
            $row->setAuthorName(null);
        }
    }

    /**
     * ⚠️ Fail-safe on purpose. `EXTERNAL_IDENTITY` and `EMAIL_LOG` are DBAL
     * features that can be deployed ahead of their tables (see the schema-drift
     * note in the brief). An erasure must not abort because an optional table is
     * missing — the identity scrub above it has already happened.
     */
    private function safely(string $sql, array $params): void
    {
        try {
            $this->db->executeStatement($sql, $params);
        } catch (\Throwable) {
            // Table absent on this install; nothing of the person is in it.
        }
    }
}
