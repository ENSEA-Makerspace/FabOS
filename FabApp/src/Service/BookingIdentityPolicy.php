<?php

namespace App\Service;

use App\Entity\Utilisateur;
use Symfony\Bundle\SecurityBundle\Security;

/**
 * Decides whether the current viewer may see *who* booked a slot.
 *
 * **Why this exists.** Until 2026-08-01 `/calendrier`, `/calendar` and
 * `/machines/{id}/calendrier` server-rendered every booking's real name and
 * free-text `motif` into the page, and all three answered 200 to an anonymous
 * visitor from the internet (S38). Gating the JSON API while the page published the
 * same rows would have been theatre.
 *
 * **Why a policy object and not an `is_granted()` in the template.** What the
 * calendar shows is a *product* decision that differs by institution — a school may
 * want trainers to see names while students see only that a slot is taken — so it is
 * an operator setting, not a role literal. Putting the rule here means the two
 * calendar templates cannot drift apart, and the next surface that shows bookings
 * asks the same question instead of inventing its own answer.
 */
final class BookingIdentityPolicy
{
    public function __construct(
        private readonly Security $security,
        private readonly SiteSettingService $siteSettings,
    ) {
    }

    /**
     * ⚠️ Each configured role is tested on its own and the results OR'd, because
     * **this app has no role hierarchy** — `ROLE_ADMIN` does not imply `ROLE_STAFF`.
     * Testing only the "highest" configured role would hide names from admins on an
     * install that ticked staff alone.
     */
    public function canSeeOthersIdentity(): bool
    {
        foreach ($this->siteSettings->getBookingIdentityRoles() as $role) {
            if ($this->security->isGranted($role)) {
                return true;
            }
        }

        return false;
    }

    /**
     * The viewer's own id, or null when nobody is signed in.
     *
     * ⚠️ The calendars badge your own booking as "Ma réservation", and that must keep
     * working when identities are hidden — a member has to recognise their own slot.
     * The templates therefore compare against **this** and emit a server-computed
     * `mine` flag rather than shipping every booking's `user_id` to the browser:
     * publishing the ids would let anyone group a person's slots together and follow
     * their week, which is most of what a name gives you anyway.
     */
    public function viewerId(): ?int
    {
        $user = $this->security->getUser();

        // ⚠️ Typed against the entity rather than probed with method_exists(): the
        // anonymous case passes null, and method_exists(null, …) is a TypeError in
        // PHP 8 — which took the three calendar pages down on first deploy, i.e.
        // exactly the visitor this code exists to serve.
        return $user instanceof Utilisateur ? $user->getId() : null;
    }
}
