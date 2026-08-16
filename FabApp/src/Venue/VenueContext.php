<?php

declare(strict_types=1);

namespace App\Venue;

use App\Entity\Utilisateur;
use App\Entity\Venue;
use App\Repository\VenueRepository;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpKernel\Exception\BadRequestHttpException;

/** Resolves a display filter only; authorization remains a separate future gate. */
final class VenueContext
{
    /**
     * The third answer to "which location?", for resources that can genuinely be
     * somewhere this installation does not run. Not a slug: no `VENUE` row may
     * ever carry it, and `single()` refuses it because there is nothing to write
     * configuration against.
     */
    public const OFFSITE = 'offsite';

    public function __construct(private readonly VenueRepository $venues) {}

    /**
     * ⚠️ **`$allowOffsite` adds a THIRD answer, and only where it means something
     * (S133).** Events can happen somewhere this installation does not run —
     * `Event::isOnsite()` has said so since the entity was written — and those rows
     * carry `venue = NULL`. With only "all" and a venue to choose from, "all"
     * quietly folded them in beside the onsite ones and no filter could isolate or
     * exclude them: the roadmap's words are "jamais mêlés en silence".
     *
     * It is opt-in rather than universal because "elsewhere" is nonsense for a
     * machine or a room, and a page that answered `selected: null` for it would be
     * silently showing everything. A caller that has not opted in refuses the value
     * exactly as it refuses an unknown slug.
     *
     * @return array{selected: ?Venue, location: string, venues: Venue[], offsite: bool, allows_offsite: bool}
     */
    public function forRequest(Request $request, ?Utilisateur $user, bool $allowOffsite = false): array
    {
        $base = ['venues' => $this->activeVenues(), 'offsite' => false, 'allows_offsite' => $allowOffsite];
        $slug = trim((string) $request->query->get('location', ''));

        if ($slug !== '') {
            if ($slug === 'all') {
                return ['selected' => null, 'location' => 'all'] + $base;
            }

            if ($slug === self::OFFSITE) {
                if (!$allowOffsite) {
                    throw new BadRequestHttpException('Lieu inconnu ou inactif.');
                }

                return ['selected' => null, 'location' => self::OFFSITE] + ['offsite' => true] + $base;
            }

            $venue = $this->venues->findOneBy(['slug' => $slug, 'active' => true]);
            if ($venue === null) {
                throw new BadRequestHttpException('Lieu inconnu ou inactif.');
            }

            return ['selected' => $venue, 'location' => $slug] + $base;
        }

        $preferred = $user?->getPreferredVenue();
        if ($preferred !== null && $preferred->isActive()) {
            return ['selected' => $preferred, 'location' => $preferred->getSlug()] + $base;
        }

        return ['selected' => null, 'location' => 'all'] + $base;
    }

    /**
     * Resolve exactly one venue, for screens that **edit** venue-scoped configuration.
     *
     * ⚠️ `forRequest()` is the list contract and legitimately answers "all" — a
     * catalogue aggregates. Opening hours, and anything else stored per venue,
     * cannot: there is no row to write when the answer is "all". Those screens were
     * calling `VenueRepository::findDefault()` directly instead, which is why the
     * hours page could only ever edit the default venue's week even though the
     * schema has carried `UNIQ_OPENING_HOUR_VENUE_DAY (venueId, dayOfWeek)` since
     * S106 (S131).
     *
     * Same precedence as `forRequest()` minus the "all" branch: explicit valid
     * `?location=`, then a still-active profile preference, then the default venue.
     * An unknown or inactive slug is refused rather than silently falling back —
     * silently editing a different venue's hours than the URL names is worse than
     * an error.
     *
     * @return array{selected: Venue, location: string, venues: Venue[]}
     */
    public function single(Request $request, ?Utilisateur $user): array
    {
        $venues = $this->activeVenues();
        $slug = trim((string) $request->query->get('location', ''));

        if ($slug !== '' && $slug !== 'all') {
            $venue = $this->venues->findOneBy(['slug' => $slug, 'active' => true]);
            if ($venue === null) {
                throw new BadRequestHttpException('Lieu inconnu ou inactif.');
            }

            return ['selected' => $venue, 'location' => $venue->getSlug(), 'venues' => $venues];
        }

        $preferred = $user?->getPreferredVenue();
        $venue = $preferred !== null && $preferred->isActive() ? $preferred : $this->venues->findDefault();

        if ($venue === null) {
            throw new \LogicException('Le lieu par défaut est introuvable.');
        }

        return ['selected' => $venue, 'location' => $venue->getSlug(), 'venues' => $venues];
    }

    /** @return Venue[] */
    private function activeVenues(): array
    {
        return $this->venues->findBy(['active' => true], ['name' => 'ASC']);
    }
}
