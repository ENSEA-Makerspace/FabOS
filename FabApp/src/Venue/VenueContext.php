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
    public function __construct(private readonly VenueRepository $venues) {}

    /** @return array{selected: ?Venue, location: string, venues: Venue[]} */
    public function forRequest(Request $request, ?Utilisateur $user): array
    {
        $slug = trim((string) $request->query->get('location', ''));
        if ($slug !== '') {
            if ($slug === 'all') {
                return ['selected' => null, 'location' => 'all', 'venues' => $this->activeVenues()];
            }

            $venue = $this->venues->findOneBy(['slug' => $slug, 'active' => true]);
            if ($venue === null) {
                throw new BadRequestHttpException('Sous-lieu inconnu ou inactif.');
            }

            return ['selected' => $venue, 'location' => $slug, 'venues' => $this->activeVenues()];
        }

        $preferred = $user?->getPreferredVenue();
        if ($preferred !== null && $preferred->isActive()) {
            return ['selected' => $preferred, 'location' => $preferred->getSlug(), 'venues' => $this->activeVenues()];
        }

        return ['selected' => null, 'location' => 'all', 'venues' => $this->activeVenues()];
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
                throw new BadRequestHttpException('Sous-lieu inconnu ou inactif.');
            }

            return ['selected' => $venue, 'location' => $venue->getSlug(), 'venues' => $venues];
        }

        $preferred = $user?->getPreferredVenue();
        $venue = $preferred !== null && $preferred->isActive() ? $preferred : $this->venues->findDefault();

        if ($venue === null) {
            throw new \LogicException('Le sous-lieu par défaut est introuvable.');
        }

        return ['selected' => $venue, 'location' => $venue->getSlug(), 'venues' => $venues];
    }

    /** @return Venue[] */
    private function activeVenues(): array
    {
        return $this->venues->findBy(['active' => true], ['name' => 'ASC']);
    }
}
