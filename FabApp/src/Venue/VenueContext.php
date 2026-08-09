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

    /** @return Venue[] */
    private function activeVenues(): array
    {
        return $this->venues->findBy(['active' => true], ['name' => 'ASC']);
    }
}
