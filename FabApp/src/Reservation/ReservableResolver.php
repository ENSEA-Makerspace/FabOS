<?php

namespace App\Reservation;

use App\Entity\Reservation;
use App\Repository\MachineRepository;
use App\Repository\PlaceRepository;
use App\Repository\UtilisateurRepository;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;

/**
 * Turns a reservation's (reservableType, reservableId) pair back into something
 * displayable. This is the one place that knows how each resource kind maps to
 * a repository, a name and a URL — adding a bookable kind is a case here plus a
 * case in ReservableType.
 *
 * Without a FK there is no join, so a naive per-row lookup would be N+1 on every
 * listing page. warm() takes a whole result set and issues one query per kind
 * ("WHERE id IN (...)"); resolve() then reads from the request-scoped cache.
 * Callers that forget to warm still get correct output, just one query per row.
 *
 * Repository lookups are wrapped fail-safe: a resource kind whose table is not
 * there yet degrades to the snapshotted label rather than 500ing the page.
 */
final class ReservableResolver
{
    /** @var array<string, ReservableRef> keyed by "type:id" */
    private array $cache = [];

    public function __construct(
        private readonly MachineRepository $machines,
        private readonly PlaceRepository $places,
        private readonly UtilisateurRepository $users,
        private readonly UrlGeneratorInterface $urls,
    ) {
    }

    /**
     * Pre-load every resource referenced by these reservations, one query per
     * kind. Call this before rendering a list.
     *
     * @param iterable<Reservation> $reservations
     */
    public function warm(iterable $reservations): void
    {
        /** @var array<string, list<int>> $wanted */
        $wanted = [];
        foreach ($reservations as $reservation) {
            $type = $reservation->getReservableType();
            $id = $reservation->getReservableId();
            if ($type === null || $id === null || isset($this->cache[$type->value . ':' . $id])) {
                continue;
            }
            $wanted[$type->value][$id] = $id;
        }

        foreach ($wanted as $typeValue => $ids) {
            $type = ReservableType::from($typeValue);
            foreach ($this->fetch($type, array_values($ids)) as $id => $resource) {
                $this->cache[$typeValue . ':' . $id] = $this->build($type, $id, $resource);
            }
        }
    }

    public function resolve(Reservation $reservation): ReservableRef
    {
        $type = $reservation->getReservableType();
        $id = $reservation->getReservableId();
        $label = $reservation->getReservableLabel();

        if ($type === null || $id === null) {
            return ReservableRef::unknown($label ?? '');
        }

        $key = $type->value . ':' . $id;
        if (!isset($this->cache[$key])) {
            $found = $this->fetch($type, [$id]);
            $this->cache[$key] = $this->build($type, $id, $found[$id] ?? null);
        }

        $ref = $this->cache[$key];

        // Resource gone: fall back to the name snapshotted at booking time.
        if (!$ref->exists && $label !== null && $label !== '') {
            return new ReservableRef($type, $id, $label, false);
        }

        return $ref;
    }

    /** Current name of a resource, or null if it no longer exists. */
    public function nameFor(ReservableType $type, int $id): ?string
    {
        $key = $type->value . ':' . $id;
        if (!isset($this->cache[$key])) {
            $found = $this->fetch($type, [$id]);
            $this->cache[$key] = $this->build($type, $id, $found[$id] ?? null);
        }

        $ref = $this->cache[$key];

        return $ref->exists ? $ref->name : null;
    }

    /**
     * Which location this resource belongs to, for a venue-scoped grant (S134b).
     *
     * ⚠️ **`null` means "this question has no location", not "no location
     * found".** A venue-scoped grant is permissive when the caller passes null —
     * see the comment on the venue clause in `UsageGrantRepository::paths()` — so
     * a wrong `null` here silently widens a restriction rather than narrowing it.
     * Two cases return it deliberately:
     *
     *  - **`ReservableType::User`.** An appointment is with a person, and a
     *    person is not at a location the way a laser cutter is. Scoping somebody's
     *    availability by venue is a different feature and is not this one.
     *  - **A resource that no longer exists.** Refusing on a lookup failure would
     *    turn a deleted machine into a permissions error, which is the wrong
     *    diagnosis to hand somebody; the booking path already answers 404 for it
     *    before reaching here.
     */
    public function venueIdFor(ReservableType $type, int $id): ?int
    {
        if ($type === ReservableType::User) {
            return null;
        }

        try {
            $resource = $this->fetch($type, [$id])[$id] ?? null;
            if ($resource === null || !method_exists($resource, 'getVenue')) {
                return null;
            }

            return $resource->getVenue()?->getId();
        } catch (\Throwable) {
            return null;
        }
    }

    /**
     * Which family this resource belongs to, for a category-scoped grant (S144b).
     *
     * "Lets you 3D print" is a sentence about a *kind* of machine, not about one
     * serial number, and a lab that buys a fourth printer must not have to edit
     * every package it sells. `MACHINE.categoryLabel` is the join key S133 kept
     * deliberately — the category CRUD renames it, it is not an id — so it is what
     * a grant stores.
     *
     * ⚠️ Same rule as the venue: `null` is permissive downstream, so it means
     * "this question has no category" and never "lookup failed, refuse".
     * Only machines have one.
     */
    public function categoryLabelFor(ReservableType $type, int $id): ?string
    {
        if ($type !== ReservableType::Machine) {
            return null;
        }

        try {
            $resource = $this->fetch($type, [$id])[$id] ?? null;
            if ($resource === null || !method_exists($resource, 'getCategoryLabel')) {
                return null;
            }

            // ⚠️ The EFFECTIVE label, not the stored one. `getCategoryLabel()`
            // falls back to "Impression 3D" when the column is null, and that
            // fallback is what every list, filter and card shows — so it is the
            // category an operator believes the machine is in, and the one they
            // will have picked in the grant editor.
            $label = trim($resource->getCategoryLabel());

            return $label === '' ? null : $label;
        } catch (\Throwable) {
            return null;
        }
    }

    /** @return array<int, object> resources by id */
    private function fetch(ReservableType $type, array $ids): array
    {
        if ($ids === []) {
            return [];
        }

        try {
            $rows = match ($type) {
                ReservableType::Machine => $this->machines->findBy(['id' => $ids]),
                ReservableType::Place => $this->places->findBy(['id' => $ids]),
                ReservableType::User => $this->users->findBy(['id' => $ids]),
            };
        } catch (\Throwable) {
            return [];
        }

        $byId = [];
        foreach ($rows as $row) {
            $byId[(int) $row->getId()] = $row;
        }

        return $byId;
    }

    private function build(ReservableType $type, int $id, ?object $resource): ReservableRef
    {
        if ($resource === null) {
            return new ReservableRef($type, $id, '', false);
        }

        return match ($type) {
            ReservableType::Machine => new ReservableRef(
                $type,
                $id,
                $resource->getNom(),
                true,
                $this->url('app_machine_detail', $id),
                $this->url('app_machine_calendar', $id),
            ),
            ReservableType::Place => new ReservableRef(
                $type,
                $id,
                $resource->getNom(),
                true,
                $this->url('app_place_detail', $id),
            ),
            ReservableType::User => new ReservableRef(
                $type,
                $id,
                $resource->getDisplayName(),
                true,
                $this->url('app_person_booking', $id),
            ),
        };
    }

    /** A disabled module 404s its routes but must not break an unrelated listing page. */
    private function url(string $route, int $id): ?string
    {
        try {
            return $this->urls->generate($route, ['id' => $id]);
        } catch (\Throwable) {
            return null;
        }
    }
}
