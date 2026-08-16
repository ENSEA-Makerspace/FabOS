<?php

declare(strict_types=1);

namespace App\Search;

use App\Entity\Creation;
use App\Entity\Event;
use App\Entity\Formation;
use App\Entity\LabPage;
use App\Entity\LoanableItem;
use App\Entity\Machine;
use App\Entity\Material;
use App\Entity\Place;
use App\Feature\SiteFeatureService;
use App\Repository\BadgeRepository;
use App\Repository\CreationRepository;
use App\Repository\EventRepository;
use App\Repository\FormationRepository;
use App\Repository\LabPageRepository;
use App\Repository\LoanableItemRepository;
use App\Repository\MachineRepository;
use App\Repository\MaterialRepository;
use App\Repository\PlaceRepository;
use App\Repository\UtilisateurRepository;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * The one place that answers "what does this word match, anywhere in FabOS".
 *
 * 🔴 Why this exists as a service. `/recherche` used to know **four** kinds of
 * thing — users, machines, formations, badges — out of the ten the product
 * publishes. Searching `usb` returned nothing while a loanable item was named
 * exactly that, and a project called `valentin` was equally invisible. That is a
 * *coverage* fault, not a ranking one: half the catalogue simply was not looked
 * at. Both reports came from the operator (2026-08-12, 2026-08-16).
 *
 * ⚠️ Every group is gated by its own site feature. A hit whose module is off
 * would hand the member a link to a page that 404s — the machines branch already
 * did this correctly and nothing else did, so formations and badges were offered
 * even with their feature switched off.
 *
 * ⚠️ Group headings reuse the **`nav.*`** catalogue keys on purpose. The heading a
 * result sits under must read as the menu entry the member would click to get
 * there; two vocabularies for one section is how they drift apart.
 */
final class SiteSearch
{
    /** Descriptions are teasers under a heading, not the record itself. */
    private const DESCRIPTION_LIMIT = 200;

    public function __construct(
        private readonly UtilisateurRepository $users,
        private readonly MachineRepository $machines,
        private readonly PlaceRepository $places,
        private readonly EventRepository $events,
        private readonly FormationRepository $formations,
        private readonly BadgeRepository $badges,
        private readonly LoanableItemRepository $loanables,
        private readonly MaterialRepository $materials,
        private readonly LabPageRepository $labPages,
        private readonly CreationRepository $creations,
        private readonly SiteFeatureService $features,
        private readonly UrlGeneratorInterface $urls,
        private readonly TranslatorInterface $translator,
    ) {
    }

    /**
     * @return list<array{key: string, label_key: string, items: list<array{title: string, description: string, meta: string, url: string}>}>
     */
    public function groups(string $query, bool $includeUsers): array
    {
        $needle = mb_strtolower(trim($query));

        $groups = [
            ['key' => 'users', 'label_key' => 'search.group.users', 'items' => $needle !== '' && $includeUsers ? $this->searchUsers($needle) : []],
            ['key' => 'machines', 'label_key' => 'nav.machines', 'items' => $this->collect($needle, 'machines', $this->searchMachines(...))],
            ['key' => 'places', 'label_key' => 'nav.places', 'items' => $this->collect($needle, 'places', $this->searchPlaces(...))],
            ['key' => 'events', 'label_key' => 'nav.events', 'items' => $this->collect($needle, 'events', $this->searchEvents(...))],
            ['key' => 'formations', 'label_key' => 'nav.trainings', 'items' => $this->collect($needle, 'formations', $this->searchFormations(...))],
            ['key' => 'badges', 'label_key' => 'nav.badges', 'items' => $this->collect($needle, 'badges', $this->searchBadges(...))],
            ['key' => 'loans', 'label_key' => 'nav.loans', 'items' => $this->collect($needle, 'loans', $this->searchLoanables(...))],
            ['key' => 'materials', 'label_key' => 'nav.materials', 'items' => $this->collect($needle, 'materials', $this->searchMaterials(...))],
            ['key' => 'creations', 'label_key' => 'nav.projects', 'items' => $this->collect($needle, 'projects', $this->searchCreations(...))],
            ['key' => 'lab_pages', 'label_key' => 'nav.lab_pages', 'items' => $this->collect($needle, 'lab_pages', $this->searchLabPages(...))],
        ];

        return array_values(array_filter($groups, static fn (array $group): bool => $group['items'] !== []));
    }

    /**
     * @param callable(string): list<array{title: string, description: string, meta: string, url: string}> $search
     * @return list<array{title: string, description: string, meta: string, url: string}>
     */
    private function collect(string $needle, string $feature, callable $search): array
    {
        if ($needle === '' || !$this->features->isEnabled($feature)) {
            return [];
        }

        return $search($needle);
    }

    /** @return list<array{title: string, description: string, meta: string, url: string}> */
    private function searchUsers(string $needle): array
    {
        $hits = [];
        foreach ($this->users->findAll() as $user) {
            if (!$this->matches($needle, $user->getFirstName(), $user->getLastName(), $user->getUsername(), $user->getEmail(), $user->getIdentifiantRfid(), $user->getNumeroId())) {
                continue;
            }

            $hits[] = [
                'title' => $user->getDisplayName(),
                'description' => (string) $user->getEmail(),
                'meta' => $user->getUsername(),
                'url' => $this->urls->generate('app_admin_user_detail', ['id' => $user->getId()]),
            ];
        }

        return $hits;
    }

    /** @return list<array{title: string, description: string, meta: string, url: string}> */
    private function searchMachines(string $needle): array
    {
        $hits = [];
        foreach ($this->machines->findAll() as $machine) {
            \assert($machine instanceof Machine);
            if (!$this->matches($needle, $machine->getNom(), $machine->getDescription(), $machine->getLocalisation(), $machine->getMachineToken(), $machine->getCategoryLabel())) {
                continue;
            }

            // ⚠️ `getStatut()` is the stored word — `idle` printed raw on a French
            // page, the exact fault S84/S135 removed elsewhere. `getStatusKey()`
            // is the one mapping.
            $hits[] = [
                'title' => $machine->getNom(),
                'description' => $this->teaser($machine->getDescription()),
                'meta' => $this->join($machine->getLocalisation(), $this->translator->trans($machine->getStatusKey())),
                'url' => $this->urls->generate('app_machine_detail', ['id' => $machine->getId()]),
            ];
        }

        return $hits;
    }

    /** @return list<array{title: string, description: string, meta: string, url: string}> */
    private function searchPlaces(string $needle): array
    {
        $hits = [];
        foreach ($this->places->findBy([], ['nom' => 'ASC']) as $place) {
            \assert($place instanceof Place);
            if (!$this->matches($needle, $place->getNom(), $place->getDescription(), $place->getCategory())) {
                continue;
            }

            $hits[] = [
                'title' => $place->getNom(),
                'description' => $this->teaser($place->getDescription()),
                'meta' => $this->join($place->getCategory(), $place->getVenue()?->getName()),
                'url' => $this->urls->generate('app_place_detail', ['id' => $place->getId()]),
            ];
        }

        return $hits;
    }

    /** @return list<array{title: string, description: string, meta: string, url: string}> */
    private function searchEvents(string $needle): array
    {
        $hits = [];
        // ⚠️ `findAll` and not `findUpcoming`: searching for `anniversaire` must
        // find last year's party (operator, 2026-08-16). "À venir" is a facet on
        // the catalogue — a way of browsing — and browsing rules are not search
        // rules. The start date is in the meta line, so a past hit says so.
        foreach ($this->events->findAll() as $event) {
            \assert($event instanceof Event);
            if (!$this->matches($needle, $event->getTitre(), $event->getDescription(), $event->getLieu(), $event->getAddress())) {
                continue;
            }

            // ⚠️ An event's start is a wall-clock value the operator typed, not a
            // machine timestamp: format it as stored, never through the lab zone.
            $hits[] = [
                'title' => $event->getTitre(),
                'description' => $this->teaser($event->getDescription()),
                'meta' => $this->join($event->getDateDebut()?->format('d/m/Y H:i'), $event->getLieu()),
                'url' => $this->urls->generate('app_event_detail', ['id' => $event->getId()]),
            ];
        }

        return $hits;
    }

    /** @return list<array{title: string, description: string, meta: string, url: string}> */
    private function searchFormations(string $needle): array
    {
        $hits = [];
        foreach ($this->formations->findVisible(['id' => 'DESC']) as $formation) {
            \assert($formation instanceof Formation);
            if (!$this->matches($needle, $formation->getTitre(), $formation->getDescription(), $formation->getCategorie())) {
                continue;
            }

            $hits[] = [
                'title' => $formation->getTitre(),
                'description' => $this->teaser($formation->getDescription()),
                'meta' => (string) $formation->getBadge()?->getNom(),
                'url' => $this->urls->generate('app_formation_detail', ['id' => $formation->getId()]),
            ];
        }

        return $hits;
    }

    /** @return list<array{title: string, description: string, meta: string, url: string}> */
    private function searchBadges(string $needle): array
    {
        $hits = [];
        foreach ($this->badges->findAll() as $badge) {
            if (!$this->matches($needle, $badge->getNom(), $badge->getDescription())) {
                continue;
            }

            // 🔴 Every badge hit used to link to `app_admin_badges`. A member
            // following it got the login wall; `/badges/{id}` is the public page
            // and has existed all along.
            $hits[] = [
                'title' => $badge->getNom(),
                'description' => $this->teaser($badge->getDescription()),
                'meta' => '',
                'url' => $this->urls->generate('app_badge_detail', ['id' => $badge->getId()]),
            ];
        }

        return $hits;
    }

    /** @return list<array{title: string, description: string, meta: string, url: string}> */
    private function searchLoanables(string $needle): array
    {
        $hits = [];
        foreach ($this->loanables->findAllSafe() as $item) {
            \assert($item instanceof LoanableItem);
            if (!$this->matches($needle, $item->getName(), $item->getDescription(), $item->getCategory())) {
                continue;
            }

            // ⚠️ Loanable items have no page of their own. `/prets` filters on the
            // name, so passing the exact name always lands on the right card —
            // whereas passing the member's query would show nothing when the hit
            // came from the description.
            $hits[] = [
                'title' => $item->getName(),
                'description' => $this->teaser($item->getDescription()),
                'meta' => $this->join($item->getCategory(), $item->getVenue()?->getName()),
                'url' => $this->urls->generate('app_loans', ['q' => $item->getName()]),
            ];
        }

        return $hits;
    }

    /** @return list<array{title: string, description: string, meta: string, url: string}> */
    private function searchMaterials(string $needle): array
    {
        $hits = [];
        foreach ($this->materials->findAllSafe() as $material) {
            \assert($material instanceof Material);
            if (!$this->matches($needle, $material->getName(), $material->getDescription(), $material->getCategory())) {
                continue;
            }

            $hits[] = [
                'title' => $material->getName(),
                'description' => $this->teaser($material->getDescription()),
                'meta' => (string) $material->getCategory(),
                'url' => $this->urls->generate('app_materials', ['q' => $material->getName()]),
            ];
        }

        return $hits;
    }

    /** @return list<array{title: string, description: string, meta: string, url: string}> */
    private function searchCreations(string $needle): array
    {
        $hits = [];
        // ⚠️ `findPublishedForGallery` and not `findAll` — an unpublished creation
        // is invisible in the gallery and must stay invisible in search.
        foreach ($this->creations->findPublishedForGallery('recent') as $creation) {
            \assert($creation instanceof Creation);
            if (!$this->matches($needle, $creation->getTitle(), $creation->getDescription(), $creation->getTags(), $creation->getDisplayAuthor())) {
                continue;
            }

            $hits[] = [
                'title' => $creation->getTitle(),
                'description' => $this->teaser($creation->getDescription()),
                'meta' => $creation->getDisplayAuthor(),
                'url' => $this->urls->generate('app_creations') . '#creation-' . $creation->getId(),
            ];
        }

        return $hits;
    }

    /** @return list<array{title: string, description: string, meta: string, url: string}> */
    private function searchLabPages(string $needle): array
    {
        $hits = [];
        foreach ($this->labPages->findBy([], ['titre' => 'ASC']) as $page) {
            \assert($page instanceof LabPage);
            if (!$this->matches($needle, $page->getTitre(), $page->getContenu())) {
                continue;
            }

            $hits[] = [
                'title' => $page->getTitre(),
                'description' => $this->teaser($page->getContenu()),
                'meta' => (string) $page->getParentPage()?->getTitre(),
                'url' => $this->urls->generate('app_lab_page', ['id' => $page->getId()]),
            ];
        }

        return $hits;
    }

    /** Case-insensitive substring over every field that is worth matching. */
    private function matches(string $needle, ?string ...$fields): bool
    {
        foreach ($fields as $field) {
            if ($field !== null && $field !== '' && str_contains(mb_strtolower($field), $needle)) {
                return true;
            }
        }

        return false;
    }

    /**
     * ⚠️ Lab page bodies carry markup. Printing one raw into a result card would
     * escape into visible tag soup, so strip first and then cut.
     */
    private function teaser(?string $text): string
    {
        $clean = trim(preg_replace('/\s+/u', ' ', strip_tags((string) $text)) ?? '');
        if ($clean === '' || mb_strlen($clean) <= self::DESCRIPTION_LIMIT) {
            return $clean;
        }

        return rtrim(mb_substr($clean, 0, self::DESCRIPTION_LIMIT)) . '…';
    }

    /** Joins the parts that exist, so an absent one leaves no dangling separator. */
    private function join(?string ...$parts): string
    {
        return implode(' · ', array_filter(array_map(static fn (?string $p): string => trim((string) $p), $parts), static fn (string $p): bool => $p !== ''));
    }
}
