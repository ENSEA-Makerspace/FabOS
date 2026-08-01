<?php

namespace App\Controller;

use App\Entity\Machine;
use App\Entity\Utilisateur;
use App\Repository\MachineRepository;
use App\Reservation\NextFreeSlotService;
use App\Reservation\ReservableType;
use App\Service\MachineQualificationService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * S58/S59 PROPOSAL PROTOTYPE — admin-only, and meant to be deleted.
 *
 * These routes render the *proposed* page shapes against real data, mounted
 * beside the pages they would replace so the two can be compared rather than
 * imagined. They are the middle option of the three costed in the roadmap:
 * not throwaway mockups (which cannot lie about contrast but can lie about
 * everything else — real string lengths, strict_variables, five locales, dark
 * mode, 320px), and not the whole phase.
 *
 * ⚠️ DELETE THIS FILE, `templates/site/proposition/`, `public/css/proposal.css`
 * and the `^/proposition` access_control line when S58/S59 ship for real. A
 * parallel page that outlives its session becomes the seventh shape, which is
 * precisely what the consistency contract forbids.
 *
 * ⚠️ Strings here are hardcoded French, NOT translation keys. That is
 * deliberate: locale parity is verified at 627 keys across five locales, and
 * adding ~40 throwaway keys × 5 would pollute a checked invariant for a
 * prototype. **The real sessions must key every string.**
 *
 * ⚠️ Gated on ROLE_ADMIN in security.yaml, not here — S49's rule is that an
 * affordance asks the route's own access_control line, and that only works if
 * the line exists.
 */
final class PropositionController extends AbstractController
{
    #[Route('/proposition', name: 'app_proposition_index', methods: ['GET'])]
    public function index(): Response
    {
        return $this->render('site/proposition/index.html.twig');
    }

    /**
     * SHAPE 1 — the list, as `/machines` would become: categories first, then
     * the machines, each with one unmissable state.
     *
     * ⚠️ This page deliberately does the expensive thing and then MEASURES it.
     * Per-card availability is exactly the work S47 refused to build ("11
     * machines, not server-paginated, so a per-card next-free-slot is 11+
     * queries growing linearly with the lab") and parked as Phase H's S41. The
     * point of doing it here is that the footer prints the real cost, so the
     * decision is taken against a number instead of an adjective.
     */
    #[Route('/proposition/machines', name: 'app_proposition_machines', methods: ['GET'])]
    public function machines(
        Request $request,
        MachineRepository $machines,
        MachineQualificationService $qualification,
        NextFreeSlotService $nextFreeSlot,
    ): Response {
        $started = microtime(true);

        $user = $this->getUser();
        $user = $user instanceof Utilisateur ? $user : null;

        $search = trim((string) $request->query->get('q', ''));
        $category = trim((string) $request->query->get('cat', ''));

        $rows = $machines->findBy([], ['nom' => 'ASC']);

        $filtered = array_values(array_filter($rows, static function (Machine $m) use ($search, $category): bool {
            if ($category !== '' && ($m->getCategorySlug() ?? '') !== $category) {
                return false;
            }
            if ($search !== '' && stripos($m->getNom(), $search) === false) {
                return false;
            }

            return true;
        }));

        // Every distinct category present, for the filter select — built from
        // the unfiltered set so choosing one does not empty the dropdown.
        $categories = [];
        foreach ($rows as $m) {
            $slug = $m->getCategorySlug() ?? '';
            if ($slug === '') {
                continue;
            }
            $categories[$slug] = $m->getCategoryLabel() ?: $slug;
        }
        asort($categories);

        // ⚠️ Grouping the grid BY category was the first design and it was wrong:
        // eleven machines across six categories means most sections render one
        // card and three empty cells, six times over. Sections cannot fill rows.
        // So categories move UP into a filter bar — more visible, always on
        // screen, carrying the counts — and the grid below stays one continuous
        // flow. Cards are still ordered by category so same-kind machines
        // cluster; they just no longer each start a new row.
        $slotCalls = 0;
        $cards = [];
        foreach ($filtered as $machine) {
            $status = $qualification->getStatus($machine, $user);

            // ⚠️ Availability is only worth asking for when the member is
            // actually allowed to book: offering "libre à 14:00" on a machine
            // they are not trained for is a promise book() would refuse. Same
            // rule S47 encoded in NextFreeSlotService.
            $slot = null;
            $raw = strtolower($machine->getStatut());
            $usable = ($status['authorized'] ?? false) && !\in_array($raw, ['maintenance', 'panne'], true);
            if ($usable) {
                $slotCalls++;
                $slot = $nextFreeSlot->find($user, ReservableType::Machine, (int) $machine->getId());
            }

            $cards[] = [
                'machine' => $machine,
                'authorized' => (bool) ($status['authorized'] ?? false),
                'slot' => $slot,
                'free' => $usable && $slot !== null,
                'catSlug' => $machine->getCategorySlug() ?: '_autres',
                'catLabel' => $machine->getCategoryLabel() ?: 'Sans catégorie',
                'makeModel' => self::guessMakeModel($machine->getNom()),
            ];
        }

        // Cluster by category, then by name, so the single grid still reads as
        // ordered without needing a heading per group.
        usort($cards, static fn (array $a, array $b): int
            => [$a['catLabel'], $a['machine']->getNom()] <=> [$b['catLabel'], $b['machine']->getNom()]);

        // The filter bar's tiles: every category, with how many machines it has
        // and how many are free right now — the number that decides whether the
        // member walks over there. Counted over the UNFILTERED set so choosing
        // one category does not blank out the others' counts.
        $tiles = [];
        foreach ($rows as $machine) {
            $slug = $machine->getCategorySlug() ?: '_autres';
            $tiles[$slug] ??= [
                'slug' => $slug,
                'label' => $machine->getCategoryLabel() ?: 'Sans catégorie',
                'total' => 0,
                'free' => 0,
            ];
            $tiles[$slug]['total']++;
        }
        foreach ($cards as $card) {
            if ($card['free'] && isset($tiles[$card['catSlug']])) {
                $tiles[$card['catSlug']]['free']++;
            }
        }
        usort($tiles, static fn (array $a, array $b): int => $a['label'] <=> $b['label']);

        $chips = [];
        if ($search !== '') {
            $chips[] = ['label' => 'Nom : ' . $search, 'url' => $this->dropParam($request, 'q')];
        }
        if ($category !== '') {
            $chips[] = [
                'label' => 'Catégorie : ' . ($categories[$category] ?? $category),
                'url' => $this->dropParam($request, 'cat'),
            ];
        }

        return $this->render('site/proposition/machines.html.twig', [
            'cards' => $cards,
            'tiles' => $tiles,
            'categories' => $categories,
            'search' => $search,
            'category' => $category,
            'chips' => $chips,
            'totalCount' => \count($filtered),
            'allCount' => \count($rows),
            'freeCount' => \count(array_filter($cards, static fn (array $c): bool => $c['free'])),
            'slotCalls' => $slotCalls,
            'elapsedMs' => (int) round((microtime(true) - $started) * 1000),
        ]);
    }

    /**
     * SHAPE 2 — the detail card, as `/machines/{id}` would become: the answer
     * first, the bookkeeping behind it.
     */
    #[Route('/proposition/machines/{id}', name: 'app_proposition_machine', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function machine(
        int $id,
        MachineRepository $machines,
        MachineQualificationService $qualification,
        NextFreeSlotService $nextFreeSlot,
    ): Response {
        $machine = $machines->find($id);
        if (!$machine) {
            throw $this->createNotFoundException('Machine introuvable');
        }

        $user = $this->getUser();
        $user = $user instanceof Utilisateur ? $user : null;

        $status = $qualification->getStatus($machine, $user);
        $slot = ($status['authorized'] ?? false)
            ? $nextFreeSlot->find($user, ReservableType::Machine, (int) $machine->getId())
            : null;

        return $this->render('site/proposition/machine-detail.html.twig', [
            'machine' => $machine,
            'authorized' => (bool) ($status['authorized'] ?? false),
            'badgeRows' => $status['badgeRows'] ?? [],
            'slot' => $slot,
        ]);
    }

    /**
     * PROTOTYPE ONLY. ⚠️ THIS MUST NOT SHIP.
     *
     * A machine is three things and we store one string for all of them:
     *
     *   individual name   "Uranus"          — how you point at THIS unit in the lab
     *   make & model      "Ultimaker S5"    — what the hardware is; what you search
     *   category          "Impression 3D"   — the family; what you filter
     *
     * `Machine` has a `nom` and a category, and nothing else. So today the make
     * is smuggled into the name — "Imprimante 3D Ultimaker S5" — and the
     * individual name does not exist at all.
     *
     * Rather than invent values, this peels a known type word off the front of
     * the real name so the card can show a real make/model where one is really
     * there, and show nothing where there is not. **Both outcomes are the point**:
     * three of eleven machines have a model in their name and eight do not, which
     * is itself the argument for real columns.
     *
     * ⚠️ It is a display heuristic on free text and it will be wrong the moment
     * somebody types a name differently. **The fix is two columns (S74), not a
     * better regex.** Nothing downstream may depend on this.
     */
    private static function guessMakeModel(string $name): ?string
    {
        $types = [
            'Imprimante 3D', 'Découpeuse Vinyle', 'Découpeuse Laser',
            'Station de Soudure', 'Oscilloscope', 'Brodeuse', 'Fraiseuse',
        ];

        foreach ($types as $type) {
            if (stripos($name, $type) !== 0) {
                continue;
            }
            $rest = trim(substr($name, \strlen($type)));

            // A leftover that is itself just a qualifier ("Numérique", "CNC",
            // "test") is not a model. Require something that looks like a
            // product name: at least one digit, or two words.
            if ($rest === '' || mb_strlen($rest) < 3) {
                return null;
            }

            // ⚠️ Both of this list's entries are here because the first run got
            // them wrong, which is the point: "Découpeuse Laser CO2" yielded
            // "CO2" (a laser type, not a make) because it contains a digit, and
            // "Découpeuse Vinyle Silhouette" yielded nothing (a real brand)
            // because it is one word. **A heuristic over free text has no way to
            // tell a product name from a qualifier**, and every lab will type
            // something this list has not seen. Two columns, not a longer regex.
            if (preg_match('/^(test|numérique|electronique|électronique|cnc|co2|laser|vinyle)$/iu', $rest) === 1) {
                return null;
            }

            return $rest;
        }

        return null;
    }

    /** Rebuild the current query string without one parameter — the chip's ✕. */
    private function dropParam(Request $request, string $drop): string
    {
        $params = $request->query->all();
        unset($params[$drop]);

        return $this->generateUrl('app_proposition_machines', $params);
    }
}
