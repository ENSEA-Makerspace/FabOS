<?php

namespace App\Controller;

use App\Repository\LoanableItemRepository;
use App\Repository\LoanRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * Public Loans page — browse what the lab lends out and how many of each are
 * currently available. Gated by the admin-toggleable `loans` module (app_loans
 * route prefix → FeatureAccessSubscriber). Checkouts themselves are managed by
 * staff in the admin; members track their own from their profile.
 */
final class LoanController extends AbstractController
{
    #[Route('/prets', name: 'app_loans', methods: ['GET'])]
    public function catalogue(LoanableItemRepository $items, LoanRepository $loans, \Symfony\Component\HttpFoundation\Request $request, \App\Venue\VenueContext $venues): Response
    {
        // ⚠️ S138. The PUBLIC catalogue had no location filter, on an install with
        // more than one location since S129 — a member was shown every row in the
        // organisation with no way to narrow it. Same gap /machines had until S137.
        $venueContext = $venues->forRequest($request, $this->getUser() instanceof \App\Entity\Utilisateur ? $this->getUser() : null);
        $rows = $items->findAllSafe();
        if ($venueContext['selected'] !== null) {
            $rows = array_values(array_filter(
                $rows,
                static fn (\App\Entity\LoanableItem $item): bool => $item->getVenue()?->getId() === $venueContext['selected']->getId(),
            ));
        }
        $active = $loans->activeCountsByItem();
        $search = trim((string) $request->query->get('q', ''));
        $category = trim((string) $request->query->get('cat', ''));

        $cards = [];
        foreach ($rows as $item) {
            if ($category !== '' && ($item->getCategory() ?? '') !== $category) {
                continue;
            }
            if ($search !== '' && stripos($item->getName(), $search) === false) {
                continue;
            }
            $out = (int) ($active[$item->getId()] ?? 0);
            $free = max(0, $item->getQuantity() - $out);
            $cards[] = ['item' => $item, 'out' => $out, 'free' => $free];
        }
        usort($cards, static fn (array $a, array $b): int
            => [$a['item']->getCategory() ?? '', $a['item']->getName()]
            <=> [$b['item']->getCategory() ?? '', $b['item']->getName()]);

        // Tiles counted over the UNFILTERED set, so picking one category does not
        // blank out the others' numbers.
        $tiles = [];
        foreach ($rows as $item) {
            $slug = $item->getCategory() ?: '';
            if ($slug === '') { continue; }
            $tiles[$slug] ??= ['slug' => $slug, 'label' => $slug, 'total' => 0, 'free' => 0];
            $tiles[$slug]['total']++;
        }
        foreach ($cards as $c) {
            $slug = $c['item']->getCategory() ?: '';
            if ($c['free'] > 0 && isset($tiles[$slug])) { $tiles[$slug]['free']++; }
        }
        usort($tiles, static fn (array $a, array $b): int => $a['label'] <=> $b['label']);

        return $this->render('site/loans.html.twig', [
            'venueContext' => $venueContext,
            'cards' => $cards,
            'tiles' => $tiles,
            'search' => $search,
            'category' => $category,
            'totalCount' => \count($cards),
            'allCount' => \count($rows),
            'freeCount' => \count(array_filter($cards, static fn (array $c): bool => $c['free'] > 0)),
        ]);
    }

    /**
     * The canonical page for one loan object (S133).
     *
     * ⚠️ **Every card on `/prets` linked to `/prets`.** The catalogue passed
     * `url: path('app_loans')` to `_catalogue_card`, so clicking an object
     * reloaded the page you were already on — a dead affordance of the exact kind
     * the roadmap has a standing rule about, and the reason "chaque objet ouvre sa
     * fiche canonique" is a S133 line. There was nowhere for it to go: this is
     * that page, and the two admin lists point at it too.
     *
     * ⚠️ **The route name must keep the `app_loans` prefix.** That prefix is what
     * `FeatureAccessSubscriber` reads to gate the whole loans feature; named
     * `app_loan_item` this page would have stayed reachable on an install where
     * loans are switched off, which is precisely the trap the subscriber's own
     * comments record having been sprung twice already.
     *
     * ⚠️ An archived object still resolves here rather than 404ing. Somebody
     * following a link from their own loan history is asking about a thing that
     * existed; the page says it is retired instead of pretending it never was.
     */
    #[Route('/prets/{id}', name: 'app_loans_item', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function item(int $id, LoanableItemRepository $items, LoanRepository $loans): Response
    {
        $item = $items->find($id);
        if ($item === null) {
            throw $this->createNotFoundException('Objet introuvable.');
        }

        $out = (int) ($loans->activeCountsByItem()[$item->getId()] ?? 0);

        return $this->render('site/loan-item.html.twig', [
            'item' => $item,
            'out' => $out,
            'free' => max(0, $item->getQuantity() - $out),
        ]);
    }
}
