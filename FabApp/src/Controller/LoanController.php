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
    public function catalogue(LoanableItemRepository $items, LoanRepository $loans, \Symfony\Component\HttpFoundation\Request $request): Response
    {
        $rows = $items->findAllSafe();
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
            'cards' => $cards,
            'tiles' => $tiles,
            'search' => $search,
            'category' => $category,
            'totalCount' => \count($cards),
            'allCount' => \count($rows),
            'freeCount' => \count(array_filter($cards, static fn (array $c): bool => $c['free'] > 0)),
        ]);
    }
}
