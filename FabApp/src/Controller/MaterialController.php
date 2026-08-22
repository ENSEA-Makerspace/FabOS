<?php

namespace App\Controller;

use App\Repository\MaterialRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * Public Materials catalogue — browse the shared catalogue of significant lab
 * materials. Gated by the admin-toggleable `materials` module (app_materials
 * route prefix → FeatureAccessSubscriber). The per-machine / per-training
 * "materials used" lists land in a follow-up slice.
 */
final class MaterialController extends AbstractController
{
    #[Route('/materiaux', name: 'app_materials', methods: ['GET'])]
    public function catalogue(MaterialRepository $materials, \Symfony\Component\HttpFoundation\Request $request): Response
    {
        $rows = $materials->findLiveSafe();
        $search = trim((string) $request->query->get('q', ''));
        $category = trim((string) $request->query->get('cat', ''));

        $cards = [];
        foreach ($rows as $m) {
            if ($category !== '' && ($m->getCategory() ?? '') !== $category) { continue; }
            if ($search !== '' && stripos($m->getName(), $search) === false) { continue; }
            $cards[] = $m;
        }
        usort($cards, static fn ($a, $b): int
            => [$a->getCategory() ?? '', $a->getName()] <=> [$b->getCategory() ?? '', $b->getName()]);

        $tiles = [];
        foreach ($rows as $m) {
            $slug = $m->getCategory() ?: '';
            if ($slug === '') { continue; }
            $tiles[$slug] ??= ['slug' => $slug, 'label' => $slug, 'total' => 0, 'free' => 0];
            $tiles[$slug]['total']++;
        }
        usort($tiles, static fn (array $a, array $b): int => $a['label'] <=> $b['label']);

        return $this->render('site/materials.html.twig', [
            'cards' => $cards,
            'tiles' => $tiles,
            'search' => $search,
            'category' => $category,
            'totalCount' => \count($cards),
            'allCount' => \count($rows),
        ]);
    }
}
