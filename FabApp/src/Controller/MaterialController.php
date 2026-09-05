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

    /**
     * 🔴 **S174 — chaque matériau du catalogue renvoyait vers le catalogue.**
     * Les cartes de `/materiaux` et les pastilles de la fiche machine portaient
     * toutes `path('app_materials')` : cliquer sur « PLA » ramenait à la liste
     * où l'on venait de cliquer. Un objet du produit sans fiche est un objet
     * qu'on ne peut ni consulter, ni partager par lien, ni atteindre depuis la
     * machine qui l'accepte.
     *
     * ⚠️ **La route s'appelle `app_materials_detail`, pas `app_material_detail`.**
     * `FeatureAccessSubscriber` reconnaît le module au PRÉFIXE de nom de route
     * (`str_starts_with($route, 'app_materials')`) : un singulier serait passé
     * à côté de la garde, et la fiche serait restée accessible avec le module
     * Matériaux éteint. Voir [[feedback-fabos-feature-gate-fails-open]].
     *
     * ⚠️ **Un matériau archivé rend 404**, comme il disparaît déjà du catalogue
     * (`findLiveSafe`). L'écran d'administration reste le seul endroit d'où on
     * le restaure.
     */
    #[Route('/materiaux/{id}', name: 'app_materials_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function detail(int $id, MaterialRepository $materials): Response
    {
        $material = $materials->find($id);
        if ($material === null || $material->getArchivedAt() !== null) {
            throw $this->createNotFoundException('Matériau introuvable');
        }

        // ⚠️ Les machines archivées sortent : la fiche répond « où puis-je
        // l'utiliser », et une machine qui a quitté le labo n'est pas une
        // réponse. Le tri est celui du catalogue machines, pour que deux écrans
        // ne présentent pas la même liste dans deux ordres.
        $machines = [];
        foreach ($material->getMachines() as $machine) {
            if (!$machine->isArchived()) {
                $machines[] = $machine;
            }
        }
        usort($machines, static fn ($a, $b): int
            => [$a->getCategoryLabel(), $a->getNom()] <=> [$b->getCategoryLabel(), $b->getNom()]);

        return $this->render('site/material-detail.html.twig', [
            'material' => $material,
            'machines' => $machines,
        ]);
    }
}
