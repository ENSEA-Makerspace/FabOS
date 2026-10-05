<?php

declare(strict_types=1);

namespace App\Controller;

use App\Entity\Material;
use App\Entity\Utilisateur;
use App\Service\MaterialStock;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * S206 — régler le stock d'un matériau et y écrire une entrée / une sortie,
 * depuis la fiche admin du matériau (`admin-material-edit`, par `include`).
 *
 * 🔴 Fonction éteinte (ou migration absente) : 404, comme si la route n'existait
 * pas — aucune mention de stock ne se devine.
 */
#[IsGranted('ROLE_ADMIN')]
final class MaterialStockController extends AbstractController
{
    #[Route('/admin/materials/{id}/stock', name: 'app_admin_material_stock_save', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function save(Material $material, Request $request, MaterialStock $stock): Response
    {
        $this->guard($stock);
        if (!$this->isCsrfTokenValid('material_stock_' . $material->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');

            return $this->back($material);
        }

        $quantity = $this->number($request->request->get('quantity'));
        $threshold = $this->number($request->request->get('lowThreshold'));
        if ($quantity === null || $quantity < 0 || ($threshold !== null && $threshold < 0)) {
            $this->addFlash('error', 'stock.err_number');

            return $this->back($material);
        }
        $stock->configure((int) $material->getId(), $quantity, (string) $request->request->get('unit'), $threshold, $this->userId());
        $this->addFlash('success', 'stock.saved');

        return $this->back($material);
    }

    #[Route('/admin/materials/{id}/stock/move', name: 'app_admin_material_stock_move', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function move(Material $material, Request $request, MaterialStock $stock): Response
    {
        $this->guard($stock);
        if (!$this->isCsrfTokenValid('material_stock_move_' . $material->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');

            return $this->back($material);
        }

        $amount = $this->number($request->request->get('amount'));
        if ($amount === null || $amount <= 0) {
            $this->addFlash('error', 'stock.err_amount');

            return $this->back($material);
        }
        // Deux boutons, un seul champ : le bouton cliqué dit le sens.
        $delta = $request->request->get('direction') === 'out' ? -$amount : $amount;
        $error = $stock->move((int) $material->getId(), $delta, trim((string) $request->request->get('note')), $this->userId());
        $this->addFlash($error === null ? 'success' : 'error', $error ?? 'stock.moved');

        return $this->back($material);
    }

    private function guard(MaterialStock $stock): void
    {
        if (!$stock->active()) {
            throw $this->createNotFoundException();
        }
    }

    private function back(Material $material): Response
    {
        return $this->redirectToRoute('app_admin_material_edit', ['id' => $material->getId(), '_fragment' => 'stock'], Response::HTTP_SEE_OTHER);
    }

    private function userId(): ?int
    {
        $user = $this->getUser();

        return $user instanceof Utilisateur ? $user->getId() : null;
    }

    /** « 1,5 » ou « 1.5 » ; null si vide ou pas un nombre. */
    private function number(mixed $raw): ?float
    {
        $text = str_replace([' ', ','], ['', '.'], trim((string) $raw));

        return $text !== '' && is_numeric($text) ? (float) $text : null;
    }
}
