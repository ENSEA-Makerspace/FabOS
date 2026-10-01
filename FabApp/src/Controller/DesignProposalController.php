<?php

declare(strict_types=1);

namespace App\Controller;

use App\Catalogue\MyReservations;
use App\Catalogue\PlaceCatalogue;
use App\Repository\AccessPointRepository;
use App\Repository\MachineRepository;
use App\Repository\UtilisateurRepository;
use App\Reservation\ReservableType;
use App\Design\AdminAttention;
use App\Design\MyTrainings;
use App\Design\PageProposals;
use App\Home\MemberToday;
use App\Entity\Utilisateur;
use App\Service\MarkdownDocService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * Les pages revues d'après les planches, en sous-pages du menu Développement
 * (demande de l'opérateur, 2026-10-01). Voir {@see PageProposals}.
 *
 * ⚠️ Comme `/admin/references` : non traduit (outillage de développement), aucun
 * formulaire qui enregistre. Une proposition se regarde, elle ne fait rien.
 */
#[Route('/admin/propositions')]
#[IsGranted('ROLE_ADMIN')]
final class DesignProposalController extends AbstractController
{
    #[Route('', name: 'app_admin_proposals', methods: ['GET'])]
    public function index(PageProposals $proposals, MarkdownDocService $docs): Response
    {
        return $this->render('site/admin-proposals.html.twig', [
            'proposals' => $proposals->all(),
            'triage' => $docs->render('proposals'),
        ]);
    }

    #[Route('/{slug}', name: 'app_admin_proposal', requirements: ['slug' => '[a-z0-9-]+'], methods: ['GET'])]
    public function show(
        string $slug,
        Request $request,
        PageProposals $proposals,
        PlaceCatalogue $places,
        MyReservations $myReservations,
        MachineRepository $machines,
        AccessPointRepository $doors,
        UtilisateurRepository $users,
        MemberToday $memberToday,
        MyTrainings $myTrainings,
        AdminAttention $adminAttention,
    ): Response
    {
        $proposal = $proposals->find($slug) ?? throw $this->createNotFoundException();
        $user = $this->getUser();
        $user = $user instanceof Utilisateur ? $user : null;
        // Démo : `?membre=<id>` regarde la proposition avec les données d'un compte
        // de test (lecture seule, page réservée aux admins).
        if ($request->query->getInt('membre') > 0) {
            $user = $users->find($request->query->getInt('membre')) ?? $user;
        }

        // Chaque proposition lit les MÊMES données que la page qu'elle remplacerait.
        $data = match ($slug) {
            'espaces-catalogue' => $places->build($request, $user),
            'mes-reservations' => $user === null ? [] : $this->withVisuals($myReservations->build($request, $user), $machines, $doors),
            'accueil-membre' => ['member' => $user, 'today' => $user === null ? null : $memberToday->for($user)],
            'mes-formations' => ['trainings' => $user === null ? null : $myTrainings->for($user)],
            'admin-attention' => ['attention' => $adminAttention->build()],
            default => [],
        };

        return $this->render('site/proposals/' . $slug . '.html.twig', $data + [
            'proposal' => $proposal,
            'all_proposals' => $proposals->all(),
        ]);
    }

    /**
     * « Mes réservations » : la photo de chaque machine réservée et les portes de
     * chaque espace, que la page actuelle ne charge pas.
     *
     * @param array<string, mixed> $data
     * @return array<string, mixed>
     */
    private function withVisuals(array $data, MachineRepository $machines, AccessPointRepository $doors): array
    {
        $photoOf = [];
        $doorsOf = [];
        foreach ($data['reservations'] as $reservation) {
            $id = (int) $reservation->getReservableId();
            if ($reservation->getReservableType() === ReservableType::Machine) {
                $photoOf[$reservation->getId()] = $machines->find($id)?->getPhoto();
            } elseif ($reservation->getReservableType() === ReservableType::Place) {
                $doorsOf[$reservation->getId()] = array_map(static fn ($d): string => (string) $d->getNom(), $doors->findForPlace($id));
            }
        }

        return $data + ['photoOf' => $photoOf, 'doorsOf' => $doorsOf];
    }
}
