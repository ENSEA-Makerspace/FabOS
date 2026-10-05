<?php

declare(strict_types=1);

namespace App\Controller;

use App\Entity\Machine;
use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Repository\MachineRepository;
use App\Service\MachineReportQr;
use App\Service\MachineReports;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * S205 — l'équipe traite les signalements de panne.
 *
 * Liste `/admin/signalements` (tuiles Ouverts / Résolus), les trois verbes
 * (résoudre avec une note facultative, rouvrir, supprimer) et la page imprimable
 * du QR à coller sur la machine. Tout est derrière `ROLE_ADMIN` (`access_control`
 * le dit déjà pour `/admin`, l'attribut le redit) et derrière la clé
 * `machine_reports` : éteinte, ces routes n'existent pas non plus.
 *
 * ⚠️ Un POST sur ces routes porte un jeton CSRF PAR LIGNE (`report_<verbe>_<id>`),
 * et revient d'où il vient (`back=machine` : la carte « Signalements » de la
 * fiche machine ; sinon la liste).
 */
#[IsGranted('ROLE_ADMIN')]
final class MachineReportAdminController extends AbstractController
{
    public function __construct(
        private readonly SiteFeatureService $features,
        private readonly MachineReports $reports,
    ) {
    }

    #[Route('/admin/signalements', name: 'app_admin_machine_reports', methods: ['GET'])]
    public function index(Request $request): Response
    {
        $this->guard();
        $status = $request->query->get('statut') === MachineReports::RESOLVED ? MachineReports::RESOLVED : MachineReports::OPEN;
        $counts = $this->reports->counts();

        return $this->render('site/admin-machine-reports.html.twig', [
            'rows' => $this->reports->all($status),
            'status' => $status,
            'tiles' => [
                ['key' => MachineReports::OPEN, 'count' => $counts['open'], 'active' => $status === MachineReports::OPEN],
                ['key' => MachineReports::RESOLVED, 'count' => $counts['resolved'], 'active' => $status === MachineReports::RESOLVED],
            ],
        ]);
    }

    #[Route('/admin/signalements/{id<\d+>}/resoudre', name: 'app_admin_machine_report_resolve', methods: ['POST'])]
    public function resolve(int $id, Request $request): Response
    {
        $this->guard();
        $user = $this->getUser();
        $machineId = $this->reports->find($id)['machineId'] ?? null;
        if ($this->checkToken($request, 'resolve', $id)) {
            $this->reports->resolve($id, (string) $request->request->get('note'), $user instanceof Utilisateur ? $user->getId() : null);
            $this->addFlash('success', 'machine_report.flash_resolved');
        }

        return $this->back($request, $machineId);
    }

    #[Route('/admin/signalements/{id<\d+>}/rouvrir', name: 'app_admin_machine_report_reopen', methods: ['POST'])]
    public function reopen(int $id, Request $request): Response
    {
        $this->guard();
        $machineId = $this->reports->find($id)['machineId'] ?? null;
        if ($this->checkToken($request, 'reopen', $id)) {
            $this->reports->reopen($id);
            $this->addFlash('success', 'machine_report.flash_reopened');
        }

        return $this->back($request, $machineId);
    }

    #[Route('/admin/signalements/{id<\d+>}/supprimer', name: 'app_admin_machine_report_delete', methods: ['POST'])]
    public function delete(int $id, Request $request): Response
    {
        $this->guard();
        $machineId = $this->reports->find($id)['machineId'] ?? null;
        if ($this->checkToken($request, 'delete', $id)) {
            $photo = $this->reports->delete($id);
            if ($photo !== null) {
                // `basename` : ce nom vient de la base, mais on ne laisse jamais un
                // chemin sortir du dossier des signalements.
                @unlink($this->getParameter('kernel.project_dir') . '/public/uploads/machine-reports/' . basename($photo));
            }
            $this->addFlash('success', 'machine_report.flash_deleted');
        }

        return $this->back($request, $machineId);
    }

    /** La page imprimable du QR : une machine, son nom, le code et l'adresse en clair. */
    #[Route('/admin/machines/{id<\d+>}/signaler-qr', name: 'app_admin_machine_report_qr', methods: ['GET'])]
    public function qr(int $id, MachineRepository $machines, MachineReportQr $qr): Response
    {
        $this->guard();
        $machine = $machines->find($id);
        if (!$machine instanceof Machine) {
            throw $this->createNotFoundException();
        }

        return $this->render('site/machine-report-qr.html.twig', [
            'machine' => $machine,
            'qr' => $qr->svgDataUri($id),
            'url' => $qr->publicUrl($id),
        ]);
    }

    private function guard(): void
    {
        if (!$this->features->allowsSurface('machine_reports') || !$this->reports->isReady()) {
            throw $this->createNotFoundException();
        }
    }

    private function checkToken(Request $request, string $verb, int $id): bool
    {
        if ($this->isCsrfTokenValid(sprintf('report_%s_%d', $verb, $id), (string) $request->request->get('_token'))) {
            return true;
        }
        $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

        return false;
    }

    /**
     * Retour là d'où l'on vient : la fiche machine (`back=machine`) ou la liste,
     * sur la même tuile (`statut`) — on continue de traiter la file qu'on avait.
     */
    private function back(Request $request, ?int $machineId): Response
    {
        if ($request->request->get('back') === 'machine' && $machineId !== null) {
            return $this->redirectToRoute('app_machine_detail', ['id' => $machineId]);
        }

        return $this->redirectToRoute('app_admin_machine_reports', [
            'statut' => (string) $request->request->get('statut', $request->query->get('statut')) === MachineReports::RESOLVED ? MachineReports::RESOLVED : MachineReports::OPEN,
        ]);
    }
}
