<?php

declare(strict_types=1);

namespace App\Controller;

use App\Feature\SiteFeatureService;
use App\Service\Checkins;
use App\Service\SiteSettingService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\HttpFoundation\StreamedResponse;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S207 — `/admin/check-in` : qui est là maintenant, les visites de la période,
 * les motifs. Le compteur « présents maintenant » vit ICI et nulle part ailleurs :
 * ce n'est pas une action à faire, donc pas dans « À traiter ».
 */
#[IsGranted('ROLE_ADMIN')]
final class CheckinAdminController extends AbstractController
{
    private const PERIODS = ['today', '7', '30'];

    public function __construct(
        private readonly Checkins $checkins,
        private readonly SiteFeatureService $features,
        private readonly SiteSettingService $siteSettings,
        private readonly TranslatorInterface $translator,
    ) {
    }

    #[Route('/admin/check-in', name: 'app_admin_checkin', methods: ['GET'])]
    public function index(Request $request): Response
    {
        $this->guard();
        $period = $this->period($request);
        $tiles = [];
        foreach (self::PERIODS as $p) {
            $tiles[] = [
                'label' => $this->translator->trans('checkin.period_' . $p),
                'count' => $this->checkins->countSince($this->checkins->periodStart($p)),
                'query' => ['period' => $p === 'today' ? null : $p],
                'active' => $p === $period,
            ];
        }
        $present = $this->checkins->present();

        return $this->render('site/admin-checkin.html.twig', [
            'present' => $present,
            'visits' => $this->checkins->since($this->checkins->periodStart($period)),
            'tiles' => $tiles,
            'period' => $period,
            'reasons' => $this->checkins->reasons(),
            'showReasons' => $this->features->allowsSurface('checkin_reason'),
            'showNote' => $this->features->allowsSurface('checkin_project'),
            'tz' => new \DateTimeZone($this->siteSettings->getTimezone()),
        ]);
    }

    #[Route('/admin/check-in/export.csv', name: 'app_admin_checkin_export', methods: ['GET'])]
    public function export(Request $request): Response
    {
        $this->guard();
        $visits = $this->checkins->since($this->checkins->periodStart($this->period($request)));
        $tz = new \DateTimeZone($this->siteSettings->getTimezone());
        $withNote = $this->features->allowsSurface('checkin_project');
        $withReason = $this->features->allowsSurface('checkin_reason');

        $response = new StreamedResponse(static function () use ($visits, $tz, $withNote, $withReason): void {
            $out = fopen('php://output', 'wb');
            if ($out === false) {
                return;
            }
            $head = ['date', 'arrivee', 'depart', 'nom', 'type', 'source'];
            $withReason && $head[] = 'motif';
            $withNote && $head[] = 'note';
            fputcsv($out, $head);
            foreach ($visits as $v) {
                $end = $v['end'] ?? null;
                $line = [
                    $v['start']->setTimezone($tz)->format('Y-m-d'),
                    $v['start']->setTimezone($tz)->format('H:i'),
                    $end instanceof \DateTimeInterface ? $end->setTimezone($tz)->format('H:i') : '',
                    // Une cellule qui commence par = + - @ serait lue comme une formule par un tableur.
                    preg_replace('/^[=+\-@]/', "'$0", (string) $v['name']),
                    $v['userId'] ? 'member' : (string) $v['visitorType'],
                    $v['source'],
                ];
                $withReason && $line[] = (string) $v['reason'];
                $withNote && $line[] = preg_replace('/^[=+\-@]/', "'$0", (string) $v['projectNote']);
                fputcsv($out, $line);
            }
            fclose($out);
        });
        $response->headers->set('Content-Type', 'text/csv; charset=UTF-8');
        $response->headers->set('Content-Disposition', 'attachment; filename="fabos-check-in.csv"');

        return $response;
    }

    /** Les motifs : ajouter, renommer, désactiver, ordonner. */
    #[Route('/admin/check-in/motifs', name: 'app_admin_checkin_reasons', methods: ['POST'])]
    public function reasons(Request $request): Response
    {
        $this->guard();
        if (!$this->isCsrfTokenValid('admin_checkin_reasons', (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');

            return $this->redirectToRoute('app_admin_checkin', ['_fragment' => 'motifs'], Response::HTTP_SEE_OTHER);
        }
        $id = (int) $request->request->get('id');
        match ((string) $request->request->get('action')) {
            'add' => $this->checkins->addReason((string) $request->request->get('label')),
            'rename' => $this->checkins->renameReason($id, (string) $request->request->get('label')),
            'enable' => $this->checkins->setReasonActive($id, true),
            'disable' => $this->checkins->setReasonActive($id, false),
            'move_up' => $this->checkins->moveReason($id, -1),
            'move_down' => $this->checkins->moveReason($id, 1),
            default => null,
        };

        return $this->redirectToRoute('app_admin_checkin', ['_fragment' => 'motifs'], Response::HTTP_SEE_OTHER);
    }

    private function guard(): void
    {
        if (!$this->features->allowsSurface('checkin') || !$this->checkins->isReady()) {
            throw $this->createNotFoundException();
        }
    }

    private function period(Request $request): string
    {
        $p = (string) $request->query->get('period', 'today');

        return \in_array($p, self::PERIODS, true) ? $p : 'today';
    }
}
