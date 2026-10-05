<?php

declare(strict_types=1);

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Service\Checkins;
use App\Service\SiteSettingService;
use Endroid\QrCode\Builder\Builder;
use Endroid\QrCode\ErrorCorrectionLevel;
use Endroid\QrCode\Writer\SvgWriter;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * S207 — Check-in : la page du membre (`/check-in`) et celle de la borne
 * (`/kiosk/check-in`).
 *
 * 🔴 **La borne ne cherche JAMAIS un membre par son nom** : un écran public qui
 * propose « tapez votre nom » liste les comptes du lab. Le membre s'enregistre
 * sur SON téléphone, connecté (la borne montre un QR) ; seul le visiteur sans
 * compte (palier 2) tape un nom — le sien, qui ne sort jamais de l'écran.
 *
 * 🔴 **Le palier 4 (note de projet) n'est jamais requis** : le champ n'existe que
 * s'il est allumé, et aucun chemin ne le suppose.
 */
final class CheckinController extends AbstractController
{
    /** Le plus de visiteurs sans compte acceptés par minute, toutes bornes confondues. */
    private const VISITOR_RATE_PER_MINUTE = 8;

    public function __construct(
        private readonly Checkins $checkins,
        private readonly SiteFeatureService $features,
    ) {
    }

    #[Route('/check-in', name: 'app_checkin', methods: ['GET'])]
    #[IsGranted('ROLE_USER')]
    public function member(): Response
    {
        $this->guard();
        $user = $this->getUser();

        return $this->render('site/checkin.html.twig', [
            'open' => $user instanceof Utilisateur ? $this->checkins->openFor($user) : null,
            'reasons' => $this->reasons(),
            'withNote' => $this->features->allowsSurface('checkin_project'),
        ]);
    }

    #[Route('/check-in', name: 'app_checkin_post', methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function memberPost(Request $request): Response
    {
        $this->guard();
        $user = $this->getUser();
        if (!$user instanceof Utilisateur || !$this->isCsrfTokenValid('checkin', (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');

            return $this->redirectToRoute('app_checkin', [], Response::HTTP_SEE_OTHER);
        }
        if ($request->request->get('action') === 'leave') {
            $this->checkins->leave($user);
        } else {
            $this->checkins->arrive($user, 'self', $this->chosenReason($request), $this->chosenNote($request));
        }

        return $this->redirectToRoute('app_checkin', [], Response::HTTP_SEE_OTHER);
    }

    #[Route('/kiosk/check-in', name: 'app_kiosk_checkin', methods: ['GET'])]
    public function kiosk(Request $request, SiteSettingService $settings): Response
    {
        $this->guard();
        $base = $settings->getPublicBaseUrl() ?: $request->getSchemeAndHttpHost();
        $qr = null;
        try {
            $qr = (new Builder(writer: new SvgWriter(), data: $base . $this->generateUrl('app_checkin'), errorCorrectionLevel: ErrorCorrectionLevel::Medium, size: 360, margin: 8))->build()->getDataUri();
        } catch (\Throwable) {
        }

        return $this->render('site/kiosk-checkin.html.twig', [
            'qr' => $qr,
            'walkin' => $this->features->allowsSurface('checkin_walkin'),
            'reasons' => $this->reasons(),
            'types' => Checkins::VISITOR_TYPES,
            'done' => $request->query->getBoolean('done'),
            'tooMany' => $request->query->getBoolean('slow'),
        ]);
    }

    /** Le visiteur sans compte (palier 2). Public : CSRF, pot de miel, limite de débit. */
    #[Route('/kiosk/check-in', name: 'app_kiosk_checkin_post', methods: ['POST'])]
    public function kioskPost(Request $request): Response
    {
        $this->guard();
        if (!$this->features->allowsSurface('checkin_walkin')) {
            throw $this->createNotFoundException();
        }
        if (!$this->isCsrfTokenValid('kiosk_checkin', (string) $request->request->get('_token'))) {
            return $this->redirectToRoute('app_kiosk_checkin', [], Response::HTTP_SEE_OTHER);
        }
        // Pot de miel : un champ que seul un robot remplit. On répond « merci » sans rien écrire.
        if (trim((string) $request->request->get('website')) !== '') {
            return $this->redirectToRoute('app_kiosk_checkin', ['done' => 1], Response::HTTP_SEE_OTHER);
        }
        if ($this->checkins->recentVisitorCount() >= self::VISITOR_RATE_PER_MINUTE) {
            return $this->redirectToRoute('app_kiosk_checkin', ['slow' => 1], Response::HTTP_SEE_OTHER);
        }
        $name = trim((string) $request->request->get('name'));
        if ($name === '') {
            return $this->redirectToRoute('app_kiosk_checkin', [], Response::HTTP_SEE_OTHER);
        }
        $this->checkins->arriveVisitor($name, (string) $request->request->get('type'), $this->chosenReason($request), null);

        return $this->redirectToRoute('app_kiosk_checkin', ['done' => 1], Response::HTTP_SEE_OTHER);
    }

    /** 404 tant que l'interrupteur est éteint ou la migration absente. */
    private function guard(): void
    {
        if (!$this->features->allowsSurface('checkin') || !$this->checkins->isReady()) {
            throw $this->createNotFoundException();
        }
    }

    /** @return list<array{id: int, label: string}> les motifs, seulement si le palier 3 est allumé */
    private function reasons(): array
    {
        return $this->features->allowsSurface('checkin_reason') ? $this->checkins->reasons(true) : [];
    }

    /** Le libellé d'un motif ACTIF désigné par son id : jamais de texte libre venu du client. */
    private function chosenReason(Request $request): ?string
    {
        $id = (int) $request->request->get('reason');

        return $id > 0 && $this->features->allowsSurface('checkin_reason') ? $this->checkins->reasonLabel($id) : null;
    }

    private function chosenNote(Request $request): ?string
    {
        return $this->features->allowsSurface('checkin_project') ? (string) $request->request->get('note') : null;
    }
}
