<?php

declare(strict_types=1);

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Mail\Mailer;
use App\Repository\UtilisateurRepository;
use App\Service\UserWarnings;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * S208 — le registre d'avertissements : en poser un depuis la fiche d'une
 * personne, le lever, voir la liste de tous, régler les motifs.
 *
 * ⚠️ Un REGISTRE : rien ici ne touche un droit, et la personne concernée ne voit
 * pas ses avertissements (le texte d'aide du formulaire le dit à l'équipe).
 * ⚠️ Tout répond 404 tant que la fonction `warnings` est éteinte ou que la
 * migration n'est pas passée.
 */
#[IsGranted('ROLE_ADMIN')]
final class UserWarningController extends AbstractController
{
    public function __construct(
        private readonly UserWarnings $warnings,
        private readonly SiteFeatureService $features,
    ) {
    }

    #[Route('/admin/avertissements', name: 'app_admin_warnings', methods: ['GET'])]
    public function index(Request $request): Response
    {
        $this->gate();
        $state = (string) $request->query->get('etat', '');
        $state = \in_array($state, ['active', 'lifted'], true) ? $state : '';
        $counts = $this->warnings->counts();

        return $this->render('site/admin-warnings.html.twig', [
            'rows' => $this->warnings->all($state),
            'state' => $state,
            'counts' => $counts,
            'reasons' => $this->warnings->reasons(),
        ]);
    }

    #[Route('/admin/utilisateurs/{id}/avertissements', name: 'app_admin_user_warning_add', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function add(int $id, Request $request, UtilisateurRepository $users, Mailer $mailer): Response
    {
        $this->gate();
        $user = $users->find($id);
        $actor = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createNotFoundException();
        }
        $reasonId = (int) $request->request->get('reason');
        $note = (string) $request->request->get('note');

        if (!$this->isCsrfTokenValid('admin_warning_add_' . $id, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');
        } elseif ($reasonId <= 0) {
            $this->addFlash('error', 'warnings.pick_reason');
        } else {
            $warningId = $this->warnings->add($id, $reasonId, $note, $actor instanceof Utilisateur ? $actor->getId() : null);
            if ($warningId === null) {
                $this->addFlash('error', 'warnings.pick_reason');
            } else {
                $this->addFlash('success', ['warnings.added', ['%name%' => $user->getDisplayName()]]);
                $this->notifyAdmins($users, $mailer, $user, $actor instanceof Utilisateur ? $actor : null, $reasonId, $note);
            }
        }

        return $this->redirectToRoute('app_admin_user_detail', ['id' => $id, '_fragment' => 'warnings'], Response::HTTP_SEE_OTHER);
    }

    #[Route('/admin/avertissements/{id}/lever', name: 'app_admin_warning_lift', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function lift(int $id, Request $request): Response
    {
        $this->gate();
        $userId = $this->warnings->userIdOf($id);
        if ($userId === null) {
            throw $this->createNotFoundException();
        }
        if (!$this->isCsrfTokenValid('admin_warning_lift_' . $id, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');
        } else {
            $this->addFlash('success', $this->warnings->lift($id) ? 'warnings.lifted' : 'warnings.already_lifted');
        }

        // On revient là d'où l'on vient : la fiche de la personne, ou la liste.
        if ($request->request->get('back') === 'list') {
            return $this->redirectToRoute('app_admin_warnings', [], Response::HTTP_SEE_OTHER);
        }

        return $this->redirectToRoute('app_admin_user_detail', ['id' => $userId, '_fragment' => 'warnings'], Response::HTTP_SEE_OTHER);
    }

    /** Le réglage des motifs : en ajouter un, le renommer, en retirer un de la liste (ou le remettre). */
    #[Route('/admin/avertissements/motifs', name: 'app_admin_warning_reasons', methods: ['POST'])]
    public function reasons(Request $request): Response
    {
        $this->gate();
        if (!$this->isCsrfTokenValid('admin_warning_reasons', (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.mise_a_jour_refusee_token_csrf');
        } elseif ($request->request->get('action') === 'add') {
            $saved = $this->warnings->addReason((string) $request->request->get('label'));
            $this->addFlash($saved ? 'success' : 'error', $saved ? 'warnings.reason_saved' : 'warnings.reason_refused');
        } elseif ($request->request->get('action') === 'rename') {
            $saved = $this->warnings->renameReason((int) $request->request->get('id'), (string) $request->request->get('label'));
            $this->addFlash($saved ? 'success' : 'error', $saved ? 'warnings.reason_updated' : 'warnings.reason_refused');
        } else {
            $this->warnings->setReasonActive((int) $request->request->get('id'), $request->request->get('action') === 'enable');
            $this->addFlash('success', 'warnings.reason_updated');
        }

        return $this->redirectToRoute('app_admin_warnings', ['_fragment' => 'motifs'], Response::HTTP_SEE_OTHER);
    }

    private function gate(): void
    {
        if (!$this->features->allowsSurface('warnings') || !$this->warnings->isReady()) {
            throw $this->createNotFoundException();
        }
    }

    /** Un e-mail à chaque administrateur actif (gabarit `warning_issued`, modifiable dans l'admin). */
    private function notifyAdmins(UtilisateurRepository $users, Mailer $mailer, Utilisateur $person, ?Utilisateur $by, int $reasonId, string $note): void
    {
        $reason = '';
        foreach ($this->warnings->reasons() as $r) {
            if ($r['id'] === $reasonId) {
                $reason = $r['label'];
            }
        }
        $context = [
            'person' => $person->getDisplayName(),
            'reason' => $reason,
            'note' => trim($note),
            'by' => $by?->getDisplayName() ?? '',
            'url' => $this->generateUrl('app_admin_user_detail', ['id' => $person->getId()], UrlGeneratorInterface::ABSOLUTE_URL),
        ];
        foreach ($users->findBy(['statut' => 'actif']) as $candidate) {
            if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $mailer->queueToUser($candidate, 'warning_issued', $context);
            }
        }
    }
}
