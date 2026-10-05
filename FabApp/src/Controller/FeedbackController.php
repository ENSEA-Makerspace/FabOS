<?php

declare(strict_types=1);

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Mail\Mailer;
use App\Mail\NotificationCategory;
use App\Repository\UtilisateurRepository;
use App\Service\Feedback;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Component\Security\Http\Attribute\IsGranted;
use Symfony\Contracts\Translation\TranslatorInterface;
use Twig\Environment;

/**
 * S210 — « Signaler un problème » : l'envoi (membre) et la boîte de l'équipe.
 *
 * 🔴 **Rien ici ne dépend de la page d'où l'on vient, sauf le retour.** L'adresse
 * jointe est celle que l'en-tête a écrite dans le champ caché depuis la requête
 * de la page (`app.request.requestUri`), jamais le Referer. Elle sert à deux
 * choses : être lue par l'équipe, et ramener le membre où il était. ⚠️ Pour le
 * second usage elle est REVALIDÉE (chemin local seulement) : une valeur postée
 * recopiée dans un `redirect()` est une redirection ouverte.
 *
 * La fonction est derrière la clé `feedback` et la migration : éteinte ou pas
 * migrée, les deux routes répondent 404 et l'en-tête ne montre rien.
 */
final class FeedbackController extends AbstractController
{
    private const MESSAGE_MAX = 2000;

    public function __construct(
        private readonly Feedback $feedback,
        private readonly SiteFeatureService $features,
        private readonly TranslatorInterface $translator,
        private readonly Environment $twig,
    ) {
    }

    #[Route('/retour', name: 'app_feedback_send', methods: ['POST'])]
    #[IsGranted('ROLE_USER')]
    public function send(Request $request, Mailer $mailer, UtilisateurRepository $users): Response
    {
        $this->guard();
        $page = $this->localPath((string) $request->request->get('page'));

        // Un message d'erreur ou de remerciement, rendu par l'en-tête (type `feedback`) :
        // les pages publiques n'ont pas de zone de flashes commune.
        if (!$this->isCsrfTokenValid('feedback', (string) $request->request->get('_token'))) {
            $this->addFlash('feedback_error', $this->translator->trans('feedback.csrf_error'));

            return $this->redirect($page);
        }

        $user = $this->getUser();
        $kind = (string) $request->request->get('kind');
        $message = trim((string) $request->request->get('message'));
        if (!$user instanceof Utilisateur || !in_array($kind, Feedback::KINDS, true) || $message === '') {
            $this->addFlash('feedback_error', $this->translator->trans('feedback.invalid'));

            return $this->redirect($page);
        }
        if ($this->feedback->tooMany((int) $user->getId())) {
            $this->addFlash('feedback_error', $this->translator->trans('feedback.too_many'));

            return $this->redirect($page);
        }

        $message = mb_substr($message, 0, self::MESSAGE_MAX);
        $agent = mb_substr((string) $request->headers->get('User-Agent'), 0, 255);
        // La version de FabOS : le global Twig `app_version` (config/packages/twig.yaml).
        $version = (string) ($this->twig->getGlobals()['app_version'] ?? '');
        $this->feedback->add((int) $user->getId(), $kind, $message, $page, $agent !== '' ? $agent : null, $version !== '' ? mb_substr($version, 0, 40) : null);

        // Un e-mail à chaque membre de l'équipe : la boîte admin fait foi, le
        // courrier n'est qu'un avertissement (jamais bloquant, jamais fatal).
        $context = [
            'author' => $user->getDisplayName(),
            'kind' => $kind,
            'body' => $message,
            'page' => $request->getSchemeAndHttpHost() . $page,
            'link' => $this->generateUrl('app_admin_feedback', [], UrlGeneratorInterface::ABSOLUTE_URL),
        ];
        foreach ($this->feedback->teamIds() as $teamId) {
            $member = $users->find($teamId);
            if ($member instanceof Utilisateur) {
                try {
                    $mailer->queueToUser($member, 'feedback_received', $context, NotificationCategory::GENERAL, false);
                } catch (\Throwable) {
                    // un courrier qui échoue ne perd pas le retour : il est déjà en base
                }
            }
        }

        $this->addFlash('feedback', $this->translator->trans('feedback.thanks'));

        return $this->redirect($page);
    }

    #[Route('/admin/retours', name: 'app_admin_feedback', methods: ['GET'])]
    #[IsGranted('ROLE_ADMIN')]
    public function index(Request $request): Response
    {
        $this->guard();
        $status = $request->query->get('statut') === 'done' ? 'done' : 'open';
        $kind = (string) $request->query->get('type');
        $kind = in_array($kind, Feedback::KINDS, true) ? $kind : '';

        return $this->render('site/admin-feedback.html.twig', [
            // `createdAt` est un instant machine en UTC : on le type ici pour que `|lab_date` convertisse juste.
            'rows' => array_map(
                static fn (array $row): array => $row + ['createdAtUtc' => new \DateTimeImmutable((string) $row['createdAt'], new \DateTimeZone('UTC'))],
                $this->feedback->list($status, $kind),
            ),
            'counts' => $this->feedback->counts(),
            'status' => $status,
            'kind' => $kind,
            'kinds' => Feedback::KINDS,
        ]);
    }

    #[Route('/admin/retours/{id<\d+>}/statut', name: 'app_admin_feedback_status', methods: ['POST'])]
    #[IsGranted('ROLE_ADMIN')]
    public function status(int $id, Request $request): Response
    {
        $this->guard();
        $row = $this->feedback->find($id);
        if ($row === null) {
            throw $this->createNotFoundException();
        }
        if (!$this->isCsrfTokenValid('admin_feedback', (string) $request->request->get('_token'))) {
            $this->addFlash('error', $this->translator->trans('feedback.csrf_error'));

            return $this->redirectToRoute('app_admin_feedback');
        }

        $done = $request->request->get('to') === 'done';
        $this->feedback->setDone($id, $done);

        // Retour à l'onglet d'où l'on vient (la liste que l'on vient de vider).
        return $this->redirectToRoute('app_admin_feedback', $done ? [] : ['statut' => 'done']);
    }

    /** 404 si la fonction est éteinte ou la migration pas passée. */
    private function guard(): void
    {
        if (!$this->features->allowsSurface('feedback') || !$this->feedback->isReady()) {
            throw $this->createNotFoundException();
        }
    }

    /** Un chemin local, sinon l'accueil : jamais une adresse extérieure. */
    private function localPath(string $path): string
    {
        $path = mb_substr($path, 0, 500);
        if ($path === '' || $path[0] !== '/' || str_starts_with($path, '//') || str_contains($path, '\\') || preg_match('/[\x00-\x1f]/', $path)) {
            return $this->generateUrl('app_home');
        }

        return $path;
    }
}
