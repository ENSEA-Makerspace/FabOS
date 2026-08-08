<?php

namespace App\Controller;

use App\Mail\NotificationCategory;
use App\Mail\NotificationPreferences;
use App\Mail\UnsubscribeLinker;
use App\Repository\UtilisateurRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * The other end of the unsubscribe link in every non-transactional mail.
 *
 * Public on purpose: someone who wants to stop hearing from the lab should not
 * have to remember a password to say so. The signed URL is the authorisation —
 * it names one user and one category and cannot be edited into naming another.
 *
 * GET only *offers* to unsubscribe; the change happens on POST. That split is
 * not politeness, it is correctness: mail clients and security appliances
 * routinely fetch every link in a message before the human sees it, and a GET
 * that mutated would unsubscribe people who never clicked anything. The POST
 * endpoint doubles as the RFC 8058 one-click target, which is why it checks the
 * signature instead of a CSRF token — the recipient of a mail has no session to
 * carry one.
 */
final class UnsubscribeController extends AbstractController
{
    public function __construct(
        private readonly UnsubscribeLinker $linker,
        private readonly NotificationPreferences $preferences,
        private readonly UtilisateurRepository $people,
    ) {
    }

    #[Route('/desabonnement', name: UnsubscribeLinker::ROUTE, methods: ['GET', 'POST'])]
    public function unsubscribe(Request $request): Response
    {
        if (!$this->linker->isValid($request)) {
            return $this->render('site/unsubscribe.html.twig', ['state' => 'invalid'], new Response('', Response::HTTP_BAD_REQUEST));
        }

        $userId = $request->query->getInt('user');
        $category = (string) $request->query->get('category', '');

        $user = $userId > 0 ? $this->people->find($userId) : null;
        if ($user === null || !NotificationCategory::isOptOutable($category)) {
            return $this->render('site/unsubscribe.html.twig', ['state' => 'invalid'], new Response('', Response::HTTP_BAD_REQUEST));
        }

        if ($request->isMethod('POST')) {
            // The same signed link serves both directions, so someone who
            // unsubscribed by accident can undo it on the spot rather than
            // having to find the setting while logged in.
            $resubscribe = $request->request->get('action') === 'resubscribe';
            $resubscribe
                ? $this->preferences->optIn($userId, $category)
                : $this->preferences->optOut($userId, $category);

            return $this->render('site/unsubscribe.html.twig', [
                'state' => $resubscribe ? 'resubscribed' : 'unsubscribed',
                'categoryLabel' => NotificationCategory::label($category),
                'email' => $user->getEmail(),
            ]);
        }

        return $this->render('site/unsubscribe.html.twig', [
            'state' => $this->preferences->accepts($userId, $category) ? 'confirm' : 'already',
            'categoryLabel' => NotificationCategory::label($category),
            'email' => $user->getEmail(),
        ]);
    }
}
