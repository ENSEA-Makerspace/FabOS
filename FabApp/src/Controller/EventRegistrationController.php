<?php

namespace App\Controller;

use App\Entity\Event;
use App\Entity\EventRegistration;
use App\Entity\Utilisateur;
use App\Event\EventRegistrationService;
use App\Event\TicketLinker;
use App\Mail\UnsubscribeLinker;
use App\Repository\EventRegistrationRepository;
use App\Service\ModuleService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * Taking and giving up a place at an event.
 *
 * Registration is open to people without an account, so none of this is behind
 * the firewall — a guest posts their name and address instead. The one thing
 * that must not be guessable is cancelling *someone else's* place, which is why
 * the guest route authenticates with a signed URL rather than a session, and
 * why a member cancelling goes through an ownership check instead.
 */
final class EventRegistrationController extends AbstractController
{
    public function __construct(
        private readonly EventRegistrationService $registrations,
        private readonly EventRegistrationRepository $repository,
        private readonly UnsubscribeLinker $links,
    ) {
    }

    #[Route('/events/{id}/inscription', name: 'app_event_register', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function register(Event $event, Request $request, ModuleService $modules): Response
    {
        if (!$modules->isEnabled('events')) {
            throw $this->createNotFoundException();
        }

        if (!$this->isCsrfTokenValid('event_register_' . $event->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Inscription refusée : token CSRF invalide.');

            return $this->redirectToRoute('app_event_detail', ['id' => $event->getId()]);
        }

        $user = $this->getUser() instanceof Utilisateur ? $this->getUser() : null;

        $result = $this->registrations->register(
            $event,
            $user,
            // A signed-in member's own address always wins over anything posted,
            // so nobody can register a colleague by typing their address in.
            $user !== null ? null : (string) $request->request->get('email'),
            $user !== null ? null : trim((string) $request->request->get('name')),
        );

        $this->addFlash($result->ok ? 'success' : 'error', (string) $result->message);

        return $this->redirectToRoute('app_event_detail', ['id' => $event->getId()]);
    }

    /** A member cancelling their own place from the event page or their profile. */
    #[Route('/events/inscriptions/{registration}/annuler', name: 'app_event_registration_cancel_own', requirements: ['registration' => '\d+'], methods: ['POST'])]
    public function cancelOwn(int $registration, Request $request): Response
    {
        $row = $this->repository->find($registration);
        $user = $this->getUser();

        if ($row === null || !$user instanceof Utilisateur || $row->getUtilisateur()?->getId() !== $user->getId()) {
            throw $this->createAccessDeniedException();
        }

        if (!$this->isCsrfTokenValid('event_cancel_' . $registration, (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'Annulation refusée : token CSRF invalide.');
        } else {
            $result = $this->registrations->cancel($row);
            $this->addFlash($result->ok ? 'success' : 'error', (string) $result->message);
        }

        $eventId = $row->getEvent()?->getId();

        return $eventId !== null
            ? $this->redirectToRoute('app_event_detail', ['id' => $eventId])
            : $this->redirectToRoute('app_events');
    }

    /**
     * The attendee's ticket: their QR, their code, and the event details.
     *
     * Signed rather than behind a login, because half the registrants are
     * guests with no account, and a ticket you can't open is not a ticket. It
     * proves *which registration* the holder is and grants nothing else — the
     * QR on it points at a staff-only route, so possessing a ticket never
     * confers the ability to check anybody in, least of all yourself.
     */
    #[Route('/events/inscriptions/{registration}/billet', name: TicketLinker::TICKET_ROUTE, requirements: ['registration' => '\d+'], methods: ['GET'])]
    public function ticket(int $registration, Request $request, TicketLinker $tickets): Response
    {
        $row = $this->repository->find($registration);

        if (!$tickets->isValid($request) || $row === null) {
            return $this->render('site/event-ticket.html.twig', ['state' => 'invalid'], new Response('', Response::HTTP_BAD_REQUEST));
        }

        return $this->render('site/event-ticket.html.twig', [
            'state' => 'ok',
            'registration' => $row,
            'event' => $row->getEvent(),
            'qr' => $tickets->qrSvgDataUri($row),
            'code' => $tickets->shortCode($row),
        ]);
    }

    /**
     * The guest's way out, reached from the link in their confirmation mail.
     *
     * GET only offers; POST performs. Same reasoning as the unsubscribe route —
     * mail clients and security scanners fetch every link in a message before a
     * human sees it, and a mutating GET would cancel people's places for them.
     */
    #[Route('/events/inscriptions/{registration}/annulation', name: UnsubscribeLinker::EVENT_CANCEL_ROUTE, requirements: ['registration' => '\d+'], methods: ['GET', 'POST'])]
    public function cancelSigned(int $registration, Request $request): Response
    {
        $row = $this->repository->find($registration);

        if (!$this->links->isValid($request) || $row === null) {
            return $this->render('site/event-cancel.html.twig', ['state' => 'invalid'], new Response('', Response::HTTP_BAD_REQUEST));
        }

        if ($request->isMethod('POST')) {
            $result = $this->registrations->cancel($row);

            return $this->render('site/event-cancel.html.twig', [
                'state' => $result->ok ? 'cancelled' : 'error',
                'registration' => $row,
                'message' => $result->message,
            ]);
        }

        return $this->render('site/event-cancel.html.twig', [
            'state' => $row->isCancelled() ? 'already' : 'confirm',
            'registration' => $row,
        ]);
    }
}
