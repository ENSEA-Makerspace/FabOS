<?php

namespace App\Controller;

use App\Service\SiteSettingService;
use App\Entity\Utilisateur;
use App\Reservation\Policy\AccessPassRepository;
use App\Reservation\ReservableType;
use App\Event\TicketLinker;
use App\Repository\EventRegistrationRepository;
use Doctrine\ORM\EntityManagerInterface;
use App\Repository\UtilisateurRepository;
use Symfony\Bundle\SecurityBundle\Security;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/**
 * Desk tools that staff use but that are not administration.
 *
 * This lives outside /admin on purpose. The whole point of separating "staff"
 * from "admin" is that issuing a pass to a member is routine front-desk work
 * and must not require handing someone the keys to SMTP config and the user
 * table. AdminController is `#[IsGranted('ROLE_ADMIN')]` at the class level and
 * the firewall pins `^/admin` to ROLE_ADMIN as well, so a staff-accessible
 * screen could not have lived there.
 *
 * Trainers are intentionally excluded: they get elevated personal quotas, not
 * the authority to elevate other people. So nobody below staff can issue.
 */
#[Route('/staff')]
#[IsGranted(new \Symfony\Component\ExpressionLanguage\Expression("is_granted('ROLE_STAFF') or is_granted('ROLE_ADMIN')"))]
final class StaffController extends AbstractController
{
    public function __construct(private readonly Security $security)
    {
    }

    /**
     * The other end of a ticket QR: a staff member scans, this admits the person.
     *
     * It lives under /staff so the firewall does the real work — the signature
     * proves the code wasn't forged, but only ROLE_STAFF/ROLE_ADMIN keeps an
     * attendee from pointing their own phone at their own ticket and walking in.
     * That is why the QR encodes *this* route and not the ticket page.
     *
     * GET shows who the ticket belongs to and asks; POST admits them. A scan is
     * a glance before a tap — the door staff needs to see a name and whether
     * this person is even supposed to be here before anything is recorded, and a
     * mutating GET would admit whoever the camera happened to catch first.
     */
    #[Route('/billets/{registration}', name: TicketLinker::SCAN_ROUTE, requirements: ['registration' => '\d+'], methods: ['GET', 'POST'])]
    public function scanTicket(
        int $registration,
        Request $request,
        TicketLinker $tickets,
        EventRegistrationRepository $registrations,
        EntityManagerInterface $em,
    ): Response {
        $row = $registrations->find($registration);

        if (!$tickets->isValid($request) || $row === null) {
            return $this->render('site/staff-scan.html.twig', ['state' => 'invalid'], new Response('', Response::HTTP_BAD_REQUEST));
        }

        if ($request->isMethod('POST')) {
            $staff = $this->security->getUser() instanceof Utilisateur ? $this->security->getUser() : null;

            if (!$row->isCheckInEligible()) {
                $state = 'not_eligible';
            } elseif ($row->isCheckedIn()) {
                // Not an error: two staff scanning the same person is routine.
                // Say so plainly rather than pretending it worked or failed.
                $state = 'already';
            } else {
                $row->checkIn($staff);
                $em->flush();
                $state = 'admitted';
            }

            return $this->render('site/staff-scan.html.twig', [
                'state' => $state,
                'registration' => $row,
                'event' => $row->getEvent(),
            ]);
        }

        return $this->render('site/staff-scan.html.twig', [
            'state' => $row->isCheckedIn() ? 'already' : ($row->isCheckInEligible() ? 'confirm' : 'not_eligible'),
            'registration' => $row,
            'event' => $row->getEvent(),
        ]);
    }

    #[Route('/acces-exceptionnels', name: 'app_staff_access_passes', methods: ['GET', 'POST'])]
    public function accessPasses(Request $request, AccessPassRepository $passes, UtilisateurRepository $users, SiteSettingService $siteSettings): Response
    {
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('staff_access_passes', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

                return $this->redirectToRoute('app_staff_access_passes');
            }

            $issuer = $this->security->getUser() instanceof Utilisateur ? $this->security->getUser()->getId() : null;

            if ($request->request->get('action') === 'revoke') {
                $passes->revoke($request->request->getInt('pass_id'), $issuer);
                $this->addFlash('success', 'flash.derogation_de_quota_revoquee');

                return $this->redirectToRoute('app_staff_access_passes');
            }

            $holder = $users->find($request->request->getInt('user_id'));
            if (!$holder instanceof Utilisateur) {
                $this->addFlash('error', 'flash.choisissez_la_personne_a_qui_accorder');

                return $this->redirectToRoute('app_staff_access_passes');
            }

            // The datetime-local field posts a bare wall-clock string. Parse it in
            // the lab zone so that, formatted back to a naive string for storage,
            // it stays the time staff actually typed — AccessPass reads it back in
            // the same zone. The server's own zone (UTC) must not enter into it.
            $zone = $this->labZone($siteSettings);
            $date = static function (string $raw) use ($zone): ?\DateTimeImmutable {
                $raw = trim($raw);

                try {
                    return $raw === '' ? null : new \DateTimeImmutable($raw, $zone);
                } catch (\Throwable) {
                    return null;
                }
            };

            $maxUses = trim((string) $request->request->get('max_uses'));

            $passes->issue(
                (int) $holder->getId(),
                ReservableType::tryParse((string) $request->request->get('reservable_type')),
                $request->request->getInt('reservable_id') ?: null,
                $date((string) $request->request->get('valid_from')),
                $date((string) $request->request->get('valid_until')),
                $maxUses === '' || !is_numeric($maxUses) ? null : max(1, (int) $maxUses),
                (string) $request->request->get('reason'),
                $issuer,
            );

            $this->addFlash('success', ['flash.acces_exceptionnel_accorde_a', ['%p1%' => $holder->getDisplayName()]]);

            return $this->redirectToRoute('app_staff_access_passes');
        }

        return $this->render('site/staff-access-passes.html.twig', [
            'passes' => $passes->findAll(),
            'useCounts' => $passes->useCounts(),
            'people' => $users->findBy(['statut' => 'actif'], ['lastName' => 'ASC', 'firstName' => 'ASC']),
            'types' => ReservableType::cases(),
            'now' => new \DateTimeImmutable(),
        ]);
    }

    /**
     * The lab's wall-clock zone, from the operator's setting.
     *
     * ⚠️ Human-entered times are parsed **and stored** in this zone, so the naive
     * string in the database keeps the time the person actually typed (the audit is
     * S38b in docs/HISTORY.md). Machine timestamps follow the opposite rule and are
     * stored UTC — those are converted at display time by the `|lab_date` filter,
     * never here.
     */
    private function labZone(SiteSettingService $siteSettings): \DateTimeZone
    {
        return new \DateTimeZone($siteSettings->getTimezone());
    }

}
