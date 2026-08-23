<?php

namespace App\Controller;

use App\Form\Admin\AccessPassType;
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
use Symfony\Component\Form\FormError;
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

    /**
     * ⚠️ **S148, J-22 — le formulaire d'octroi passe au thème.**
     *
     * 🔴 **Sept champs, et le refus redirigeait.** « Choisissez la personne »
     * renvoyait un 302 : la personne, le type, l'identifiant de ressource, les deux
     * dates, le plafond et le motif étaient tous à retaper. Il se re-rend.
     *
     * ⚠️ **La révocation reste lue à la main, et c'est délibéré** : c'est un bouton
     * par ligne portant un identifiant, pas une liste de champs. Un type unique
     * poserait le même `name` sur chaque ligne du tableau.
     */
    #[Route('/acces-exceptionnels', name: 'app_staff_access_passes', methods: ['GET', 'POST'])]
    public function accessPasses(Request $request, AccessPassRepository $passes, UtilisateurRepository $users, SiteSettingService $siteSettings): Response
    {
        $issuer = $this->security->getUser() instanceof Utilisateur ? $this->security->getUser()->getId() : null;

        if ($request->isMethod('POST') && $request->request->get('action') === 'revoke') {
            if (!$this->isCsrfTokenValid('staff_access_passes', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'flash.action_refusee_token_csrf_invalide');

                return $this->redirectToRoute('app_staff_access_passes');
            }

            $passes->revoke($request->request->getInt('pass_id'), $issuer);
            $this->addFlash('success', 'flash.derogation_de_quota_revoquee');

            return $this->redirectToRoute('app_staff_access_passes');
        }

        $people = $users->findBy(['statut' => 'actif'], ['lastName' => 'ASC', 'firstName' => 'ASC']);
        $zone = $this->labZone($siteSettings);

        $form = $this->createForm(AccessPassType::class, null, [
            'people_choices' => $this->accessPassPeopleChoices($people),
            'type_choices' => $this->accessPassTypeChoices(),
            'lab_timezone' => $zone->getName(),
            'resource_placeholder' => 'access_passes.resource_placeholder',
            'unlimited_placeholder' => 'access_passes.unlimited',
            'reason_placeholder' => 'access_passes.reason_placeholder',
        ]);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            /** @var array<string, mixed> $data */
            $data = $form->getData();
            $holder = $users->find((int) $data['user_id']);

            if (!$holder instanceof Utilisateur) {
                $form->get('user_id')->addError(new FormError('Choisissez la personne à qui accorder cet accès.'));
            } else {
                // The datetime-local field posts a bare wall-clock string. Parse it in
                // the lab zone so that, formatted back to a naive string for storage,
                // it stays the time staff actually typed — AccessPass reads it back in
                // the same zone. The server's own zone (UTC) must not enter into it.
                $date = static function (mixed $raw) use ($zone): ?\DateTimeImmutable {
                    $raw = trim((string) $raw);

                    try {
                        return $raw === '' ? null : new \DateTimeImmutable($raw, $zone);
                    } catch (\Throwable) {
                        return null;
                    }
                };

                $passes->issue(
                    (int) $holder->getId(),
                    ReservableType::tryParse((string) ($data['reservable_type'] ?? '')),
                    $data['reservable_id'] !== null ? (int) $data['reservable_id'] : null,
                    $date($data['valid_from'] ?? ''),
                    $date($data['valid_until'] ?? ''),
                    $data['max_uses'] !== null ? max(1, (int) $data['max_uses']) : null,
                    (string) ($data['reason'] ?? ''),
                    $issuer,
                );

                $this->addFlash('success', ['flash.acces_exceptionnel_accorde_a', ['%p1%' => $holder->getDisplayName()]]);

                return $this->redirectToRoute('app_staff_access_passes');
            }
        }

        return $this->render('site/staff-access-passes.html.twig', [
            'passes' => $passes->findAll(),
            'useCounts' => $passes->useCounts(),
            // ⚠️ Toujours passé : le tableau des dérogations retrouve le titulaire
            // d'une ligne dans cette liste (`people|filter(...)`).
            'people' => $people,
            'form' => $form->createView(),
            'now' => new \DateTimeImmutable(),
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    /**
     * ⚠️ Le nom ET l'e-mail, comme avant : deux personnes peuvent porter le même
     * nom, et l'écran sert à accorder un accès à l'une d'elles.
     *
     * @param list<Utilisateur> $people
     *
     * @return array<string, int>
     */
    private function accessPassPeopleChoices(array $people): array
    {
        $choices = [];
        foreach ($people as $person) {
            $choices[sprintf('%s (%s)', $person->getDisplayName(), $person->getEmail())] = (int) $person->getId();
        }

        return $choices;
    }

    /** @return array<string, string> */
    private function accessPassTypeChoices(): array
    {
        $choices = [];
        foreach (ReservableType::cases() as $type) {
            // La clé est traduite par `ChoiceType`, exactement comme le gabarit le
            // faisait avec `('reservable.type.' ~ type.value)|trans`.
            $choices['reservable.type.' . $type->value] = $type->value;
        }

        return $choices;
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
