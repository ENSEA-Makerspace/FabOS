<?php

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Reservation\Policy\AccessPassRepository;
use App\Reservation\ReservableType;
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

    #[Route('/acces-exceptionnels', name: 'app_staff_access_passes', methods: ['GET', 'POST'])]
    public function accessPasses(Request $request, AccessPassRepository $passes, UtilisateurRepository $users): Response
    {
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('staff_access_passes', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'Action refusée : token CSRF invalide.');

                return $this->redirectToRoute('app_staff_access_passes');
            }

            $issuer = $this->security->getUser() instanceof Utilisateur ? $this->security->getUser()->getId() : null;

            if ($request->request->get('action') === 'revoke') {
                $passes->revoke($request->request->getInt('pass_id'), $issuer);
                $this->addFlash('success', 'Accès exceptionnel révoqué.');

                return $this->redirectToRoute('app_staff_access_passes');
            }

            $holder = $users->find($request->request->getInt('user_id'));
            if (!$holder instanceof Utilisateur) {
                $this->addFlash('error', 'Choisissez la personne à qui accorder cet accès.');

                return $this->redirectToRoute('app_staff_access_passes');
            }

            $date = static function (string $raw): ?\DateTimeImmutable {
                $raw = trim($raw);

                try {
                    return $raw === '' ? null : new \DateTimeImmutable($raw);
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

            $this->addFlash('success', sprintf('Accès exceptionnel accordé à %s.', $holder->getDisplayName()));

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
}
