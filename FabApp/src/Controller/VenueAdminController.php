<?php

namespace App\Controller;

use App\Entity\Venue;
use App\Form\VenueType;
use App\Repository\VenueRepository;
use App\Venue\VenueGuard;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\String\Slugger\SluggerInterface;

/**
 * The **Lieux** tab of the Lieux workspace (S129).
 *
 * **Why this existed only as a contract until now.** `FeatureWorkspaceRegistry`
 * has declared the `locations` workspace with tabs `['Lieux', 'Horaires']`
 * since S103, but only the Horaires route was ever registered — so the operator
 * could see that locations were a concept and could not create one. The `Venue`
 * entity shipped in S106 with getters only. This controller is the missing half.
 *
 * ⚠️ **No delete route, deliberately.** `Machine.venueId` and
 * `LoanableItem.venueId` are `NOT NULL … ON DELETE RESTRICT`, so the database
 * refuses to remove a venue in use, and the roadmap's standing decision is
 * "archiver plutôt que supprimer". Archive is reversible; the rows keep pointing
 * at the venue and return on restore.
 */
#[Route('/admin/lieux')]
final class VenueAdminController extends AbstractController
{
    #[Route('', name: 'app_admin_venues', methods: ['GET'])]
    public function index(VenueRepository $venues, VenueGuard $guard): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $rows = [];
        foreach ($venues->findBy([], ['active' => 'DESC', 'name' => 'ASC']) as $venue) {
            $rows[] = [
                'venue' => $venue,
                'attachments' => $guard->attachmentTotal($venue),
                'refusal' => $guard->archiveRefusal($venue),
            ];
        }

        return $this->render('site/admin-venues.html.twig', [
            'venueRows' => $rows,
            'activeCount' => $venues->count(['active' => true]),
        ]);
    }

    #[Route('/new', name: 'app_admin_venue_new', methods: ['GET', 'POST'])]
    public function new(Request $request, EntityManagerInterface $em, VenueRepository $venues, SluggerInterface $slugger): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $venue = (new Venue())->setSlug('');
        $form = $this->createForm(VenueType::class, $venue, ['with_slug' => true]);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            // Blank slug is the normal path: one fewer decision on the shortest
            // route through the form. It is still shown, because it is permanent
            // and the operator should see what they are committing to.
            if ($venue->getSlug() === '') {
                $venue->setSlug($this->uniqueSlug($slugger->slug($venue->getName())->lower()->toString() ?: 'venue', $venues));
            }

            $em->persist($venue);
            $em->flush();
            $this->addFlash('success', $this->trans('venues.flash.created', ['%name%' => $venue->getName()]));

            return $this->redirectToRoute('app_admin_venues');
        }

        return $this->render('site/admin-venue-form.html.twig', [
            'venue' => $venue,
            'form' => $form,
            'isNew' => true,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    #[Route('/{id}/edit', name: 'app_admin_venue_edit', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function edit(Venue $venue, Request $request, EntityManagerInterface $em): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $form = $this->createForm(VenueType::class, $venue);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $em->flush();
            $this->addFlash('success', $this->trans('venues.flash.updated', ['%name%' => $venue->getName()]));

            return $this->redirectToRoute('app_admin_venues');
        }

        return $this->render('site/admin-venue-form.html.twig', [
            'venue' => $venue,
            'form' => $form,
            'isNew' => false,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    /**
     * Confirmation on GET, action on POST.
     *
     * The GET is not a formality: it is the only place the operator is told what
     * archiving actually costs — how many machines, spaces, events, loanable
     * items, opening hours and member preferences point at this venue and will
     * drop out of the aggregate view with it.
     */
    #[Route('/{id}/archiver', name: 'app_admin_venue_archive', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function archive(Venue $venue, Request $request, VenueGuard $guard): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $refusal = $guard->archiveRefusal($venue);

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('archive_venue_' . $venue->getId(), (string) $request->request->get('_token'))) {
                $this->addFlash('error', $this->trans('venues.flash.csrf'));

                return $this->redirectToRoute('app_admin_venues');
            }

            try {
                $guard->archive($venue);
            } catch (\DomainException $refused) {
                $this->addFlash('error', $this->trans($refused->getMessage()));

                return $this->redirectToRoute('app_admin_venues');
            }

            $this->addFlash('success', $this->trans('venues.flash.archived', ['%name%' => $venue->getName()]));

            return $this->redirectToRoute('app_admin_venues');
        }

        return $this->render('site/admin-venue-archive.html.twig', [
            'venue' => $venue,
            'attachments' => $guard->attachments($venue),
            'refusal' => $refusal,
        ]);
    }

    #[Route('/{id}/restaurer', name: 'app_admin_venue_restore', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function restore(Venue $venue, Request $request, VenueGuard $guard): Response
    {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        if (!$this->isCsrfTokenValid('restore_venue_' . $venue->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', $this->trans('venues.flash.csrf'));

            return $this->redirectToRoute('app_admin_venues');
        }

        $guard->restore($venue);
        $this->addFlash('success', $this->trans('venues.flash.restored', ['%name%' => $venue->getName()]));

        return $this->redirectToRoute('app_admin_venues');
    }

    /** Append `-2`, `-3`… until the unique index will accept it. */
    private function uniqueSlug(string $base, VenueRepository $venues): string
    {
        $base = substr($base, 0, 74);
        $candidate = $base;
        $suffix = 1;
        while ($venues->findOneBy(['slug' => $candidate]) !== null) {
            $candidate = $base . '-' . (++$suffix);
        }

        return $candidate;
    }

    /** @param array<string, string> $params */
    private function trans(string $key, array $params = []): string
    {
        return $this->container->get('translator')->trans($key, $params);
    }

    public static function getSubscribedServices(): array
    {
        return array_merge(parent::getSubscribedServices(), ['translator' => \Symfony\Contracts\Translation\TranslatorInterface::class]);
    }
}
