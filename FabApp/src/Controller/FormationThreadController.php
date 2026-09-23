<?php

namespace App\Controller;

use App\Entity\Formation;
use App\Entity\Utilisateur;
use App\Form\ThreadMessageType;
use App\Training\FormationThreads;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

/**
 * Le fil privé apprenant ↔ équipe de formation (S183b).
 *
 * 🔴 **Deux portes, et aucune n'est sous `/admin`.** L'équipe qui répond est le
 * groupe `trainers` ; un formateur qui n'est pas administrateur ne passerait pas
 * le pare-feu de `/admin`, et un administrateur qui n'est pas formateur n'a rien
 * à lire ici. La boîte vit donc sous `/formateur`, gardée par `ROLE_TRAINER`.
 *
 * ⚠️ **Les gardes sont faites ICI, pas seulement par le lien.** Un lien absent
 * n'empêche pas de taper l'URL : chaque action redemande qui a le droit.
 */
final class FormationThreadController extends AbstractController
{
    public function __construct(
        private readonly FormationThreads $threads,
        private readonly EntityManagerInterface $em,
    ) {
    }

    /**
     * Le fil de l'apprenant pour cette formation.
     *
     * ⚠️ **Le fil n'est CRÉÉ qu'au premier envoi**, pas à la simple ouverture :
     * sinon chaque apprenant curieux ferait apparaître un fil vide dans la boîte
     * de l'équipe. La boîte ne liste que les fils qui ont un message.
     */
    #[Route('/formations/{id}/messages', name: 'app_formation_thread', requirements: ['id' => '\d+'], methods: ['GET', 'POST'])]
    public function learner(Formation $formation, Request $request): Response
    {
        $this->denyAccessUnlessGranted('ROLE_USER');
        $user = $this->getUser();
        if (!$user instanceof Utilisateur || !$this->threads->canWrite($formation, $user)) {
            throw $this->createAccessDeniedException();
        }

        $thread = $this->threads->threadFor($formation, $user, false);
        $form = $this->createForm(ThreadMessageType::class);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $thread ??= $this->threads->threadFor($formation, $user, true);
            $sent = $thread !== null && $this->threads->post($thread, $formation, $user, (string) $form->get('body')->getData()) !== null;
            $this->addFlash($sent ? 'success' : 'error', $sent ? 'thread.sent' : 'thread.not_sent');

            return $this->redirectToRoute('app_formation_thread', ['id' => $formation->getId()]);
        }

        if ($thread !== null) {
            $this->threads->markRead((int) $thread['id'], (int) $user->getId());
        }

        return $this->render('site/formation-messages.html.twig', [
            'formation' => $formation,
            'messages' => $thread !== null ? $this->threads->messages((int) $thread['id']) : [],
            'learnerId' => (int) $user->getId(),
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    /** La boîte de l'équipe : tous les fils, les plus récents d'abord. */
    #[Route('/formateur/messages', name: 'app_trainer_inbox', methods: ['GET'])]
    public function inbox(): Response
    {
        $this->denyAccessUnlessGranted('ROLE_TRAINER');
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException();
        }

        return $this->render('site/trainer-messages.html.twig', [
            'threads' => $this->threads->inbox($user),
            'available' => $this->threads->isAvailable(),
        ]);
    }

    #[Route('/formateur/messages/{thread}', name: 'app_trainer_thread', requirements: ['thread' => '\d+'], methods: ['GET', 'POST'])]
    public function trainer(int $thread, Request $request): Response
    {
        $this->denyAccessUnlessGranted('ROLE_TRAINER');
        $user = $this->getUser();
        $row = $this->threads->find($thread);
        if (!$user instanceof Utilisateur || $row === null || !$this->threads->canRead($row, $user)) {
            throw $this->createNotFoundException();
        }

        $formation = $this->em->find(Formation::class, (int) $row['formationId']);
        $learner = $this->em->find(Utilisateur::class, (int) $row['learnerId']);
        if (!$formation instanceof Formation) {
            throw $this->createNotFoundException();
        }

        $form = $this->createForm(ThreadMessageType::class);
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            $sent = $this->threads->post($row, $formation, $user, (string) $form->get('body')->getData()) !== null;
            $this->addFlash($sent ? 'success' : 'error', $sent ? 'thread.sent' : 'thread.not_sent');

            return $this->redirectToRoute('app_trainer_thread', ['thread' => $thread]);
        }

        $this->threads->markRead($thread, (int) $user->getId());

        return $this->render('site/trainer-thread.html.twig', [
            'formation' => $formation,
            'learner' => $learner,
            'messages' => $this->threads->messages($thread),
            'learnerId' => (int) $row['learnerId'],
            'form' => $form,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }
}
