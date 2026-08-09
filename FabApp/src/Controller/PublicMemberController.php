<?php

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Repository\ProgressionRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Repository\UtilisateurRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

final class PublicMemberController extends AbstractController
{
    #[Route('/m/{slug}', name: 'app_public_member', requirements: ['slug' => '[a-z0-9-]{1,80}'], methods: ['GET'])]
    public function show(string $slug, UtilisateurRepository $users, UtilisateurBadgeRepository $badges, ProgressionRepository $progressions): Response
    {
        $user = $users->findOneBy(['publicSlug' => $slug, 'publicProfileEnabled' => true, 'statut' => 'actif']);
        if (!$user instanceof Utilisateur) { throw $this->createNotFoundException(); }
        $fields = $user->getPublicFields();

        return $this->render('site/public-member.html.twig', [
            'member' => $user,
            'fields' => $fields,
            'badges' => in_array('badges', $fields, true) ? $badges->findBy(['utilisateur' => $user], ['dateObtention' => 'DESC']) : [],
            'trainings' => in_array('trainings', $fields, true) ? array_values(array_filter($progressions->findVisibleByUser($user), static fn ($p): bool => $p->isCompleted())) : [],
        ], new Response(headers: ['X-Robots-Tag' => 'noindex, nofollow']));
    }
}
