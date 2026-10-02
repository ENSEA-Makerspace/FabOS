<?php

declare(strict_types=1);

namespace App\Controller;

use App\Page\ModuleReading;
use App\Page\MyTrainings;
use App\Entity\Utilisateur;
use App\Repository\FormationRepository;
use App\Service\TrainingQualificationService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

/** Les pages d'apprentissage d'une personne : ses formations, et la lecture d'une étape. */
final class LearningController extends AbstractController
{
    #[Route('/mes-formations', name: 'app_my_trainings', methods: ['GET'])]
    #[IsGranted('ROLE_USER')]
    public function myTrainings(MyTrainings $myTrainings): Response
    {
        $user = $this->getUser();
        \assert($user instanceof Utilisateur);

        return $this->render('site/my-trainings.html.twig', ['trainings' => $myTrainings->for($user)]);
    }

    /** Même accès que la fiche formation (`app_formation_detail`) : publique, hors catégories internes. */
    #[Route('/formations/{id}/etapes/{n}', name: 'app_formation_section', requirements: ['id' => '\d+', 'n' => '\d+'], methods: ['GET'])]
    public function section(int $id, int $n, FormationRepository $formations, ModuleReading $moduleReading): Response
    {
        $formation = $formations->find($id);
        if (!$formation || TrainingQualificationService::isInternalCategory($formation->getCategorie())) {
            throw $this->createNotFoundException('Formation introuvable');
        }

        $user = $this->getUser();
        $reading = $moduleReading->build($user instanceof Utilisateur ? $user : null, $id, max(1, $n));
        if ($reading === null) {
            throw $this->createNotFoundException('Cette formation n’a pas de parcours par étapes.');
        }

        return $this->render('site/formation-section.html.twig', ['reading' => $reading]);
    }
}
