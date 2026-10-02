<?php

declare(strict_types=1);

namespace App\Page;

use App\Entity\Utilisateur;
use App\Repository\FormationRepository;
use App\Repository\ProgressionRepository;
use App\Training\LearnerJourney;

/**
 * « Mes formations » (proposition du 2026-10-01, planche `lms-my-trainings.png`) :
 * les formations commencées d'une personne, triées en à reprendre / en cours /
 * terminées, chacune avec son parcours (`LearnerJourney`, S179 — qui ORDONNE et
 * NOMME l'étape suivante ; rien n'est recalculé ici).
 */
final class MyTrainings
{
    public function __construct(
        private readonly ProgressionRepository $progressions,
        private readonly LearnerJourney $journey,
        private readonly FormationRepository $formations,
    ) {
    }

    /** @return array{resume: ?array<string, mixed>, ongoing: list<array<string, mixed>>, done: list<array<string, mixed>>, discover: list<\App\Entity\Formation>} */
    public function for(Utilisateur $user): array
    {
        $ongoing = [];
        $done = [];
        $started = [];
        foreach ($this->progressions->findVisibleByUser($user) as $progression) {
            $formation = $progression->getFormation();
            if ($formation === null) {
                continue;
            }
            $started[$formation->getId()] = true;
            $row = ['formation' => $formation, 'progression' => $progression, 'journey' => $this->journey->of($formation, $user)];
            if ($progression->isCompleted() && $row['journey']['next'] === null) {
                $done[] = $row;
            } else {
                $ongoing[] = $row;
            }
        }
        // « À reprendre » = la plus avancée : c'est elle qu'un clic peut finir.
        usort($ongoing, static fn (array $a, array $b): int => $b['journey']['percent'] <=> $a['journey']['percent']);

        // « Découvrir » : le catalogue moins ce qu'on a déjà commencé — la carte y
        // dit « Commencer », pas « Voir » (planche `lms-training-catalogue.png`).
        $discover = array_values(array_filter(
            $this->formations->findVisible(['id' => 'DESC']),
            static fn ($f): bool => !isset($started[$f->getId()]),
        ));

        return ['resume' => array_shift($ongoing), 'ongoing' => $ongoing, 'done' => $done, 'discover' => \array_slice($discover, 0, 6)];
    }
}
