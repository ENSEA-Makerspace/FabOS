<?php

namespace App\Training;

use App\Entity\Formation;
use App\Entity\Machine;
use App\Entity\Utilisateur;
use App\Repository\MachineBadgeRepository;
use App\Service\TrainingQualificationService;

/**
 * Où en est un apprenant, et **quoi faire ensuite** (S179).
 *
 * 🔴 **Ce que le produit ne disait pas.** Une fiche de formation portait tout :
 * description, objectifs, programme, prérequis, matériel, sessions, participation,
 * formations liées — et deux boutons génériques, « Voir ma progression » et
 * « Voir les quiz ». Aucun des deux ne répond à la seule question qu'on se pose
 * en arrivant : **qu'est-ce que je fais maintenant ?** L'apprenant devait lire la
 * page entière pour le déduire, et sa progression réelle vivait ailleurs — dans
 * un tableau brut de six colonnes, coincé entre deux graphiques de l'onglet
 * Statistiques de `/profil`.
 *
 * ✅ **Aucun modèle neuf, et c'est la contrainte de la session.**
 * `TrainingQualificationService::getStatus()` calcule déjà tout : progression
 * théorique, quiz validés sur total, exigence pratique, badge. Ce fichier ne
 * recalcule rien — il ORDONNE et NOMME. Une seconde arithmétique du même parcours
 * serait la deuxième vérité que ce projet passe son temps à retirer.
 *
 * ✅ **Et le rendu réutilise `_commissioning`** (S176), écrit pour la mise en
 * service d'un boîtier. Une liste ordonnée d'étapes faites ou non ne connaît ni
 * les boîtiers ni les formations : c'est ce qui en fait un composant. Aucun
 * dessin neuf n'a été inventé ici.
 *
 * ⚠️ **`blocking` a un sens précis** : rien ne marchera tant que ce n'est pas
 * fait, et ce n'est pas l'apprenant qui peut le faire. La validation pratique est
 * la seule étape de ce parcours dans ce cas — elle attend l'équipe.
 */
final class LearnerJourney
{
    public function __construct(
        private readonly TrainingQualificationService $qualification,
        private readonly MachineBadgeRepository $machineBadges,
    ) {
    }

    /**
     * @return array{
     *   steps: list<array{key: string, done: bool, blocking: bool}>,
     *   next: ?string,
     *   quizValidated: int, quizTotal: int, percent: int,
     *   badge: ?\App\Entity\Badge,
     *   badgeOwned: bool,
     *   machines: Machine[],
     * }
     */
    public function of(Formation $formation, ?Utilisateur $user): array
    {
        $badge = $formation->getBadge();
        $machines = $badge?->getId() !== null
            ? $this->machineBadges->machinesOpenedBy($badge->getId())
            : [];

        /*
         * ⚠️ **Un visiteur anonyme voit le PARCOURS, pas un faux zéro.** Toutes
         * les étapes à « non faites » lui annoncerait qu'il a échoué à des
         * épreuves qu'il n'a pas passées. `next` reste `null` : la page a déjà
         * un bouton « se connecter », et lui en fabriquer un second serait la
         * quatrième invitation à se connecter que S151 a retirée de la fiche
         * machine.
         */
        if (!$user instanceof Utilisateur) {
            return [
                'steps' => [],
                'next' => null,
                'quizValidated' => 0,
                'quizTotal' => 0,
                'percent' => 0,
                'badge' => $badge,
                'badgeOwned' => false,
                'machines' => $machines,
            ];
        }

        $status = $this->qualification->getStatus($formation, $user);

        $steps = [
            // Le contenu : lu et validé, ou en cours.
            [
                'key' => 'content',
                'done' => (bool) ($status['trainingProgression']?->isCompleted() ?? false),
                'blocking' => false,
            ],
            // Les quiz : tous validés, ou pas. ⚠️ Une formation SANS quiz a
            // `quizTotal = 0`, et l'étape est donc faite — pas « zéro sur zéro ».
            [
                'key' => 'quiz',
                'done' => $status['quizTotal'] === 0 || $status['quizValidated'] === $status['quizTotal'],
                'blocking' => false,
            ],
        ];

        if ($status['physicalRequired']) {
            /*
             * 🔴 **La seule étape que l'apprenant ne peut PAS faire lui-même**,
             * et c'est pour ça qu'elle est bloquante : elle attend l'équipe.
             * Depuis S180 il n'a rien à demander — finir la théorie le fait
             * apparaître dans la file des validations.
             */
            $steps[] = [
                'key' => 'practical',
                'done' => (bool) $status['physicalCompleted'],
                'blocking' => (bool) $status['theoryReady'] && !$status['physicalCompleted'],
            ];
        }

        if ($badge !== null) {
            $steps[] = ['key' => 'badge', 'done' => (bool) $status['badgeOwned'], 'blocking' => false];
        }

        $next = null;
        foreach ($steps as $step) {
            if (!$step['done']) {
                $next = $step['key'];
                break;
            }
        }

        return [
            'steps' => $steps,
            'next' => $next,
            'quizValidated' => (int) $status['quizValidated'],
            'quizTotal' => (int) $status['quizTotal'],
            'percent' => (int) $status['overallPercent'],
            'badge' => $badge,
            'badgeOwned' => (bool) $status['badgeOwned'],
            'machines' => $machines,
        ];
    }
}
