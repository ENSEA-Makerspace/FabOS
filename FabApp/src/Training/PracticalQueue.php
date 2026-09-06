<?php

namespace App\Training;

use App\Entity\Formation;
use App\Entity\Utilisateur;
use App\Repository\FormationRepository;
use App\Repository\UtilisateurRepository;
use App\Service\TrainingQualificationService;

/**
 * Qui attend une validation pratique, en ce moment (S180).
 *
 * 🔴 **Le chaînon qui manquait entre « quiz réussi » et « badge ».** Le geste de
 * validation existait — un membre du staff ouvre la fiche d'un membre et clique
 * — mais **rien ne disait QUI ouvrir**. Un apprenant qui avait fini la théorie
 * restait donc bloqué jusqu'à ce qu'il pense à le demander de vive voix, et
 * l'équipe n'avait aucun moyen de voir combien de gens attendaient.
 *
 * ✅ **Et il n'y a RIEN à demander.** La feuille de route prévoyait un bouton
 * « demander une évaluation » et un enregistrement de demande. Ce serait une
 * seconde source de vérité sur qui est prêt : le jour où quelqu'un finit la
 * théorie sans cliquer, il n'existe pour personne. **Avoir fini la théorie EST
 * la demande** — la file se déduit, elle ne se remplit pas. Même raisonnement
 * qu'en S178 : ce qui n'est pas accordé n'a pas à être révoqué.
 *
 * 🅿️ **Ce que ce choix perd, et il faut le dire** : un apprenant ne peut pas
 * signaler qu'il est disponible à un MOMENT donné. C'est de la prise de
 * rendez-vous, pas de la qualification, et ça vit dans les créneaux — pas ici.
 *
 * ⚠️ **Le coût est réel et assumé** : une passe sur les formations visibles ×
 * les membres. À l'échelle d'un FabLab (quelques dizaines de formations,
 * quelques centaines de membres) c'est une page d'administration consultée
 * quelques fois par jour. Le jour où ça pique, la requête se remonte en SQL —
 * mais optimiser avant d'avoir mesuré aurait produit une requête illisible pour
 * une file qui compte trois lignes.
 */
final class PracticalQueue
{
    public function __construct(
        private readonly FormationRepository $formations,
        private readonly UtilisateurRepository $users,
        private readonly TrainingQualificationService $qualification,
    ) {
    }

    /**
     * @return list<array{user: Utilisateur, formation: Formation, physical: Formation, since: ?\DateTimeImmutable, percent: int}>
     *         Trié du plus ancien au plus récent : c'est une FILE, et celui qui
     *         attend depuis le plus longtemps passe devant. Trier par nom
     *         donnerait un ordre stable et injuste.
     */
    public function pending(): array
    {
        $rows = [];
        $members = $this->users->findBy([], ['lastName' => 'ASC']);

        foreach ($this->formations->findVisible(['titre' => 'ASC']) as $formation) {
            // ⚠️ Sans badge à la clé, une validation pratique n'ouvre rien : la
            // file ne sert qu'aux formations qui débouchent sur un accès.
            if ($formation->getBadge() === null) {
                continue;
            }

            foreach ($members as $user) {
                $status = $this->qualification->getStatus($formation, $user);

                if (!$status['physicalRequired'] || !($status['physicalFormation'] instanceof Formation)) {
                    continue;
                }
                // 🔴 La condition de la file, en une ligne : la théorie est
                // finie, la pratique ne l'est pas. Ni « presque » ni « en
                // cours » — les deux extrémités du chaînon manquant.
                if (!$status['theoryReady'] || $status['physicalCompleted']) {
                    continue;
                }

                $rows[] = [
                    'user' => $user,
                    'formation' => $formation,
                    'physical' => $status['physicalFormation'],
                    // Depuis quand : la fin de la théorie, à défaut son début.
                    'since' => $status['trainingProgression']?->getDateEnd()
                        ?? $status['trainingProgression']?->getDateDebut(),
                    'percent' => (int) $status['overallPercent'],
                ];
            }
        }

        usort($rows, static function (array $a, array $b): int {
            // ⚠️ Une date inconnue passe EN TÊTE, pas en queue : c'est le cas
            // d'une progression écrite avant que la colonne existe, et la
            // reléguer en bas la ferait oublier indéfiniment.
            $left = $a['since']?->getTimestamp() ?? 0;
            $right = $b['since']?->getTimestamp() ?? 0;

            return $left <=> $right;
        });

        return $rows;
    }
}
