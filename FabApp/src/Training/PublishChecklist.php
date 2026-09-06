<?php

namespace App\Training;

use App\Entity\Formation;

/**
 * Cette formation est-elle prête à être publiée ? (S181)
 *
 * 🔴 **Ce que l'écran de contenu ne disait pas.** Neuf replis, chacun avec son
 * formulaire, et aucun endroit qui réponde à la question qu'on se pose en
 * arrivant : **est-ce que ça tient debout ?** L'auteur devait ouvrir les neuf
 * cartes pour découvrir qu'il manquait un quiz — ou pire, ne pas le découvrir et
 * publier un parcours qui ne mène à rien.
 *
 * ✅ **Rien de neuf n'est calculé, et rien de neuf n'est dessiné.** Les comptes
 * existent déjà dans le contrôleur ; le rendu est `_commissioning`, écrit en S176
 * pour la mise en service d'un boîtier et déjà réemployé en S179 pour le parcours
 * d'un apprenant. C'est sa TROISIÈME utilisation, sur trois sujets sans rapport —
 * la définition d'un composant.
 *
 * 🔴 **Une seule étape est BLOQUANTE, et ce n'est pas celle qu'on croit.** Ce
 * n'est ni le quiz ni le badge : c'est **l'exigence de validation pratique
 * laissée à `null`**. Tant qu'elle l'est, la question « faut-il une évaluation
 * sur la machine ? » est encore tranchée par une correspondance de chaîne sur le
 * titre (S180b) — une garde de sécurité décidée par un mot-clé français. Publier
 * dans cet état, c'est publier une formation dont personne n'a validé le niveau
 * de risque.
 *
 * ⚠️ **Une formation SANS badge n'est pas un défaut.** Un cours purement
 * informatif n'ouvre rien et n'a pas à le faire. L'étape le dit et ne bloque pas.
 */
final class PublishChecklist
{
    /**
     * @param int $sectionCount le nombre de sections du parcours
     * @param int $quizCount    le nombre de quiz rattachés
     *
     * @return list<array{key: string, done: bool, blocking: bool}>
     */
    public function steps(Formation $formation, int $sectionCount, int $quizCount): array
    {
        return [
            [
                'key' => 'described',
                'done' => trim((string) $formation->getDescription()) !== '',
                'blocking' => false,
            ],
            [
                'key' => 'sections',
                'done' => $sectionCount > 0,
                'blocking' => false,
            ],
            [
                // ⚠️ Un quiz n'est pas obligatoire pour tout : une formation peut
                // se valider à la pratique seule. L'étape informe, elle ne bloque
                // pas — bloquer ici forcerait à inventer un quiz vide.
                'key' => 'quiz',
                'done' => $quizCount > 0,
                'blocking' => false,
            ],
            [
                'key' => 'badge',
                'done' => $formation->getBadge() !== null,
                'blocking' => false,
            ],
            [
                // 🔴 La seule bloquante : tant que c'est `null`, une garde de
                // SÉCURITÉ est décidée par un mot-clé dans le titre.
                'key' => 'risk',
                'done' => $formation->getRequiresPractical() !== null,
                'blocking' => $formation->getRequiresPractical() === null,
            ],
        ];
    }
}
