<?php

namespace App\Training;

use App\Entity\Formation;
use App\Entity\Section;
use App\Repository\SectionRepository;
use Doctrine\ORM\EntityManagerInterface;

/**
 * S181 — l'ordre des étapes d'un parcours se DÉPLACE, il ne se tape plus.
 *
 * 🔴 **Avant, chaque section portait un numéro saisi à la main.** Passer la
 * quatrième en tête voulait dire rouvrir les quatre formulaires et renuméroter,
 * et deux sections au même numéro s'ordonnaient par leur id — un ordre que
 * l'auteur ne voyait nulle part.
 *
 * ✅ Un déplacement renumérote TOUT le parcours de 1 à n, dans l'ordre affiché,
 * puis échange deux voisines. Les doublons et les trous hérités disparaissent au
 * premier geste. Les blocs de contenu de la page (`__page__…`) vivent dans la
 * même table et ne sont jamais touchés.
 *
 * ⚠️ Aucune progression n'est réécrite : l'accès à une étape se CALCULE à
 * l'affichage (la précédente est-elle réussie ?), il n'est pas enregistré.
 */
final class JourneyOrder
{
    public function __construct(
        private readonly SectionRepository $sections,
        private readonly EntityManagerInterface $entityManager,
    ) {
    }

    /**
     * @param int $delta -1 pour monter, +1 pour descendre
     *
     * @return bool faux si la section est déjà au bout, ou n'est pas une étape de ce parcours
     */
    public function move(Formation $formation, Section $section, int $delta): bool
    {
        $journey = array_values($this->sections->findJourneySections($formation));
        $index = null;
        foreach ($journey as $position => $candidate) {
            if ($candidate->getId() === $section->getId()) {
                $index = $position;
                break;
            }
        }

        $target = $index === null ? null : $index + ($delta < 0 ? -1 : 1);
        if ($target === null || $target < 0 || $target >= count($journey)) {
            return false;
        }

        [$journey[$index], $journey[$target]] = [$journey[$target], $journey[$index]];

        $this->entityManager->wrapInTransaction(static function () use ($journey): void {
            foreach ($journey as $position => $step) {
                $step->setOrdre($position + 1);
            }
        });

        return true;
    }
}
