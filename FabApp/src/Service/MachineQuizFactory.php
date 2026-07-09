<?php

namespace App\Service;

use App\Entity\Machine;

/**
 * Construit un quiz de sécurité en mémoire lorsqu'aucun quiz n'est encore
 * enregistré pour la machine. Aucune écriture en base n'est effectuée.
 */
final class MachineQuizFactory
{
    /** @return array<string, mixed> */
    public function create(Machine $machine): array
    {
        $name = $machine->getNom();
        $category = $this->resolveCategory($machine);
        $specificQuestions = $this->categoryQuestions($category, $name);
        $rotation = max(0, (($machine->getId() ?? 1) - 1) % max(1, count($specificQuestions)));
        $specificQuestions = array_merge(
            array_slice($specificQuestions, $rotation),
            array_slice($specificQuestions, 0, $rotation),
        );

        $questions = [
            $this->single(
                'machine-' . ($machine->getId() ?? 'x') . '-1',
                1,
                sprintf('Avant d’utiliser %s, quelle vérification doit toujours être faite en premier ?', $name),
                [
                    ['Vérifier son état, la zone de travail et l’absence de signalement de panne', true],
                    ['Lancer immédiatement un cycle pour gagner du temps', false],
                    ['Modifier les réglages de maintenance', false],
                    ['Débrancher les dispositifs de sécurité', false],
                ],
            ),
            $specificQuestions[0],
            $this->multiple(
                'machine-' . ($machine->getId() ?? 'x') . '-3',
                3,
                sprintf('Quels comportements sont adaptés pendant l’utilisation de %s ?', $name),
                [
                    ['Rester attentif aux bruits, odeurs et mouvements inhabituels', true],
                    ['Respecter les protections et les consignes affichées', true],
                    ['Laisser la machine sans surveillance quand cela est interdit', false],
                    ['Contourner un capot ou un interverrouillage', false],
                ],
            ),
            $specificQuestions[1],
            $this->single(
                'machine-' . ($machine->getId() ?? 'x') . '-5',
                5,
                sprintf('Que faire si %s présente un comportement anormal ?', $name),
                [
                    ['Arrêter en sécurité, signaler le problème et prévenir un responsable', true],
                    ['Continuer jusqu’à la fin du travail', false],
                    ['Masquer le défaut pour le prochain utilisateur', false],
                    ['Réparer la machine sans autorisation', false],
                ],
            ),
            $specificQuestions[2],
            $this->multiple(
                'machine-' . ($machine->getId() ?? 'x') . '-7',
                7,
                sprintf('Quelles conditions doivent être réunies avant une utilisation autonome de %s ?', $name),
                [
                    ['Avoir suivi la formation ou reçu l’autorisation nécessaire', true],
                    ['Comprendre la procédure d’arrêt normal et d’urgence', true],
                    ['Utiliser le compte d’un autre membre', false],
                    ['Ignorer un statut de maintenance', false],
                ],
            ),
            $this->single(
                'machine-' . ($machine->getId() ?? 'x') . '-8',
                8,
                sprintf('Après avoir terminé sur %s, quelle dernière action protège les utilisateurs suivants ?', $name),
                [
                    ['Laisser le poste propre et signaler immédiatement toute anomalie', true],
                    ['Fermer la page sans vérifier la machine', false],
                    ['Masquer les défauts mineurs', false],
                    ['Laisser les consommables usagés sur le poste', false],
                ],
            ),
        ];

        return [
            'id' => 'machine-' . ($machine->getId() ?? 'x'),
            'source' => 'machine-fallback',
            'title' => 'Quiz sécurité · ' . $name,
            'subtitle' => 'Validez les règles essentielles avant votre première utilisation.',
            'eyebrow' => $machine->getCategoryLabel(),
            'passingScore' => 80,
            'questionCount' => count($questions),
            'questions' => $questions,
        ];
    }

    private function resolveCategory(Machine $machine): string
    {
        $value = mb_strtolower(implode(' ', [
            $machine->getCategorySlug(),
            $machine->getCategoryLabel(),
            $machine->getIconSlug(),
            $machine->getNom(),
        ]));

        return match (true) {
            str_contains($value, 'laser') => 'laser',
            str_contains($value, 'vinyle') || str_contains($value, 'plotter') => 'vinyle',
            str_contains($value, 'cnc') || str_contains($value, 'fraise') || str_contains($value, 'usinage') => 'cnc',
            str_contains($value, 'soud') => 'soudure',
            str_contains($value, 'oscillo') || str_contains($value, 'mesure') || str_contains($value, 'electron') => 'electronique',
            str_contains($value, 'brod') || str_contains($value, 'textile') => 'textile',
            str_contains($value, '3d') || str_contains($value, 'prusa') || str_contains($value, 'ultimaker') => 'impression-3d',
            default => 'general',
        };
    }

    /** @return list<array<string, mixed>> */
    private function categoryQuestions(string $category, string $machineName): array
    {
        return match ($category) {
            'laser' => [
                $this->single('laser-material', 2, 'Quel matériau ne doit jamais être découpé au laser sans validation explicite ?', [
                    ['Le PVC, qui peut dégager des gaz corrosifs et toxiques', true],
                    ['Le contreplaqué validé par le FabLab', false],
                    ['Le carton propre et autorisé', false],
                    ['L’acrylique compatible laser', false],
                ]),
                $this->single('laser-fire', 4, 'Pendant une découpe laser, quelle attitude est correcte ?', [
                    ['Surveiller en permanence la zone de découpe et rester près de l’arrêt', true],
                    ['Quitter la salle jusqu’à la fin du programme', false],
                    ['Ouvrir le capot pendant le tir laser', false],
                    ['Désactiver l’extraction pour réduire le bruit', false],
                ]),
                $this->multiple('laser-end', 6, 'Après utilisation de la découpeuse laser, quelles actions sont attendues ?', [
                    ['Retirer les chutes et nettoyer la zone selon les consignes', true],
                    ['Vérifier l’absence de braise ou de départ de feu', true],
                    ['Laisser les déchets inflammables dans la machine', false],
                    ['Modifier l’alignement optique sans autorisation', false],
                ]),
            ],
            'vinyle' => [
                $this->single('vinyl-blade', 2, 'Avant une découpe vinyle, que faut-il contrôler ?', [
                    ['Le réglage de lame, le support et la zone de déplacement du chariot', true],
                    ['Que la lame dépasse au maximum', false],
                    ['Que le matériau soit tenu à la main pendant la découpe', false],
                    ['Que les galets soient retirés', false],
                ]),
                $this->single('vinyl-test', 4, 'Pourquoi réaliser une petite découpe de test ?', [
                    ['Pour valider pression et profondeur sans gaspiller le support', true],
                    ['Pour chauffer le moteur', false],
                    ['Pour désactiver le repérage', false],
                    ['Pour remplacer l’échenillage', false],
                ]),
                $this->multiple('vinyl-end', 6, 'Quelles actions terminent correctement une session de découpe vinyle ?', [
                    ['Retirer le support sans forcer sur le chariot', true],
                    ['Ranger les outils d’échenillage', true],
                    ['Laisser la lame sortie et accessible', false],
                    ['Jeter les rouleaux réutilisables', false],
                ]),
            ],
            'cnc' => [
                $this->single('cnc-clamp', 2, 'Quel point est indispensable avant de démarrer un usinage CNC ?', [
                    ['Le bridage sûr de la pièce et le contrôle du parcours outil', true],
                    ['Tenir la pièce à la main', false],
                    ['Retirer le martyr', false],
                    ['Démarrer avec les portes ouvertes', false],
                ]),
                $this->multiple('cnc-ppe', 4, 'Quels contrôles sont adaptés avant l’usinage CNC ?', [
                    ['Choisir une fraise compatible et en bon état', true],
                    ['Vérifier l’origine et les limites du programme', true],
                    ['Porter des vêtements amples près des parties tournantes', false],
                    ['Retirer les protections de la machine', false],
                ]),
                $this->single('cnc-chip', 6, 'Comment retirer des copeaux après l’arrêt complet ?', [
                    ['Avec les outils prévus, jamais avec les mains près d’une arête coupante', true],
                    ['En soufflant directement avec la bouche', false],
                    ['Pendant que la broche tourne', false],
                    ['En relançant l’usinage', false],
                ]),
            ],
            'soudure' => [
                $this->single('solder-heat', 2, 'Où doit être posé le fer à souder lorsqu’il n’est pas utilisé ?', [
                    ['Dans son support stable et résistant à la chaleur', true],
                    ['Directement sur la table', false],
                    ['Sur le câble d’alimentation', false],
                    ['Dans un bac plastique', false],
                ]),
                $this->multiple('solder-smoke', 4, 'Quelles pratiques réduisent les risques au poste de soudure ?', [
                    ['Utiliser l’extraction de fumées', true],
                    ['Se laver les mains après manipulation', true],
                    ['Toucher la panne pour tester sa température', false],
                    ['Laisser le fer allumé sans surveillance', false],
                ]),
                $this->single('solder-end', 6, 'À la fin d’une session de soudure, que faut-il faire ?', [
                    ['Mettre le poste en sécurité, nettoyer et attendre le refroidissement', true],
                    ['Enrouler immédiatement le câble autour du fer chaud', false],
                    ['Jeter les résidus chauds dans une poubelle classique', false],
                    ['Laisser l’alimentation active', false],
                ]),
            ],
            'electronique' => [
                $this->single('scope-ground', 2, 'Avant de connecter une sonde de mesure, que faut-il identifier ?', [
                    ['La référence de masse et les tensions maximales admissibles', true],
                    ['Uniquement la couleur du câble', false],
                    ['Le nom du dernier utilisateur', false],
                    ['La luminosité de l’écran', false],
                ]),
                $this->multiple('scope-input', 4, 'Quelles précautions protègent le matériel et le montage ?', [
                    ['Utiliser une sonde et une atténuation adaptées', true],
                    ['Vérifier la tension attendue avant branchement', true],
                    ['Dépasser volontairement la tension d’entrée', false],
                    ['Déplacer les masses sous tension au hasard', false],
                ]),
                $this->single('scope-end', 6, 'Comment terminer proprement une mesure ?', [
                    ['Mettre le montage en sécurité puis ranger sondes et accessoires', true],
                    ['Tirer sur les câbles pour les retirer', false],
                    ['Laisser les sondes en court-circuit sur le montage', false],
                    ['Forcer les connecteurs', false],
                ]),
            ],
            'textile' => [
                $this->single('textile-needle', 2, 'Avant de lancer une broderie, que faut-il vérifier ?', [
                    ['L’aiguille, le cadre, le fil et la fixation du textile', true],
                    ['Que les mains restent sous l’aiguille', false],
                    ['Que le cadre puisse se déplacer contre un obstacle', false],
                    ['Que le capot soit neutralisé', false],
                ]),
                $this->single('textile-zone', 4, 'Pendant le mouvement de la brodeuse, où placer les mains ?', [
                    ['En dehors de la zone mobile et du cadre', true],
                    ['Sur le textile pour le guider', false],
                    ['Près de l’aiguille pour couper le fil', false],
                    ['Sous le cadre', false],
                ]),
                $this->multiple('textile-end', 6, 'Après une broderie, quelles actions sont correctes ?', [
                    ['Arrêter la machine avant de retirer le cadre', true],
                    ['Ramasser fils et chutes', true],
                    ['Forcer sur l’aiguille si le cadre bloque', false],
                    ['Laisser une aiguille cassée sur le poste', false],
                ]),
            ],
            'impression-3d' => [
                $this->single('print-bed', 2, 'Quel risque doit être pris en compte autour d’une imprimante 3D en fonctionnement ?', [
                    ['Les éléments chauds et les axes en mouvement', true],
                    ['Uniquement le bruit du ventilateur', false],
                    ['La couleur du filament', false],
                    ['Le nom du fichier', false],
                ]),
                $this->multiple('print-start', 4, sprintf('Avant de lancer une impression sur %s, que faut-il contrôler ?', $machineName), [
                    ['Le matériau et le profil de tranchage', true],
                    ['L’adhérence et la propreté du plateau', true],
                    ['La présence d’un objet dans la zone mobile', false],
                    ['Le retrait des protections de la machine', false],
                ]),
                $this->single('print-end', 6, 'Quand peut-on retirer une pièce du plateau ?', [
                    ['Après la fin du cycle et lorsque le plateau permet une manipulation sûre', true],
                    ['Pendant les premiers mouvements', false],
                    ['En tirant sur les axes', false],
                    ['Avant l’arrêt du chauffage', false],
                ]),
            ],
            default => [
                $this->single('general-zone', 2, 'Pourquoi faut-il garder la zone de travail dégagée ?', [
                    ['Pour éviter les chutes, collisions et interférences avec la machine', true],
                    ['Pour augmenter automatiquement la puissance', false],
                    ['Pour supprimer les contrôles de sécurité', false],
                    ['Pour travailler sans consigne', false],
                ]),
                $this->single('general-training', 4, 'Que faire si une étape d’utilisation n’est pas comprise ?', [
                    ['Arrêter et demander l’aide d’un membre du staff', true],
                    ['Essayer plusieurs commandes au hasard', false],
                    ['Contourner la procédure', false],
                    ['Utiliser le compte d’un autre membre', false],
                ]),
                $this->multiple('general-end', 6, 'Quelles actions sont attendues après utilisation ?', [
                    ['Nettoyer et ranger la zone', true],
                    ['Signaler toute anomalie observée', true],
                    ['Laisser les consommables et déchets sur place', false],
                    ['Cacher les incidents', false],
                ]),
            ],
        };
    }

    /** @param list<array{0: string, 1: bool}> $choices */
    private function single(string $id, int $order, string $text, array $choices): array
    {
        return $this->question($id, $order, $text, 'single', $choices);
    }

    /** @param list<array{0: string, 1: bool}> $choices */
    private function multiple(string $id, int $order, string $text, array $choices): array
    {
        return $this->question($id, $order, $text, 'multiple', $choices);
    }

    /** @param list<array{0: string, 1: bool}> $choices */
    private function question(string $id, int $order, string $text, string $type, array $choices): array
    {
        return [
            'id' => $id,
            'order' => $order,
            'text' => $text,
            'type' => $type,
            'choices' => array_map(
                static fn (array $choice, int $index): array => [
                    'id' => $id . '-choice-' . ($index + 1),
                    'text' => $choice[0],
                    'correct' => $choice[1],
                ],
                $choices,
                array_keys($choices),
            ),
        ];
    }
}
