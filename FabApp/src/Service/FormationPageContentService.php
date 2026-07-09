<?php

namespace App\Service;

use App\Entity\Formation;
use App\Entity\Section;
use App\Repository\SectionRepository;
use Doctrine\ORM\EntityManagerInterface;

final class FormationPageContentService
{
    /** @var array<string, mixed> */
    private const DEFAULTS = [
        'labels' => [
            'descriptionTitle' => 'Description détaillée',
            'objectivesTitle' => 'Objectifs pédagogiques',
            'prerequisitesTitle' => 'Prérequis',
            'materialTitle' => 'Matériel fourni',
        ],
        'journey' => [
            'kicker' => 'Parcours guidé',
            'title' => 'Un parcours progressif pour apprendre vraiment la machine',
            'intro' => 'Le suivi est pensé comme une montée en autonomie : vous découvrez une étape, vous retenez les gestes importants, puis vous validez vos acquis avec un mini-quiz. Chaque réussite déverrouille automatiquement la suite et met à jour votre progression sans bouton final à cliquer.',
            'cards' => [
                [
                    'title' => 'Comment avancer',
                    'text' => 'Ouvrez la première section disponible, lisez les explications, parcourez les gestes clés puis terminez par le mini-quiz de validation. Tant qu’une section n’est pas réussie, la suivante reste verrouillée.',
                ],
                [
                    'title' => 'Ce qui compte dans la progression',
                    'text' => 'Les sections guidées et les quiz obligatoires contribuent à la progression globale. Les quiz bonus servent à s’entraîner, mais n’augmentent pas le pourcentage affiché.',
                ],
                [
                    'title' => 'Validation automatique',
                    'text' => 'La formation se valide automatiquement quand toutes les sections et les quiz obligatoires sont complétés.',
                ],
            ],
        ],
        'program' => [
            'title' => 'Programme horaire',
            'items' => [
                ['time' => '00:00', 'title' => 'Accueil et sécurité', 'description' => 'Présentation des risques, EPI, consignes atelier et bonnes pratiques.'],
                ['time' => '00:30', 'title' => 'Préparation du projet', 'description' => 'Choix du fichier, réglages principaux et validation avec un encadrant.'],
                ['time' => '01:15', 'title' => 'Démonstration machine', 'description' => 'Lancement, surveillance, arrêt et résolution des incidents fréquents.'],
                ['time' => '02:00', 'title' => 'Mise en pratique', 'description' => 'Production accompagnée et validation des acquis pour l’usage autonome.'],
            ],
        ],
        'sessions' => [
            'title' => 'Prochaines sessions',
            'items' => [
                ['date' => 'Mardi prochain', 'time' => '14:00 - 16:30', 'status' => 'available', 'label' => 'Places disponibles'],
                ['date' => 'Jeudi prochain', 'time' => '10:00 - 12:30', 'status' => 'available', 'label' => 'Places disponibles'],
                ['date' => 'Vendredi prochain', 'time' => '15:00 - 17:30', 'status' => 'full', 'label' => 'Complet'],
            ],
        ],
        'practical' => [
            'title' => 'Formation pratique',
            'requiredLabel' => 'Validation pratique nécessaire',
            'requiredDescription' => 'Une séance pratique en présentiel reste nécessaire avant l’habilitation finale.',
            'requiredStatus' => 'Présentiel à valider',
            'optionalLabel' => 'Formation pratique non nécessaire',
            'optionalDescription' => 'Le parcours en ligne suffit pour finaliser cette formation.',
            'optionalStatus' => 'Parcours en ligne suffisant',
        ],
        'related' => [
            'title' => 'Formations similaires',
            'items' => [
                [
                    'badge' => 'Catalogue',
                    'title' => 'Parcours machines FabLab',
                    'description' => 'Retrouvez les autres formations disponibles dans le catalogue connecté à MariaDB.',
                    'button' => 'Voir les formations',
                ],
                [
                    'badge' => 'Suivi',
                    'title' => 'Continuer cette formation',
                    'description' => 'Consultez les sections, quiz et progressions déjà branchés pour cette formation.',
                    'button' => 'Ouvrir le suivi',
                ],
            ],
        ],
    ];

    public function __construct(
        private readonly EntityManagerInterface $entityManager,
        private readonly SectionRepository $sections,
    ) {
    }

    /** @return array<string, mixed> */
    public function getContent(Formation $formation): array
    {
        $content = self::DEFAULTS;

        foreach ($this->sections->findPageContentBlocks($formation) as $section) {
            $key = substr($section->getTitre(), strlen(SectionRepository::PAGE_BLOCK_PREFIX));
            if (!array_key_exists($key, self::DEFAULTS)) {
                continue;
            }

            $decoded = $this->decode($section->getContenu());
            if ($decoded === null) {
                continue;
            }

            $content[$key] = $this->mergeRecursive(self::DEFAULTS[$key], $decoded);
        }

        $content['lists'] = [
            'objectives' => $this->splitList($formation->getObjectifs(), ['.', "\n"]),
            'prerequisites' => $this->splitList($formation->getPrerequis(), ["\n", '.']),
            'material' => $this->splitList($formation->getMaterielFourni(), [',', "\n"]),
        ];

        return $content;
    }

    /** @param array<string, mixed> $payload */
    public function saveBlock(Formation $formation, string $key, array $payload): void
    {
        if (!array_key_exists($key, self::DEFAULTS)) {
            throw new \InvalidArgumentException('Bloc de contenu inconnu.');
        }

        $section = $this->sections->findPageContentBlock($formation, $key);
        if (!$section instanceof Section) {
            $section = (new Section())
                ->setFormation($formation)
                ->setTitre(SectionRepository::PAGE_BLOCK_PREFIX . $key)
                ->setOrdre(-1000 + array_search($key, array_keys(self::DEFAULTS), true));
            $this->entityManager->persist($section);
        }

        $section->setContenu(json_encode($payload, JSON_UNESCAPED_UNICODE | JSON_UNESCAPED_SLASHES | JSON_THROW_ON_ERROR));
        $this->entityManager->flush();
    }

    /** @return array<string, mixed> */
    public function getDefaults(): array
    {
        return self::DEFAULTS;
    }

    /** @return list<string> */
    private function splitList(?string $value, array $delimiters): array
    {
        $value = trim((string) $value);
        if ($value === '') {
            return [];
        }

        $patternParts = array_map(static fn (string $delimiter): string => preg_quote($delimiter, '/'), $delimiters);
        $parts = preg_split('/(?:' . implode('|', $patternParts) . ')+/u', $value) ?: [];

        return array_values(array_filter(array_map(
            static fn (string $item): string => trim($item),
            $parts,
        ), static fn (string $item): bool => $item !== ''));
    }

    /** @return array<string, mixed>|null */
    private function decode(?string $value): ?array
    {
        if (!is_string($value) || trim($value) === '') {
            return null;
        }

        try {
            $decoded = json_decode($value, true, 512, JSON_THROW_ON_ERROR);
        } catch (\JsonException) {
            return null;
        }

        return is_array($decoded) ? $decoded : null;
    }

    private function mergeRecursive(mixed $defaults, mixed $stored): mixed
    {
        if (!is_array($defaults) || !is_array($stored)) {
            return $stored;
        }

        if (array_is_list($defaults)) {
            return array_is_list($stored) ? $stored : $defaults;
        }

        $result = $defaults;
        foreach ($stored as $key => $value) {
            if (array_key_exists($key, $defaults)) {
                $result[$key] = $this->mergeRecursive($defaults[$key], $value);
            }
        }

        return $result;
    }
}
