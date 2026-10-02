<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\Formation;
use App\Entity\Quiz;
use App\Repository\FormationRepository;
use App\Repository\MachineBadgeRepository;
use App\Repository\QuestionRepository;
use App\Repository\QuizRepository;
use App\Repository\SectionRepository;
use App\Service\PlaceBadges;
use App\Training\PublishChecklist;
use Doctrine\DBAL\Connection;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * Le parcours d'une formation, en UNE liste ordonnée (proposition
 * `constructeur-formation`, 2026-10-01, d'après `lms-training-builder.png`).
 *
 * ⚠️ LECTURE SEULE et rien de neuf : mêmes dépôts que
 * `FormationContentAdminController` (`findJourneySections`,
 * `findQuizFormationsForParent`, `Quiz::getSection()`, `PublishChecklist`).
 * Seul l'ASSEMBLAGE est nouveau : sections, puis leurs quiz, puis les quiz de
 * fin, la validation pratique, le badge.
 *
 * ⚠️ Une section n'a pas de statut en base : « Publié » = elle a du contenu ou
 * une vidéo, « Brouillon » = elle est vide. Un quiz sans question = Brouillon.
 *
 * Forme d'une étape : `kind` (section|quiz|practical|badge), `icon`, `typeLabel`,
 * `title`, `meta`, `state` {label, signal}, `verb`, `route`, `params`, `nested`.
 */
final class TrainingBuilder
{
    public function __construct(
        private readonly FormationRepository $formations,
        private readonly SectionRepository $sections,
        private readonly QuizRepository $quizzes,
        private readonly QuestionRepository $questions,
        private readonly PublishChecklist $checklist,
        private readonly MachineBadgeRepository $machineBadges,
        private readonly PlaceBadges $placeBadges,
        private readonly Connection $db,
        private readonly TranslatorInterface $translator,
    ) {
    }

    /**
     * @return array{
     *   formation: Formation, choices: list<array{formation: Formation, sections: int, current: bool}>,
     *   steps: list<array<string, mixed>>, publishSteps: list<array{key: string, done: bool, blocking: bool}>,
     *   result: array{badge: ?\App\Entity\Badge, machines: list<\App\Entity\Machine>, places: list<array{id: int, nom: string}>},
     * }|null
     */
    public function build(int $requestedId): ?array
    {
        $visible = $this->formations->findVisible(['id' => 'ASC']);
        if ($visible === []) {
            return null;
        }

        $counts = [];
        foreach ($visible as $candidate) {
            $counts[(int) $candidate->getId()] = \count($this->sections->findJourneySections($candidate));
        }

        $formation = null;
        foreach ($visible as $candidate) {
            if ($candidate->getId() === $requestedId) {
                $formation = $candidate;
            }
        }
        if ($formation === null) {
            $best = -1;
            foreach ($visible as $candidate) {
                if ($counts[(int) $candidate->getId()] > $best) {
                    $best = $counts[(int) $candidate->getId()];
                    $formation = $candidate;
                }
            }
        }
        \assert($formation instanceof Formation);

        // Les liens du haut : la formation montrée, puis les plus riches.
        $ranked = $visible;
        usort($ranked, static fn (Formation $a, Formation $b): int => $counts[(int) $b->getId()] <=> $counts[(int) $a->getId()]);
        $others = \array_slice(array_values(array_filter($ranked, static fn (Formation $f): bool => $f->getId() !== $formation->getId())), 0, 3);
        $choices = [];
        foreach ([$formation, ...$others] as $f) {
            $choices[] = ['formation' => $f, 'sections' => $counts[(int) $f->getId()], 'current' => $f->getId() === $formation->getId()];
        }

        $journey = $this->sections->findJourneySections($formation);
        $quizRows = $this->quizRows($formation);

        $steps = [];
        $attachedTo = [];
        foreach ($quizRows as $row) {
            $sectionId = $row['quiz']->getSection()?->getId();
            if ($sectionId !== null) {
                $attachedTo[$sectionId][] = $row;
            }
        }
        $placed = [];
        foreach ($journey as $section) {
            $filled = trim((string) $section->getContenu()) !== '' || trim((string) $section->getVideoUrl()) !== '';
            $steps[] = [
                'kind' => 'section', 'icon' => 'view', 'typeLabel' => $this->t('type_section'), 'nested' => false,
                'title' => $section->getTitre(),
                'meta' => $section->getVideoUrl() ? $this->t('with_video') : '',
                'state' => $filled ? ['label' => $this->t('state_published'), 'signal' => 'go'] : ['label' => $this->t('state_draft'), 'signal' => 'wait'],
                'verb' => $this->t('verb_edit'), 'route' => 'app_admin_formation_section_edit',
                'params' => ['id' => $formation->getId(), 'sectionId' => $section->getId()],
            ];
            foreach ($attachedTo[$section->getId()] ?? [] as $row) {
                $steps[] = $this->quizStep($formation, $row, true);
                $placed[$row['quiz']->getId()] = true;
            }
        }
        // Les quiz sans section : à la fin du parcours, avant la pratique.
        foreach ($quizRows as $row) {
            if (!isset($placed[$row['quiz']->getId()])) {
                $steps[] = $this->quizStep($formation, $row, false);
            }
        }

        $required = $formation->getRequiresPractical();
        $physical = $this->formations->findPhysicalValidationForParent((int) $formation->getId());
        $points = $physical === null ? 0 : \count(array_filter(preg_split('/\R/', (string) $physical->getObjectifs()) ?: [], static fn (string $l): bool => trim($l) !== ''));
        $steps[] = [
            'kind' => 'practical', 'icon' => 'tool', 'typeLabel' => $this->t('type_practical'), 'nested' => false,
            'title' => $this->t('practical_title'),
            'meta' => $required === true && $points > 0 ? $this->translator->trans('training_builder.practical_points', ['%n%' => $points]) : '',
            'state' => match (true) {
                $required === null => ['label' => $this->t('state_undecided'), 'signal' => 'stop'],
                $required === false => ['label' => $this->t('state_not_required'), 'signal' => 'muted'],
                $physical === null => ['label' => $this->t('state_missing'), 'signal' => 'caution'],
                default => ['label' => $this->t('state_published'), 'signal' => 'go'],
            },
            'verb' => $required === null ? $this->t('verb_decide') : $this->t('verb_edit'), 'route' => 'app_admin_formation_content',
            'params' => ['id' => $formation->getId(), 'ouvrir' => 'practical', '_fragment' => 'practical'],
        ];

        $badge = $formation->getBadge();
        $steps[] = [
            'kind' => 'badge', 'icon' => 'key', 'typeLabel' => $this->t('type_badge'), 'nested' => false,
            'title' => $badge?->getNom() ?? $this->t('no_badge'),
            'meta' => $badge === null ? $this->t('no_badge_hint') : '',
            'state' => $badge === null ? ['label' => $this->t('state_missing'), 'signal' => 'muted'] : ['label' => $this->t('state_published'), 'signal' => 'go'],
            'verb' => $badge === null ? $this->t('verb_choose') : $this->t('verb_view_badge'),
            'route' => $badge === null ? 'app_admin_formation_content' : 'app_admin_badge_edit',
            'params' => $badge === null
                ? ['id' => $formation->getId(), 'ouvrir' => 'general', '_fragment' => 'general']
                : ['id' => $badge->getId()],
        ];

        $machines = [];
        $places = [];
        if ($badge !== null) {
            $machines = $this->machineBadges->machinesOpenedBy((int) $badge->getId());
            if ($this->placeBadges->isReady()) {
                foreach ($this->db->fetchAllAssociative(
                    'SELECT p.id, p.nom FROM PLACE_BADGE pb JOIN PLACE p ON p.id = pb.placeId WHERE pb.badgeId = ? AND p.archivedAt IS NULL ORDER BY p.nom',
                    [$badge->getId()],
                ) as $place) {
                    $places[] = ['id' => (int) $place['id'], 'nom' => (string) $place['nom']];
                }
            }
        }

        return [
            'formation' => $formation,
            'choices' => $choices,
            'steps' => $steps,
            'publishSteps' => $this->checklist->steps($formation, \count($journey), \count($quizRows)),
            'result' => ['badge' => $badge, 'machines' => $machines, 'places' => $places],
        ];
    }

    /** @return list<array{quiz: Quiz, title: string, questionCount: int}> */
    private function quizRows(Formation $formation): array
    {
        $rows = [];
        $seen = [];
        foreach ($this->formations->findQuizFormationsForParent((int) $formation->getId()) as $quizFormation) {
            foreach ($this->quizzes->findBy(['formation' => $quizFormation], ['id' => 'ASC']) as $quiz) {
                $seen[$quiz->getId()] = true;
                $rows[] = $this->quizRow($quiz);
            }
        }
        foreach ($this->quizzes->findBy(['formation' => $formation], ['id' => 'ASC']) as $quiz) {
            if (!isset($seen[$quiz->getId()])) {
                $rows[] = $this->quizRow($quiz);
            }
        }

        return $rows;
    }

    /** @return array{quiz: Quiz, title: string, questionCount: int} */
    private function quizRow(Quiz $quiz): array
    {
        return [
            'quiz' => $quiz,
            'title' => $quiz->getFormation()?->getTitre() ?: ($quiz->getSection()?->getTitre() ?: $this->t('type_quiz_plain')),
            'questionCount' => $this->questions->count(['quiz' => $quiz]),
        ];
    }

    /**
     * @param array{quiz: Quiz, title: string, questionCount: int} $row
     *
     * @return array<string, mixed>
     */
    private function quizStep(Formation $formation, array $row, bool $nested): array
    {
        $count = $row['questionCount'];

        return [
            'kind' => 'quiz', 'icon' => 'check', 'typeLabel' => $nested ? $this->t('type_quiz_section') : $this->t('type_quiz_final'), 'nested' => $nested,
            'title' => $row['title'],
            'meta' => $this->translator->trans($count > 1 ? 'training_builder.questions_many' : 'training_builder.questions_one', ['%n%' => $count]) . ' · ' . $this->translator->trans('training_builder.min_score', ['%n%' => $row['quiz']->getNoteMinimale()]),
            'state' => $count > 0 ? ['label' => $this->t('state_published'), 'signal' => 'go'] : ['label' => $this->t('state_draft'), 'signal' => 'wait'],
            'verb' => $this->t('verb_edit'), 'route' => 'app_admin_formation_quiz_edit',
            'params' => ['id' => $formation->getId(), 'quizId' => $row['quiz']->getId()],
        ];
    }

    private function t(string $key): string
    {
        return $this->translator->trans('training_builder.' . $key);
    }
}
