<?php

namespace App\Service;

use App\Entity\Formation;
use App\Entity\Progression;
use App\Entity\Quiz;
use App\Entity\Section;
use App\Entity\Utilisateur;
use App\Repository\ChoixRepository;
use App\Repository\FormationRepository;
use App\Repository\ProgressionRepository;
use App\Repository\QuestionRepository;
use App\Repository\QuizRepository;
use App\Repository\SectionRepository;
use Doctrine\ORM\EntityManagerInterface;

final class GuidedTrainingService
{
    public function __construct(
        private readonly EntityManagerInterface $entityManager,
        private readonly FormationRepository $formations,
        private readonly SectionRepository $sections,
        private readonly QuizRepository $quizzes,
        private readonly QuestionRepository $questions,
        private readonly ChoixRepository $choices,
        private readonly ProgressionRepository $progressions,
        private readonly QuizCatalogService $catalog,
    ) {
    }

    /**
     * @return array{
     *   configured:bool,
     *   total:int,
     *   completed:int,
     *   started:int,
     *   score:int,
     *   completedAll:bool,
     *   progression:?Progression,
     *   sectionResults:array<int, ?Progression>
     * }
     */
    public function getProgress(Formation $parentFormation, Utilisateur $user): array
    {
        $parentFormation = $this->resolveVisibleParent($parentFormation);
        $sectionRows = $this->sections->findJourneySections($parentFormation);
        $quizFormationMap = $this->getSectionQuizFormationMap($parentFormation);

        $completed = 0;
        $started = 0;
        $sectionResults = [];

        foreach ($sectionRows as $section) {
            $sectionId = $section->getId();
            if ($sectionId === null) {
                continue;
            }

            $result = null;
            $quizFormation = $quizFormationMap[$sectionId] ?? null;
            if ($quizFormation instanceof Formation) {
                $result = $this->progressions->findOneBy([
                    'utilisateur' => $user,
                    'formation' => $quizFormation,
                ]);
            }

            $sectionResults[$sectionId] = $result;
            if ($result instanceof Progression) {
                ++$started;
                if ($result->isCompleted()) {
                    ++$completed;
                }
            }
        }

        $total = count($sectionRows);
        $configured = $total > 0;
        foreach ($sectionRows as $section) {
            $sectionId = $section->getId();
            if ($sectionId === null || !isset($quizFormationMap[$sectionId])) {
                $configured = false;
                break;
            }
        }
        $score = $configured && $total > 0
            ? (int) round(($completed / $total) * 100)
            : 0;
        $completedAll = $configured && $total > 0 && $completed === $total;

        return [
            'configured' => $configured,
            'total' => $total,
            'completed' => $completed,
            'started' => $started,
            'score' => $score,
            'completedAll' => $completedAll,
            'progression' => $this->progressions->findOneBy([
                'utilisateur' => $user,
                'formation' => $parentFormation,
            ]),
            'sectionResults' => $sectionResults,
        ];
    }

    /**
     * @return array{
     *   configured:bool,
     *   total:int,
     *   completed:int,
     *   started:int,
     *   score:int,
     *   completedAll:bool,
     *   overallScore?:int,
     *   pathCompleted?:bool,
     *   requiredQuizTotal?:int,
     *   requiredQuizValidated?:int,
     *   progression:?Progression,
     *   sectionResults:array<int, ?Progression>
     * }
     */
    public function synchronizeParentProgress(
        Formation $parentFormation,
        Utilisateur $user,
        bool $createIfMissing = false,
    ): array {
        $parentFormation = $this->resolveVisibleParent($parentFormation);
        $status = $this->getProgress($parentFormation, $user);
        if (!$status['configured']) {
            return $status;
        }

        $requiredQuizTotal = 0;
        $requiredQuizValidated = 0;
        $parentId = $parentFormation->getId();
        if ($parentId !== null) {
            foreach ($this->formations->findQuizFormationsForParent($parentId) as $quizFormation) {
                if (!$this->catalog->isRequiredPageQuizFormation($quizFormation)) {
                    continue;
                }

                ++$requiredQuizTotal;
                $quizProgression = $this->progressions->findOneBy([
                    'utilisateur' => $user,
                    'formation' => $quizFormation,
                ]);
                if ($quizProgression instanceof Progression && $quizProgression->isCompleted()) {
                    ++$requiredQuizValidated;
                }
            }
        }

        $quizPercent = $requiredQuizTotal > 0
            ? (int) round(($requiredQuizValidated / $requiredQuizTotal) * 100)
            : 100;
        $trainingWeight = $requiredQuizTotal > 0 ? 60 : 100;
        $quizWeight = $requiredQuizTotal > 0 ? 40 : 0;
        $overallScore = (int) round(
            ($status['score'] / 100) * $trainingWeight
            + ($quizPercent / 100) * $quizWeight,
        );
        $pathCompleted = $status['completedAll']
            && ($requiredQuizTotal === 0 || $requiredQuizValidated === $requiredQuizTotal);

        $status['overallScore'] = $overallScore;
        $status['pathCompleted'] = $pathCompleted;
        $status['requiredQuizTotal'] = $requiredQuizTotal;
        $status['requiredQuizValidated'] = $requiredQuizValidated;

        $progression = $status['progression'];
        $hasAnyProgress = $status['started'] > 0 || $requiredQuizValidated > 0;
        if (!$progression instanceof Progression && !$createIfMissing && !$hasAnyProgress) {
            return $status;
        }

        $now = new \DateTimeImmutable();
        if (!$progression instanceof Progression) {
            $progression = (new Progression())
                ->setUtilisateur($user)
                ->setFormation($parentFormation);
            $this->entityManager->persist($progression);
        }

        $dateEnd = null;
        if ($pathCompleted) {
            if ($progression->isCompleted() && $progression->getDateEnd() !== null) {
                $dateEnd = $progression->getDateEnd();
            } else {
                $minimumDateEnd = $progression->getDateDebut()->modify('+1 second');
                $dateEnd = $now > $minimumDateEnd ? $now : $minimumDateEnd;
            }
        }

        $progression
            ->setScore($overallScore)
            ->setCompleted($pathCompleted)
            ->setDateEnd($dateEnd);

        $this->entityManager->flush();
        $status['progression'] = $progression;

        return $status;
    }

    public function assertQuizUnlocked(Quiz $quiz, Utilisateur $user): void
    {
        $quizFormation = $quiz->getFormation();
        if (!$this->catalog->isSectionQuizFormation($quizFormation)) {
            return;
        }

        $parentId = $this->catalog->getParentFormationId($quizFormation);
        $sectionId = $this->catalog->getParentSectionId($quizFormation);
        if ($parentId === null || $sectionId === null) {
            throw new \LogicException('La validation de cette section est incomplètement configurée.');
        }

        $parent = $this->formations->find($parentId);
        $currentSection = $this->sections->find($sectionId);
        if (!$parent instanceof Formation || !$currentSection instanceof Section || $currentSection->getFormation()?->getId() !== $parentId) {
            throw new \LogicException('Cette section ne correspond pas à la formation demandée.');
        }

        $quizFormationMap = $this->getSectionQuizFormationMap($parent);
        foreach ($this->sections->findJourneySections($parent) as $section) {
            if ($section->getId() === $sectionId) {
                return;
            }

            $requiredFormation = $section->getId() !== null ? ($quizFormationMap[$section->getId()] ?? null) : null;
            if (!$requiredFormation instanceof Formation) {
                throw new \LogicException('La section précédente ne possède pas encore de mini-quiz de validation.');
            }

            $previousResult = $this->progressions->findOneBy([
                'utilisateur' => $user,
                'formation' => $requiredFormation,
            ]);
            if (!$previousResult instanceof Progression || !$previousResult->isCompleted()) {
                throw new \LogicException('Validez la section précédente avant de continuer.');
            }
        }

        throw new \LogicException('Section introuvable dans le parcours de cette formation.');
    }

    /** @return array<int, Formation> indexed by visible SECTION id */
    public function getSectionQuizFormationMap(Formation $parentFormation): array
    {
        $parentFormation = $this->resolveVisibleParent($parentFormation);
        $parentId = $parentFormation->getId();
        if ($parentId === null) {
            return [];
        }

        $map = [];
        foreach ($this->formations->findQuizFormationsForParent($parentId) as $quizFormation) {
            if (!$this->catalog->isSectionQuizFormation($quizFormation)) {
                continue;
            }

            $sectionId = $this->catalog->getParentSectionId($quizFormation);
            if ($sectionId !== null && !isset($map[$sectionId])) {
                $map[$sectionId] = $quizFormation;
            }
        }

        return $map;
    }

    /**
     * @return array{
     *   configured:bool,
     *   total:int,
     *   completed:int,
     *   percent:int,
     *   items:list<array<string,mixed>>
     * }
     */
    public function buildJourney(Formation $parentFormation, ?Utilisateur $user): array
    {
        $parentFormation = $this->resolveVisibleParent($parentFormation);
        $sectionRows = $this->sections->findJourneySections($parentFormation);
        $quizFormationMap = $this->getSectionQuizFormationMap($parentFormation);
        $guestConfigured = $sectionRows !== [];
        foreach ($sectionRows as $section) {
            $sectionId = $section->getId();
            if ($sectionId === null || !isset($quizFormationMap[$sectionId])) {
                $guestConfigured = false;
                break;
            }
        }

        $progress = $user instanceof Utilisateur
            ? $this->getProgress($parentFormation, $user)
            : [
                'configured' => $guestConfigured,
                'total' => count($sectionRows),
                'completed' => 0,
                'sectionResults' => [],
            ];

        $items = [];
        $previousCompleted = true;
        $activeAssigned = false;

        foreach ($sectionRows as $index => $section) {
            $sectionId = $section->getId();
            if ($sectionId === null) {
                continue;
            }

            $quizFormation = $quizFormationMap[$sectionId] ?? null;
            $quiz = $quizFormation instanceof Formation
                ? $this->quizzes->findOneBy(['formation' => $quizFormation], ['id' => 'ASC'])
                : null;
            $result = $progress['sectionResults'][$sectionId] ?? null;
            $completed = $result instanceof Progression && $result->isCompleted();
            $unlocked = $index === 0 || $previousCompleted;
            $active = false;

            if (!$activeAssigned && $unlocked && !$completed) {
                $active = true;
                $activeAssigned = true;
            }

            $items[] = [
                'section' => $section,
                'content' => $this->parseContent($section->getContenu()),
                'quiz' => $quiz,
                'questions' => $quiz instanceof Quiz ? $this->buildQuestionData($quiz) : [],
                'result' => $result,
                'completed' => $completed,
                'unlocked' => $unlocked,
                'active' => $active,
                'status' => $completed ? 'completed' : ($unlocked ? 'available' : 'locked'),
                'position' => $index + 1,
            ];

            $previousCompleted = $completed;
        }

        if (!$activeAssigned && $items !== []) {
            $lastIndex = array_key_last($items);
            $items[$lastIndex]['active'] = true;
        }

        $total = count($sectionRows);
        $completed = (int) ($progress['completed'] ?? 0);

        return [
            'configured' => (bool) ($progress['configured'] ?? false),
            'total' => $total,
            'completed' => $completed,
            'percent' => $total > 0 ? (int) round(($completed / $total) * 100) : 0,
            'items' => $items,
        ];
    }

    /** @return list<array{id:int,text:string,type:string,choices:list<array{id:int,text:string}>}> */
    private function buildQuestionData(Quiz $quiz): array
    {
        $rows = [];
        foreach ($this->questions->findBy(['quiz' => $quiz], ['ordre' => 'ASC', 'id' => 'ASC']) as $question) {
            $choices = [];
            $correctCount = 0;
            foreach ($this->choices->findBy(['question' => $question], ['ordre' => 'ASC', 'id' => 'ASC']) as $choice) {
                if ($choice->isEstCorrect()) {
                    ++$correctCount;
                }
                $choices[] = [
                    'id' => (int) $choice->getId(),
                    'text' => $choice->getTexte(),
                ];
            }

            if ($choices === []) {
                continue;
            }

            $rows[] = [
                'id' => (int) $question->getId(),
                'text' => $question->getTexte(),
                'type' => $correctCount > 1 ? 'multiple' : 'single',
                'choices' => $choices,
            ];
        }

        return $rows;
    }

    /** @return array{intro:string,objectives:list<string>,steps:list<array{title:string,text:string}>,callouts:list<array{type:string,title:string,text:string}>} */
    private function parseContent(?string $content): array
    {
        $fallback = [
            'intro' => trim((string) $content),
            'objectives' => [],
            'steps' => [],
            'callouts' => [],
        ];

        if (!is_string($content) || trim($content) === '') {
            $fallback['intro'] = 'Le contenu de cette étape sera bientôt disponible.';
            return $fallback;
        }

        try {
            $decoded = json_decode($content, true, 512, JSON_THROW_ON_ERROR);
        } catch (\JsonException) {
            return $fallback;
        }

        if (!is_array($decoded)) {
            return $fallback;
        }

        $intro = trim((string) ($decoded['intro'] ?? ''));
        $objectives = [];
        foreach (($decoded['objectives'] ?? []) as $objective) {
            $objective = trim((string) $objective);
            if ($objective !== '') {
                $objectives[] = $objective;
            }
        }

        $steps = [];
        foreach (($decoded['steps'] ?? []) as $step) {
            if (!is_array($step)) {
                continue;
            }
            $title = trim((string) ($step['title'] ?? ''));
            $text = trim((string) ($step['text'] ?? ''));
            if ($title !== '' || $text !== '') {
                $steps[] = ['title' => $title, 'text' => $text];
            }
        }

        $callouts = [];
        foreach (($decoded['callouts'] ?? []) as $callout) {
            if (!is_array($callout)) {
                continue;
            }
            $type = strtolower(trim((string) ($callout['type'] ?? 'tip')));
            if (!in_array($type, ['tip', 'warning', 'check', 'info'], true)) {
                $type = 'tip';
            }
            $title = trim((string) ($callout['title'] ?? ''));
            $text = trim((string) ($callout['text'] ?? ''));
            if ($title !== '' || $text !== '') {
                $callouts[] = ['type' => $type, 'title' => $title, 'text' => $text];
            }
        }

        return [
            'intro' => $intro !== '' ? $intro : $fallback['intro'],
            'objectives' => $objectives,
            'steps' => $steps,
            'callouts' => $callouts,
        ];
    }

    private function resolveVisibleParent(Formation $formation): Formation
    {
        if (!$this->catalog->isInternalQuizFormation($formation)) {
            return $formation;
        }

        $parentId = $this->catalog->getParentFormationId($formation);
        $parent = $parentId !== null ? $this->formations->find($parentId) : null;

        return $parent instanceof Formation ? $parent : $formation;
    }
}
