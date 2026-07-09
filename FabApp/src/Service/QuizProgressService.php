<?php

namespace App\Service;

use App\Entity\Progression;
use App\Entity\Quiz;
use App\Entity\Utilisateur;
use App\Repository\ChoixRepository;
use App\Repository\ProgressionRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Repository\QuestionRepository;
use Doctrine\ORM\EntityManagerInterface;

final class QuizProgressService
{
    public function __construct(
        private readonly EntityManagerInterface $entityManager,
        private readonly ProgressionRepository $progressions,
        private readonly QuestionRepository $questions,
        private readonly ChoixRepository $choices,
        private readonly QuizCatalogService $catalog,
        private readonly GuidedTrainingService $guidedTraining,
        private readonly TrainingQualificationService $qualification,
        private readonly UtilisateurBadgeRepository $userBadges,
    ) {
    }

    /**
     * @param array<string, mixed> $submittedAnswers
     * @return array{
     *   attemptScore:int,
     *   bestScore:int,
     *   correctCount:int,
     *   questionCount:int,
     *   passed:bool,
     *   completed:bool,
     *   badgeAwarded:bool,
     *   badge:?array{id:int,nom:string,description:?string,icone:?string},
     *   qualification:array{overallPercent:int,theoryReady:bool,pathCompleted:bool,physicalCompleted:bool,eligible:bool}
     * }
     */
    public function saveResult(Quiz $quiz, Utilisateur $user, array $submittedAnswers): array
    {
        $resultFormation = $quiz->getFormation();
        if (!$this->catalog->isInternalQuizFormation($resultFormation)) {
            throw new \LogicException('Ce quiz doit être initialisé avec le script de données avant de pouvoir enregistrer un résultat.');
        }

        $this->guidedTraining->assertQuizUnlocked($quiz, $user);

        $questionRows = $this->questions->findBy(['quiz' => $quiz], ['ordre' => 'ASC']);
        if ($questionRows === []) {
            throw new \InvalidArgumentException('Ce quiz ne contient aucune question enregistrable.');
        }

        $correctCount = 0;
        $questionCount = 0;
        foreach ($questionRows as $question) {
            $choiceRows = $this->choices->findBy(['question' => $question], ['ordre' => 'ASC']);
            if ($choiceRows === []) {
                continue;
            }

            ++$questionCount;
            $expected = [];
            $allowed = [];

            foreach ($choiceRows as $choice) {
                $id = (string) $choice->getId();
                $allowed[$id] = true;
                if ($choice->isEstCorrect()) {
                    $expected[] = $id;
                }
            }

            $answer = $submittedAnswers[(string) $question->getId()] ?? [];
            if (!is_array($answer)) {
                $answer = [$answer];
            }

            $actual = [];
            foreach ($answer as $choiceId) {
                $choiceId = (string) $choiceId;
                if (isset($allowed[$choiceId])) {
                    $actual[] = $choiceId;
                }
            }

            $expected = array_values(array_unique($expected));
            $actual = array_values(array_unique($actual));
            sort($expected, SORT_STRING);
            sort($actual, SORT_STRING);

            if ($expected === $actual) {
                ++$correctCount;
            }
        }

        if ($questionCount === 0) {
            throw new \InvalidArgumentException('Ce quiz ne contient aucune question avec des choix.');
        }

        $attemptScore = (int) round(($correctCount / $questionCount) * 100);
        $passingScore = max(0, min(100, $quiz->getNoteMinimale()));

        $parentFormation = $this->qualification->resolveParentFormation($resultFormation);
        $badge = $parentFormation?->getBadge();
        $hadBadge = $badge !== null && $this->userBadges->findOneBy([
            'utilisateur' => $user,
            'badge' => $badge,
        ]) !== null;

        $progression = $this->progressions->findOneBy([
            'utilisateur' => $user,
            'formation' => $resultFormation,
        ]);

        $now = new \DateTimeImmutable();
        if (!$progression instanceof Progression) {
            $progression = (new Progression())
                ->setUtilisateur($user)
                ->setFormation($resultFormation)
                ->setDateDebut($now->modify('-1 second'));
            $this->entityManager->persist($progression);
        }

        $bestScore = max($progression->getScore(), $attemptScore);
        $progression
            ->setScore($bestScore)
            ->setCompleted($bestScore >= $passingScore)
            ->setDateEnd($now);

        $this->entityManager->flush();

        if (($this->catalog->isSectionQuizFormation($resultFormation) || $this->catalog->isRequiredPageQuizFormation($resultFormation)) && $parentFormation !== null) {
            $this->guidedTraining->synchronizeParentProgress($parentFormation, $user, true);
        }

        $qualificationStatus = $parentFormation !== null
            ? $this->qualification->getStatus($parentFormation, $user)
            : null;
        $hasBadge = $badge !== null && $this->userBadges->findOneBy([
            'utilisateur' => $user,
            'badge' => $badge,
        ]) !== null;

        return [
            'attemptScore' => $attemptScore,
            'bestScore' => $bestScore,
            'correctCount' => $correctCount,
            'questionCount' => $questionCount,
            'passed' => $attemptScore >= $passingScore,
            'completed' => $bestScore >= $passingScore,
            'badgeAwarded' => !$hadBadge && $hasBadge,
            'badge' => $badge !== null ? [
                'id' => (int) $badge->getId(),
                'nom' => $badge->getNom(),
                'description' => $badge->getDescription(),
                'icone' => $badge->getIcone(),
            ] : null,
            'qualification' => [
                'overallPercent' => (int) ($qualificationStatus['overallPercent'] ?? 0),
                'theoryReady' => (bool) ($qualificationStatus['theoryReady'] ?? false),
                'pathCompleted' => (bool) ($qualificationStatus['pathCompleted'] ?? false),
                'physicalCompleted' => (bool) ($qualificationStatus['physicalCompleted'] ?? false),
                'eligible' => (bool) ($qualificationStatus['eligible'] ?? false),
            ],
        ];
    }
}
