<?php

namespace App\Service;

use App\Entity\Badge;
use App\Entity\Formation;
use App\Entity\Progression;
use App\Entity\Utilisateur;
use App\Repository\FormationRepository;
use App\Repository\ProgressionRepository;
use App\Repository\UtilisateurBadgeRepository;

final class TrainingQualificationService
{
    public const PHYSICAL_CATEGORY = 'Validation physique';
    private const QUIZ_PARENT_MARKER = 'FABOS_PARENT_FORMATION_ID=';
    private const PHYSICAL_PARENT_MARKER = 'FABOS_PHYSICAL_PARENT_FORMATION_ID=';

    public function __construct(
        private readonly FormationRepository $formations,
        private readonly ProgressionRepository $progressions,
        private readonly UtilisateurBadgeRepository $userBadges,
        private readonly TrainingPolicyService $policy,
        private readonly QuizCatalogService $quizCatalog,
        private readonly GuidedTrainingService $guidedTraining,
    ) {
    }

    public function resolveParentFormation(Formation $formation): ?Formation
    {
        $category = mb_strtolower(trim((string) $formation->getCategorie()));
        $parentId = null;

        if ($category === mb_strtolower(QuizCatalogService::INTERNAL_CATEGORY)) {
            $parentId = self::extractMarkerId($formation->getPrerequis(), self::QUIZ_PARENT_MARKER);
        } elseif ($category === mb_strtolower(self::PHYSICAL_CATEGORY)) {
            $parentId = self::extractMarkerId($formation->getPrerequis(), self::PHYSICAL_PARENT_MARKER);
        }

        if ($parentId === null) {
            return $formation;
        }

        return $this->formations->find($parentId);
    }

    public function getPhysicalFormation(Formation $parentFormation): ?Formation
    {
        $parentFormation = $this->resolveParentFormation($parentFormation) ?? $parentFormation;
        if (!$this->policy->formationRequiresPhysicalTraining($parentFormation)) {
            return null;
        }

        $parentId = $parentFormation->getId();
        if ($parentId === null) {
            return null;
        }

        return $this->formations->findPhysicalValidationForParent($parentId);
    }

    /**
     * @return array{
     *   parentFormation: Formation,
     *   badge: ?Badge,
     *   badgeOwned: bool,
     *   badgeUnlocked: bool,
     *   badgeObtainedAt: ?\DateTimeImmutable,
     *   trainingProgression: ?Progression,
     *   trainingScore: int,
     *   quizTotal: int,
     *   quizValidated: int,
     *   quizAverage: int,
     *   overallPercent: int,
     *   theoryReady: bool,
     *   pathCompleted: bool,
     *   physicalRequired: bool,
     *   physicalFormation: ?Formation,
     *   physicalProgression: ?Progression,
     *   physicalCompleted: bool,
     *   eligible: bool
     * }
     */
    public function getStatus(Formation $formation, Utilisateur $user): array
    {
        $parent = $this->resolveParentFormation($formation) ?? $formation;
        $guidedStatus = $this->guidedTraining->getProgress($parent, $user);
        $trainingProgression = $guidedStatus['progression'];
        if ($guidedStatus['configured']) {
            $trainingScore = $guidedStatus['score'];
            $trainingCompleted = $guidedStatus['completedAll'];
        } else {
            $trainingScore = $trainingProgression?->isCompleted()
                ? 100
                : max(0, min(100, $trainingProgression?->getScore() ?? 0));
            $trainingCompleted = $trainingProgression?->isCompleted() ?? false;
        }

        $quizFormations = [];
        if ($parent->getId() !== null) {
            foreach ($this->formations->findQuizFormationsForParent($parent->getId()) as $quizFormation) {
                if ($this->quizCatalog->isRequiredPageQuizFormation($quizFormation)) {
                    $quizFormations[] = $quizFormation;
                }
            }
        }

        $quizTotal = count($quizFormations);
        $quizValidated = 0;

        foreach ($quizFormations as $quizFormation) {
            $quizProgression = $this->progressions->findOneBy([
                'utilisateur' => $user,
                'formation' => $quizFormation,
            ]);

            if ($quizProgression instanceof Progression && $quizProgression->isCompleted()) {
                ++$quizValidated;
            }
        }

        $quizAverage = $quizTotal > 0 ? (int) round(($quizValidated / $quizTotal) * 100) : 100;
        $trainingWeight = $quizTotal > 0 ? 60 : 100;
        $quizWeight = $quizTotal > 0 ? 40 : 0;
        $overallPercent = (int) round(
            ($trainingScore / 100) * $trainingWeight
            + ($quizAverage / 100) * $quizWeight,
        );

        $physicalRequired = $this->policy->formationRequiresPhysicalTraining($parent);
        $physicalFormation = $physicalRequired ? $this->getPhysicalFormation($parent) : null;
        $physicalProgression = $physicalFormation instanceof Formation
            ? $this->progressions->findOneBy([
                'utilisateur' => $user,
                'formation' => $physicalFormation,
            ])
            : null;
        $physicalCompleted = !$physicalRequired || (
            $physicalProgression instanceof Progression
            && $physicalProgression->isCompleted()
        );

        $badge = $parent->getBadge();
        $userBadge = $badge instanceof Badge
            ? $this->userBadges->findOneBy([
                'utilisateur' => $user,
                'badge' => $badge,
            ])
            : null;

        $pathCompleted = $trainingCompleted && ($quizTotal === 0 || $quizValidated === $quizTotal);
        $theoryReady = $overallPercent >= 80;

        return [
            'parentFormation' => $parent,
            'badge' => $badge,
            'badgeOwned' => $userBadge !== null,
            'badgeUnlocked' => $userBadge !== null && $theoryReady && $physicalCompleted,
            'badgeObtainedAt' => $userBadge?->getDateObtention(),
            'trainingProgression' => $trainingProgression,
            'trainingScore' => $trainingScore,
            'quizTotal' => $quizTotal,
            'quizValidated' => $quizValidated,
            'quizAverage' => $quizAverage,
            'overallPercent' => $overallPercent,
            'theoryReady' => $theoryReady,
            'pathCompleted' => $pathCompleted,
            'physicalRequired' => $physicalRequired,
            'physicalFormation' => $physicalFormation,
            'physicalProgression' => $physicalProgression,
            'physicalCompleted' => $physicalCompleted,
            'eligible' => $badge instanceof Badge && $theoryReady && $physicalCompleted,
        ];
    }

    public static function isInternalCategory(?string $category): bool
    {
        $normalized = mb_strtolower(trim((string) $category));

        return in_array($normalized, [
            mb_strtolower(QuizCatalogService::INTERNAL_CATEGORY),
            mb_strtolower(self::PHYSICAL_CATEGORY),
        ], true);
    }

    private static function extractMarkerId(?string $value, string $marker): ?int
    {
        if (!is_string($value) || $value === '') {
            return null;
        }

        if (preg_match('/(?:^|;)' . preg_quote($marker, '/') . '(\d+);?/', $value, $matches) !== 1) {
            return null;
        }

        $id = (int) $matches[1];

        return $id > 0 ? $id : null;
    }
}
