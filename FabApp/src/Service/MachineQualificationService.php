<?php

namespace App\Service;

use App\Entity\Machine;
use App\Entity\Utilisateur;
use App\Repository\FormationRepository;
use App\Repository\UtilisateurBadgeRepository;

final class MachineQualificationService
{
    public function __construct(
        private readonly UtilisateurBadgeRepository $userBadges,
        private readonly FormationRepository $formations,
        private readonly TrainingQualificationService $qualification,
        private readonly TrainingPolicyService $policy,
    ) {
    }

    /**
     * @return array{
     *   authorized: bool,
     *   authorizationStatus: string,
     *   badgeRows: array<int, array{machineBadge: object, badge: object, owned: bool, qualified: bool}>,
     *   missingBadges: array<int, object>,
     *   trainingMode: string,
     *   trainingRequired: bool,
     *   physicalTrainingRequired: bool,
 *   physicalTrainingCompleted: bool,
 *   trainingBlockReason: ?string
     * }
     */
    public function getStatus(Machine $machine, ?Utilisateur $user): array
    {
        $trainingMode = $this->policy->getMachineMode($machine);
        $trainingRequired = $trainingMode !== TrainingPolicyService::MODE_FREE;
        $physicalTrainingRequired = $trainingMode === TrainingPolicyService::MODE_THEORY_PHYSICAL;

        if (!$trainingRequired) {
            return [
                'authorized' => $user instanceof Utilisateur,
                'authorizationStatus' => 'no_badge_required',
                'badgeRows' => [],
                'missingBadges' => [],
                'trainingMode' => $trainingMode,
                'trainingRequired' => false,
                'physicalTrainingRequired' => false,
                'physicalTrainingCompleted' => true,
                'trainingBlockReason' => null,
            ];
        }

        $badgeRows = [];
        $missingBadges = [];
        $physicalStatusKnown = false;
        $physicalTrainingCompleted = !$physicalTrainingRequired;

        foreach ($machine->getRequiredMachineBadges() as $machineBadge) {
            $badge = $machineBadge->getBadge();
            if ($badge === null) {
                continue;
            }

            $owned = $user instanceof Utilisateur && $this->userBadges->findOneBy([
                'utilisateur' => $user,
                'badge' => $badge,
            ]) !== null;

            $qualified = $owned;
            $physicalCompletedForBadge = null;
            if ($user instanceof Utilisateur) {
                $formation = $this->formations->findVisibleByBadge($badge);
                if ($formation !== null) {
                    $qualificationStatus = $this->qualification->getStatus($formation, $user);
                    $qualified = (bool) $qualificationStatus['badgeUnlocked'];
                    if ((bool) ($qualificationStatus['physicalRequired'] ?? false)) {
                        $physicalStatusKnown = true;
                        $physicalCompletedForBadge = (bool) ($qualificationStatus['physicalCompleted'] ?? false);
                        $physicalTrainingCompleted = $physicalTrainingCompleted || $physicalCompletedForBadge;
                    }
                }
            }

            $badgeRows[] = [
                'machineBadge' => $machineBadge,
                'badge' => $badge,
                'owned' => $qualified,
                'qualified' => $qualified,
                'physicalCompleted' => $physicalCompletedForBadge,
            ];

            if (!$qualified) {
                $missingBadges[] = $badge;
            }
        }

        if ($physicalTrainingRequired && !$physicalStatusKnown) {
            $physicalTrainingCompleted = false;
        }

        $hasRequirements = $badgeRows !== [];
        $authorized = $user instanceof Utilisateur
            && (!$hasRequirements || $missingBadges === [])
            && (!$physicalTrainingRequired || $physicalTrainingCompleted);

        $trainingBlockReason = null;
        if ($physicalTrainingRequired && !$physicalTrainingCompleted) {
            $trainingBlockReason = 'physical_training_required';
        } elseif ($missingBadges !== []) {
            $trainingBlockReason = 'training_required';
        }

        $authorizationStatus = !$hasRequirements
            ? ($authorized ? 'no_badge_required' : 'physical_training_required')
            : ($authorized ? 'authorized' : ($trainingBlockReason ?? 'missing_badge'));

        return [
            'authorized' => $authorized,
            'authorizationStatus' => $authorizationStatus,
            'badgeRows' => $badgeRows,
            'missingBadges' => $missingBadges,
            'trainingMode' => $trainingMode,
            'trainingRequired' => $trainingRequired,
            'physicalTrainingRequired' => $physicalTrainingRequired,
            'physicalTrainingCompleted' => $physicalTrainingCompleted,
            'trainingBlockReason' => $trainingBlockReason,
        ];
    }
}
