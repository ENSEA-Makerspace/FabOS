<?php

namespace App\Controller;

use App\Entity\Formation;
use App\Entity\Machine;
use App\Entity\Quiz;
use App\Entity\Utilisateur;
use App\Repository\ChoixRepository;
use App\Repository\FormationRepository;
use App\Repository\MachineRepository;
use App\Repository\ProgressionRepository;
use App\Repository\QuestionRepository;
use App\Repository\QuizRepository;
use App\Service\MachineQuizFactory;
use App\Service\QuizCatalogService;
use App\Service\QuizProgressService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\JsonResponse;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;

final class QuizController extends AbstractController
{
    #[Route('/formations/{formationId}/quiz/{quizId}', name: 'app_quiz_show', requirements: ['formationId' => '\\d+', 'quizId' => '\\d+'], methods: ['GET'])]
    public function show(
        int $formationId,
        int $quizId,
        FormationRepository $formations,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        ChoixRepository $choices,
        ProgressionRepository $progressions,
        QuizCatalogService $catalog,
    ): Response {
        $contextFormation = $formations->find($formationId);
        $quiz = $quizzes->find($quizId);

        if (!$contextFormation instanceof Formation || !$quiz instanceof Quiz) {
            throw $this->createNotFoundException('Quiz introuvable pour cette formation.');
        }

        $quizFormation = $quiz->getFormation();
        $directMatch = $quizFormation?->getId() === $contextFormation->getId();
        $childMatch = $catalog->isInternalQuizFormation($quizFormation)
            && $catalog->getParentFormationId($quizFormation) === $contextFormation->getId();

        if (!$directMatch && !$childMatch) {
            throw $this->createNotFoundException('Quiz introuvable pour cette formation.');
        }

        $quizData = $this->buildStoredQuizData($quiz, $questions, $choices);

        return $this->renderQuiz($quizData, $contextFormation, null, $quiz, $progressions, $catalog);
    }

    #[Route('/machines/{id}/quiz', name: 'app_machine_quiz', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function machineQuiz(
        int $id,
        MachineRepository $machines,
        FormationRepository $formations,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        ChoixRepository $choices,
        ProgressionRepository $progressions,
        MachineQuizFactory $factory,
        QuizCatalogService $catalog,
    ): Response {
        $machine = $machines->find($id);
        if (!$machine instanceof Machine) {
            throw $this->createNotFoundException('Machine introuvable.');
        }

        $storedCandidates = [];
        foreach ($formations->findQuizFormationsForMachine($machine->getId() ?? 0) as $quizFormation) {
            foreach ($quizzes->findBy(['formation' => $quizFormation], ['id' => 'ASC']) as $quiz) {
                $storedCandidates[] = $quiz;
            }
        }

        if ($storedCandidates !== []) {
            $selectedQuiz = $storedCandidates[0];
            $user = $this->getUser();
            if ($user instanceof Utilisateur) {
                foreach ($storedCandidates as $candidate) {
                    $result = $progressions->findOneBy([
                        'utilisateur' => $user,
                        'formation' => $candidate->getFormation(),
                    ]);
                    if ($result === null || !$result->isCompleted()) {
                        $selectedQuiz = $candidate;
                        break;
                    }
                }
            }

            $parentFormationId = $catalog->getParentFormationId($selectedQuiz->getFormation());
            $contextFormation = $parentFormationId !== null ? $formations->find($parentFormationId) : null;
            $quizData = $this->buildStoredQuizData($selectedQuiz, $questions, $choices);
            $quizData['title'] = ($selectedQuiz->getSection()?->getTitre() ?: 'Quiz sécurité') . ' · ' . $machine->getNom();
            $quizData['subtitle'] = $contextFormation instanceof Formation
                ? 'Évaluation liée à la formation « ' . $contextFormation->getTitre() . ' ».'
                : 'Évaluation personnalisée pour cette machine.';
            $quizData['eyebrow'] = $machine->getCategoryLabel();

            return $this->renderQuiz($quizData, $contextFormation, $machine, $selectedQuiz, $progressions, $catalog);
        }

        $resolved = $this->resolveLegacyStoredQuizForMachine($machine, $formations, $quizzes);
        if ($resolved !== null) {
            [$formation, $quiz] = $resolved;
            $quizData = $this->buildStoredQuizData($quiz, $questions, $choices);
            $quizData['title'] = 'Quiz sécurité · ' . $machine->getNom();
            $quizData['subtitle'] = 'Évaluation liée à la formation « ' . $formation->getTitre() . ' ».';
            $quizData['eyebrow'] = $machine->getCategoryLabel();

            return $this->renderQuiz($quizData, $formation, $machine, $quiz, $progressions, $catalog);
        }

        return $this->renderQuiz($factory->create($machine), null, $machine, null, $progressions, $catalog);
    }

    #[Route('/api/quizzes/{quizId}/result', name: 'app_quiz_result_save', requirements: ['quizId' => '\\d+'], methods: ['POST'])]
    public function saveResult(
        int $quizId,
        Request $request,
        QuizRepository $quizzes,
        QuizProgressService $quizProgress,
        QuizCatalogService $catalog,
    ): JsonResponse {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            return $this->json(['ok' => false, 'message' => 'Connectez-vous pour enregistrer votre résultat.'], Response::HTTP_UNAUTHORIZED);
        }

        $quiz = $quizzes->find($quizId);
        if (!$quiz instanceof Quiz) {
            return $this->json(['ok' => false, 'message' => 'Quiz introuvable.'], Response::HTTP_NOT_FOUND);
        }

        try {
            $payload = $request->toArray();
        } catch (\Throwable) {
            return $this->json(['ok' => false, 'message' => 'Données de résultat invalides.'], Response::HTTP_BAD_REQUEST);
        }

        $token = (string) ($payload['_token'] ?? '');
        if (!$this->isCsrfTokenValid('quiz_result_' . $quizId, $token)) {
            return $this->json(['ok' => false, 'message' => 'La sauvegarde a été refusée. Rechargez la page puis réessayez.'], Response::HTTP_FORBIDDEN);
        }

        $answers = $payload['answers'] ?? null;
        if (!is_array($answers)) {
            return $this->json(['ok' => false, 'message' => 'Les réponses transmises sont invalides.'], Response::HTTP_UNPROCESSABLE_ENTITY);
        }

        try {
            $result = $quizProgress->saveResult($quiz, $user, $answers);
        } catch (\LogicException $exception) {
            return $this->json(['ok' => false, 'message' => $exception->getMessage()], Response::HTTP_CONFLICT);
        } catch (\InvalidArgumentException $exception) {
            return $this->json(['ok' => false, 'message' => $exception->getMessage()], Response::HTTP_UNPROCESSABLE_ENTITY);
        }

        if ($catalog->isSectionQuizFormation($quiz->getFormation())) {
            $message = $result['completed']
                ? 'Section validée. La prochaine étape est maintenant déverrouillée.'
                : 'Résultat enregistré. Relisez la section puis recommencez pour la valider.';
        } elseif ($catalog->isBonusQuizFormation($quiz->getFormation())) {
            $message = $result['completed']
                ? 'Quiz bonus réussi. Ce résultat reste hors du calcul de progression.'
                : 'Résultat bonus enregistré. Il ne modifie pas la progression obligatoire.';
        } else {
            $message = $result['completed']
                ? 'Résultat enregistré. Votre meilleur score valide ce quiz.'
                : 'Résultat enregistré. Vous pourrez recommencer pour améliorer votre score.';
        }

        if ($result['badgeAwarded']) {
            $message = 'Résultat enregistré : votre badge vient d’être débloqué.';
        } elseif ($result['qualification']['theoryReady'] && !$result['qualification']['physicalCompleted']) {
            $message .= ' La progression théorique est suffisante ; la validation physique reste nécessaire pour obtenir le badge.';
        }

        return $this->json([
            'ok' => true,
            'message' => $message,
            'result' => $result,
        ]);
    }

    /**
     * @return array{0: Formation, 1: Quiz}|null
     */
    private function resolveLegacyStoredQuizForMachine(
        Machine $machine,
        FormationRepository $formations,
        QuizRepository $quizzes,
    ): ?array {
        $candidateTitles = [
            'Formation sécurité - ' . $machine->getNom(),
            'Quiz sécurité - ' . $machine->getNom(),
            'Formation ' . $machine->getNom(),
            $machine->getNom(),
        ];

        foreach ($candidateTitles as $title) {
            $formation = $formations->findOneByNormalizedTitle($title);
            if (!$formation instanceof Formation) {
                continue;
            }

            $quiz = $quizzes->findOneBy(['formation' => $formation], ['id' => 'ASC']);
            if ($quiz instanceof Quiz) {
                return [$formation, $quiz];
            }
        }

        foreach ($machine->getRequiredMachineBadges() as $machineBadge) {
            $badge = $machineBadge->getBadge();
            if ($badge === null) {
                continue;
            }

            foreach ($formations->findBy(['badge' => $badge], ['id' => 'ASC']) as $formation) {
                $quiz = $quizzes->findOneBy(['formation' => $formation], ['id' => 'ASC']);
                if ($quiz instanceof Quiz) {
                    return [$formation, $quiz];
                }
            }
        }

        return null;
    }

    /** @return array<string, mixed> */
    private function buildStoredQuizData(
        Quiz $quiz,
        QuestionRepository $questions,
        ChoixRepository $choices,
    ): array {
        $questionRows = [];
        foreach ($questions->findBy(['quiz' => $quiz], ['ordre' => 'ASC']) as $question) {
            $choiceRows = [];
            $correctCount = 0;

            foreach ($choices->findBy(['question' => $question], ['ordre' => 'ASC']) as $choice) {
                if ($choice->isEstCorrect()) {
                    ++$correctCount;
                }

                $choiceRows[] = [
                    'id' => (string) $choice->getId(),
                    'text' => $choice->getTexte(),
                    'correct' => $choice->isEstCorrect(),
                ];
            }

            if ($choiceRows === []) {
                continue;
            }

            $questionRows[] = [
                'id' => (string) $question->getId(),
                'order' => $question->getOrdre(),
                'text' => $question->getTexte(),
                'type' => $correctCount > 1 ? 'multiple' : 'single',
                'choices' => $choiceRows,
            ];
        }

        $formation = $quiz->getFormation();
        $passingScore = max(0, min(100, $quiz->getNoteMinimale()));

        return [
            'id' => (string) $quiz->getId(),
            'source' => 'database',
            'title' => $quiz->getSection()?->getTitre() ?: ($quiz->getFormation()?->getTitre() ?: 'Quiz · ' . ($formation?->getTitre() ?? 'Formation')),
            'subtitle' => 'Répondez à toutes les questions. Le résultat est vérifié par le serveur puis enregistré.',
            'eyebrow' => $formation?->getCategorie() ?: 'Formation FabOS',
            'passingScore' => $passingScore,
            'questionCount' => count($questionRows),
            'questions' => $questionRows,
        ];
    }

    /** @param array<string, mixed> $quizData */
    private function renderQuiz(
        array $quizData,
        ?Formation $formation,
        ?Machine $machine,
        ?Quiz $databaseQuiz,
        ProgressionRepository $progressions,
        QuizCatalogService $catalog,
    ): Response {
        $user = $this->getUser();
        $persistent = $databaseQuiz instanceof Quiz
            && $catalog->isInternalQuizFormation($databaseQuiz->getFormation());
        $previousResult = null;

        if ($persistent && $user instanceof Utilisateur) {
            $previousResult = $progressions->findOneBy([
                'utilisateur' => $user,
                'formation' => $databaseQuiz->getFormation(),
            ]);
        }

        return $this->render('site/quiz.html.twig', [
            'quizData' => $quizData,
            'quizDataJson' => json_encode(
                $quizData,
                JSON_THROW_ON_ERROR | JSON_HEX_TAG | JSON_HEX_AMP | JSON_HEX_APOS | JSON_HEX_QUOT,
            ),
            'formation' => $formation,
            'machine' => $machine,
            'quizPersistence' => [
                'enabled' => $persistent && $user instanceof Utilisateur,
                'requiresLogin' => $persistent && !$user instanceof Utilisateur,
                'quizId' => $databaseQuiz?->getId(),
                'previousScore' => $previousResult?->getScore(),
                'previousCompleted' => $previousResult?->isCompleted() ?? false,
            ],
            'quizContext' => [
                'section' => $databaseQuiz instanceof Quiz && $catalog->isSectionQuizFormation($databaseQuiz->getFormation()),
                'bonus' => $databaseQuiz instanceof Quiz && $catalog->isBonusQuizFormation($databaseQuiz->getFormation()),
            ],
        ]);
    }
}
