<?php

namespace App\Controller;

use App\Entity\Choix;
use App\Entity\Formation;
use App\Entity\Question;
use App\Entity\Quiz;
use App\Entity\Section;
use App\Repository\ChoixRepository;
use App\Repository\FormationRepository;
use App\Repository\QuestionRepository;
use App\Repository\QuizRepository;
use App\Repository\SectionRepository;
use App\Service\FormationPageContentService;
use App\Service\QuizCatalogService;
use App\Service\TrainingQualificationService;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

#[Route('/admin/formations')]
#[IsGranted('ROLE_ADMIN')]
final class FormationContentAdminController extends AbstractController
{
    #[Route('/{id}/content', name: 'app_admin_formation_content', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function editor(
        int $id,
        FormationRepository $formations,
        SectionRepository $sections,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        FormationPageContentService $pageContent,
        QuizCatalogService $catalog,
    ): Response {
        $formation = $this->findVisibleFormation($id, $formations);

        return $this->render('site/admin-formation-content.html.twig', [
            'formation' => $formation,
            'pageContent' => $pageContent->getContent($formation),
            'sections' => $sections->findJourneySections($formation),
            'quizRows' => $this->buildQuizRows($formation, $formations, $quizzes, $questions, $catalog),
        ]);
    }

    #[Route('/{id}/content/general', name: 'app_admin_formation_content_general', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function updateGeneral(
        int $id,
        Request $request,
        FormationRepository $formations,
        EntityManagerInterface $entityManager,
    ): Response {
        $formation = $this->findVisibleFormation($id, $formations);
        $this->assertCsrf('formation_content_general_' . $id, (string) $request->request->get('_token'));

        $title = trim((string) $request->request->get('titre'));
        if ($title === '') {
            $this->addFlash('error', 'Le titre de la formation ne peut pas être vide.');
            return $this->redirectToRoute('app_admin_formation_content', ['id' => $id], Response::HTTP_SEE_OTHER);
        }
        if (mb_strlen($title) > 255) {
            $this->addFlash('error', 'Le titre de la formation est limité à 255 caractères.');
            return $this->redirectToRoute('app_admin_formation_content', ['id' => $id], Response::HTTP_SEE_OTHER);
        }

        $level = trim((string) $request->request->get('niveau'));
        $places = trim((string) $request->request->get('placesTotales'));

        $formation
            ->setTitre($title)
            ->setDescription($this->nullableText($request->request->get('description')))
            ->setCategorie($this->limitedNullableText($request->request->get('categorie'), 100))
            ->setNiveau($level === '' ? null : max(0, (int) $level))
            ->setDuree($this->limitedNullableText($request->request->get('duree'), 100))
            ->setFormateur($this->limitedNullableText($request->request->get('formateur'), 150))
            ->setPlacesTotales($places === '' ? null : max(0, (int) $places))
            ->setObjectifs($this->linesToStoredText($request->request->get('objectifs'), '. '))
            ->setPrerequis($this->linesToStoredText($request->request->get('prerequis'), "\n"))
            ->setMaterielFourni($this->linesToStoredText($request->request->get('materielFourni'), ', '));

        $entityManager->flush();
        $this->addFlash('success', 'Les informations générales de la formation ont été mises à jour.');

        return $this->redirectToRoute('app_admin_formation_content', ['id' => $id], Response::HTTP_SEE_OTHER);
    }

    #[Route('/{id}/content/block/{block}', name: 'app_admin_formation_content_block', requirements: ['id' => '\\d+', 'block' => 'labels|journey|program|sessions|practical|related'], methods: ['POST'])]
    public function updateBlock(
        int $id,
        string $block,
        Request $request,
        FormationRepository $formations,
        FormationPageContentService $pageContent,
    ): Response {
        $formation = $this->findVisibleFormation($id, $formations);
        $this->assertCsrf('formation_content_block_' . $id . '_' . $block, (string) $request->request->get('_token'));

        $payload = match ($block) {
            'labels' => $this->buildLabelsPayload($request),
            'journey' => $this->buildJourneyPayload($request),
            'program' => $this->buildProgramPayload($request),
            'sessions' => $this->buildSessionsPayload($request),
            'practical' => $this->buildPracticalPayload($request),
            'related' => $this->buildRelatedPayload($request),
            default => throw $this->createNotFoundException('Bloc de contenu inconnu.'),
        };

        $pageContent->saveBlock($formation, $block, $payload);
        $this->addFlash('success', 'Le bloc de texte a été mis à jour.');

        return $this->redirectToRoute('app_admin_formation_content', ['id' => $id], Response::HTTP_SEE_OTHER);
    }

    #[Route('/{id}/sections/new', name: 'app_admin_formation_section_new', requirements: ['id' => '\\d+'], methods: ['GET', 'POST'])]
    public function newSection(
        int $id,
        Request $request,
        FormationRepository $formations,
        SectionRepository $sections,
        EntityManagerInterface $entityManager,
    ): Response {
        $formation = $this->findVisibleFormation($id, $formations);
        $section = (new Section())
            ->setFormation($formation)
            ->setOrdre($sections->getNextJourneyOrder($formation));

        return $this->handleSectionForm($formation, $section, $request, $entityManager, true);
    }

    #[Route('/{id}/sections/{sectionId}/edit', name: 'app_admin_formation_section_edit', requirements: ['id' => '\\d+', 'sectionId' => '\\d+'], methods: ['GET', 'POST'])]
    public function editSection(
        int $id,
        int $sectionId,
        Request $request,
        FormationRepository $formations,
        SectionRepository $sections,
        EntityManagerInterface $entityManager,
    ): Response {
        $formation = $this->findVisibleFormation($id, $formations);
        $section = $sections->find($sectionId);
        if (!$section instanceof Section || $section->getFormation()?->getId() !== $formation->getId() || SectionRepository::isPageContentBlock($section)) {
            throw $this->createNotFoundException('Section introuvable pour cette formation.');
        }

        return $this->handleSectionForm($formation, $section, $request, $entityManager, false);
    }

    #[Route('/{id}/quizzes/new', name: 'app_admin_formation_quiz_new', requirements: ['id' => '\\d+'], methods: ['GET', 'POST'])]
    public function newQuiz(
        int $id,
        Request $request,
        FormationRepository $formations,
        SectionRepository $sections,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        ChoixRepository $choices,
        EntityManagerInterface $entityManager,
        QuizCatalogService $catalog,
    ): Response {
        $formation = $this->findVisibleFormation($id, $formations);

        return $this->handleQuizForm(
            $formation,
            null,
            $request,
            $formations,
            $sections,
            $quizzes,
            $questions,
            $choices,
            $entityManager,
            $catalog,
        );
    }

    #[Route('/{id}/quizzes/{quizId}/edit', name: 'app_admin_formation_quiz_edit', requirements: ['id' => '\\d+', 'quizId' => '\\d+'], methods: ['GET', 'POST'])]
    public function editQuiz(
        int $id,
        int $quizId,
        Request $request,
        FormationRepository $formations,
        SectionRepository $sections,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        ChoixRepository $choices,
        EntityManagerInterface $entityManager,
        QuizCatalogService $catalog,
    ): Response {
        $formation = $this->findVisibleFormation($id, $formations);
        $quiz = $quizzes->find($quizId);
        if (!$quiz instanceof Quiz || !$this->quizBelongsToFormation($quiz, $formation, $catalog)) {
            throw $this->createNotFoundException('Quiz introuvable pour cette formation.');
        }

        if (!$catalog->isInternalQuizFormation($quiz->getFormation())) {
            $this->addFlash('warning', 'Ce quiz historique est directement rattaché à la formation. Il reste consultable, mais son type ne peut pas être converti depuis cet éditeur.');
        }

        return $this->handleQuizForm(
            $formation,
            $quiz,
            $request,
            $formations,
            $sections,
            $quizzes,
            $questions,
            $choices,
            $entityManager,
            $catalog,
        );
    }

    private function handleSectionForm(
        Formation $formation,
        Section $section,
        Request $request,
        EntityManagerInterface $entityManager,
        bool $isNew,
    ): Response {
        $formData = $this->sectionFormData($section);
        $errors = [];

        if ($request->isMethod('POST')) {
            $this->assertCsrf('formation_section_' . ($isNew ? 'new_' . $formation->getId() : 'edit_' . $section->getId()), (string) $request->request->get('_token'));
            $formData = [
                'titre' => trim((string) $request->request->get('titre')),
                'ordre' => max(1, (int) $request->request->get('ordre', 1)),
                'videoUrl' => trim((string) $request->request->get('videoUrl')),
                'intro' => trim((string) $request->request->get('intro')),
                'objectives' => trim((string) $request->request->get('objectives')),
                'steps' => trim((string) $request->request->get('steps')),
                'callouts' => trim((string) $request->request->get('callouts')),
            ];

            if ($formData['titre'] === '') {
                $errors[] = 'Le titre de la section est obligatoire.';
            } elseif (mb_strlen($formData['titre']) > 255) {
                $errors[] = 'Le titre de la section est limité à 255 caractères.';
            } elseif (str_starts_with($formData['titre'], SectionRepository::PAGE_BLOCK_PREFIX)) {
                $errors[] = 'Ce préfixe de titre est réservé au système de contenu de la page.';
            }
            if (mb_strlen($formData['videoUrl']) > 255) {
                $errors[] = 'L’URL vidéo est limitée à 255 caractères.';
            }
            if ($formData['intro'] === '') {
                $errors[] = 'Le texte d’introduction de la section est obligatoire.';
            }

            $steps = $this->parsePipeLines($formData['steps'], 2, ['title', 'text']);
            if ($steps === []) {
                $errors[] = 'Ajoutez au moins une étape au format Titre | Texte.';
            }

            if ($errors === []) {
                $content = [
                    'intro' => $formData['intro'],
                    'objectives' => $this->parseSimpleLines($formData['objectives']),
                    'steps' => $steps,
                    'callouts' => $this->parseCallouts($formData['callouts']),
                ];

                $section
                    ->setTitre($formData['titre'])
                    ->setOrdre($formData['ordre'])
                    ->setVideoUrl($formData['videoUrl'] !== '' ? $formData['videoUrl'] : null)
                    ->setContenu(json_encode($content, JSON_UNESCAPED_UNICODE | JSON_UNESCAPED_SLASHES | JSON_THROW_ON_ERROR));

                if ($isNew) {
                    $entityManager->persist($section);
                }
                $entityManager->flush();

                $this->addFlash('success', $isNew ? 'La nouvelle section a été créée.' : 'La section a été mise à jour.');

                return $this->redirectToRoute('app_admin_formation_content', ['id' => $formation->getId()], Response::HTTP_SEE_OTHER);
            }
        }

        return $this->render('site/admin-formation-section-form.html.twig', [
            'formation' => $formation,
            'section' => $section,
            'formData' => $formData,
            'errors' => $errors,
            'isNew' => $isNew,
        ], $errors !== [] ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    private function handleQuizForm(
        Formation $formation,
        ?Quiz $quiz,
        Request $request,
        FormationRepository $formations,
        SectionRepository $sections,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        ChoixRepository $choices,
        EntityManagerInterface $entityManager,
        QuizCatalogService $catalog,
    ): Response {
        $isNew = !$quiz instanceof Quiz;
        $formData = $this->quizFormData($formation, $quiz, $questions, $choices, $catalog);
        if ($isNew) {
            $preselectedSection = trim((string) $request->query->get('section'));
            if (ctype_digit($preselectedSection)) {
                $formData['type'] = 'section';
                $formData['sectionId'] = $preselectedSection;
            }
        }
        $errors = [];

        if ($request->isMethod('POST')) {
            $this->assertCsrf('formation_quiz_' . ($isNew ? 'new_' . $formation->getId() : 'edit_' . $quiz->getId()), (string) $request->request->get('_token'));

            $formData = [
                'title' => trim((string) $request->request->get('title')),
                'type' => trim((string) $request->request->get('type')),
                'sectionId' => trim((string) $request->request->get('sectionId')),
                'noteMinimale' => max(0, min(100, (int) $request->request->get('noteMinimale', 80))),
                'questions' => $this->normalizeSubmittedQuestions($request->request->all('questions')),
            ];

            if ($formData['title'] === '') {
                $errors[] = 'Le titre du quiz est obligatoire.';
            } elseif (mb_strlen($formData['title']) > 255) {
                $errors[] = 'Le titre du quiz est limité à 255 caractères.';
            }
            if (!in_array($formData['type'], ['section', 'required', 'bonus'], true)) {
                $errors[] = 'Le type de quiz sélectionné est invalide.';
            }

            $selectedSection = null;
            if ($formData['type'] === 'section') {
                $selectedSection = ctype_digit($formData['sectionId']) ? $sections->find((int) $formData['sectionId']) : null;
                if (!$selectedSection instanceof Section || $selectedSection->getFormation()?->getId() !== $formation->getId() || SectionRepository::isPageContentBlock($selectedSection)) {
                    $errors[] = 'Sélectionnez une section valide pour ce mini-quiz.';
                } elseif ($this->sectionAlreadyHasQuiz($formation, $selectedSection, $formations, $quizzes, $catalog, $quiz)) {
                    $errors[] = 'Cette section possède déjà un autre mini-quiz. Modifiez le quiz existant au lieu d’en créer un second.';
                }
            }

            $errors = [...$errors, ...$this->validateQuestions($formData['questions'])];

            if ($errors === []) {
                $this->persistQuiz(
                    $formation,
                    $quiz,
                    $formData,
                    $selectedSection,
                    $questions,
                    $choices,
                    $entityManager,
                    $catalog,
                );

                $this->addFlash('success', $isNew ? 'Le nouveau quiz a été créé.' : 'Le quiz a été mis à jour.');

                return $this->redirectToRoute('app_admin_formation_content', ['id' => $formation->getId()], Response::HTTP_SEE_OTHER);
            }
        }

        return $this->render('site/admin-formation-quiz-form.html.twig', [
            'formation' => $formation,
            'quiz' => $quiz,
            'sections' => $sections->findJourneySections($formation),
            'formData' => $formData,
            'errors' => $errors,
            'isNew' => $isNew,
            'legacyDirectQuiz' => $quiz instanceof Quiz && !$catalog->isInternalQuizFormation($quiz->getFormation()),
        ], $errors !== [] ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
    }

    /** @param array<string, mixed> $formData */
    private function persistQuiz(
        Formation $parent,
        ?Quiz $quiz,
        array $formData,
        ?Section $selectedSection,
        QuestionRepository $questions,
        ChoixRepository $choices,
        EntityManagerInterface $entityManager,
        QuizCatalogService $catalog,
    ): void {
        $connection = $entityManager->getConnection();
        $connection->beginTransaction();

        try {
            $isNew = !$quiz instanceof Quiz;
            if ($isNew) {
                $internalFormation = (new Formation())
                    ->setBadge($parent->getBadge())
                    ->setImage($parent->getImage())
                    ->setCategorie(QuizCatalogService::INTERNAL_CATEGORY)
                    ->setNiveau($parent->getNiveau())
                    ->setDuree('5 min')
                    ->setFormateur('FabOS')
                    ->setObjectifs('Valider les acquis du quiz.')
                    ->setMaterielFourni('Quiz en ligne');
                $entityManager->persist($internalFormation);

                $quiz = (new Quiz())->setFormation($internalFormation);
                $entityManager->persist($quiz);
            } else {
                $internalFormation = $quiz->getFormation();
            }

            $isInternal = $catalog->isInternalQuizFormation($internalFormation) || $isNew;
            if ($isInternal && $internalFormation instanceof Formation) {
                $markers = $this->buildQuizMarkers(
                    $parent,
                    (string) $formData['type'],
                    $selectedSection,
                    $this->slugify((string) $formData['title']),
                );

                $internalFormation
                    ->setTitre((string) $formData['title'])
                    ->setDescription('Quiz administrable associé à la formation « ' . $parent->getTitre() . ' ».')
                    ->setPrerequis($markers);
            }

            $quiz
                ->setSection($formData['type'] === 'section' ? $selectedSection : null)
                ->setNoteMinimale((int) $formData['noteMinimale']);

            if (!$isNew) {
                foreach ($questions->findBy(['quiz' => $quiz]) as $question) {
                    foreach ($choices->findBy(['question' => $question]) as $choice) {
                        $entityManager->remove($choice);
                    }
                    $entityManager->remove($question);
                }
                $entityManager->flush();
            } else {
                $entityManager->flush();
            }

            foreach ($formData['questions'] as $questionIndex => $questionData) {
                $correctCount = count(array_filter($questionData['choices'], static fn (array $choice): bool => $choice['correct']));
                $question = (new Question())
                    ->setQuiz($quiz)
                    ->setTexte($questionData['text'])
                    ->setType($correctCount > 1 ? 'multiple' : 'single')
                    ->setOrdre($questionIndex + 1);
                $entityManager->persist($question);

                foreach ($questionData['choices'] as $choiceIndex => $choiceData) {
                    $choice = (new Choix())
                        ->setQuestion($question)
                        ->setTexte($choiceData['text'])
                        ->setEstCorrect($choiceData['correct'])
                        ->setOrdre($choiceIndex + 1);
                    $entityManager->persist($choice);
                }
            }

            $entityManager->flush();
            $connection->commit();
        } catch (\Throwable $exception) {
            if ($connection->isTransactionActive()) {
                $connection->rollBack();
            }
            throw $exception;
        }
    }

    /** @return list<array<string, mixed>> */
    private function buildQuizRows(
        Formation $formation,
        FormationRepository $formations,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        QuizCatalogService $catalog,
    ): array {
        $rows = [];
        $seen = [];

        foreach ($formations->findQuizFormationsForParent($formation->getId() ?? 0) as $quizFormation) {
            foreach ($quizzes->findBy(['formation' => $quizFormation], ['id' => 'ASC']) as $quiz) {
                $seen[$quiz->getId()] = true;
                $rows[] = $this->buildQuizRow($quiz, $questions, $catalog, false);
            }
        }

        foreach ($quizzes->findBy(['formation' => $formation], ['id' => 'ASC']) as $quiz) {
            if (!isset($seen[$quiz->getId()])) {
                $rows[] = $this->buildQuizRow($quiz, $questions, $catalog, true);
            }
        }

        return $rows;
    }

    /** @return array<string, mixed> */
    private function buildQuizRow(Quiz $quiz, QuestionRepository $questions, QuizCatalogService $catalog, bool $legacy): array
    {
        $quizFormation = $quiz->getFormation();
        $type = $catalog->isSectionQuizFormation($quizFormation)
            ? 'section'
            : ($catalog->isBonusQuizFormation($quizFormation) ? 'bonus' : 'required');

        return [
            'quiz' => $quiz,
            'title' => $quizFormation?->getTitre() ?: ($quiz->getSection()?->getTitre() ?: ''),
            'type' => $type,
            // Translation keys, not French text: this array is rendered by a template,
            // and a literal here is a string no catalogue can reach. The template
            // `|trans`es both, and falls back to the key-less title when empty.
            'typeLabelKey' => 'quiz_form.type_' . $type,
            'legacy' => $legacy,
            'section' => $quiz->getSection(),
            'questionCount' => $questions->count(['quiz' => $quiz]),
        ];
    }

    /** @return array<string, mixed> */
    private function sectionFormData(Section $section): array
    {
        $content = $this->decodeJson($section->getContenu());
        $steps = [];
        foreach (($content['steps'] ?? []) as $step) {
            if (is_array($step)) {
                $steps[] = trim((string) ($step['title'] ?? '')) . ' | ' . trim((string) ($step['text'] ?? ''));
            }
        }
        $callouts = [];
        foreach (($content['callouts'] ?? []) as $callout) {
            if (is_array($callout)) {
                $callouts[] = trim((string) ($callout['type'] ?? 'tip')) . ' | ' . trim((string) ($callout['title'] ?? '')) . ' | ' . trim((string) ($callout['text'] ?? ''));
            }
        }

        return [
            'titre' => $section->getTitre(),
            'ordre' => $section->getOrdre(),
            'videoUrl' => $section->getVideoUrl() ?? '',
            'intro' => trim((string) ($content['intro'] ?? $section->getContenu() ?? '')),
            'objectives' => implode("\n", array_map('strval', $content['objectives'] ?? [])),
            'steps' => implode("\n", $steps),
            'callouts' => implode("\n", $callouts),
        ];
    }

    /** @return array<string, mixed> */
    private function quizFormData(
        Formation $parent,
        ?Quiz $quiz,
        QuestionRepository $questions,
        ChoixRepository $choices,
        QuizCatalogService $catalog,
    ): array {
        if (!$quiz instanceof Quiz) {
            return [
                'title' => 'Nouveau quiz · ' . $parent->getTitre(),
                'type' => 'required',
                'sectionId' => '',
                'noteMinimale' => 80,
                'questions' => [[
                    'text' => '',
                    'choices' => [
                        ['text' => '', 'correct' => true],
                        ['text' => '', 'correct' => false],
                    ],
                ]],
            ];
        }

        $quizFormation = $quiz->getFormation();
        $type = $catalog->isSectionQuizFormation($quizFormation)
            ? 'section'
            : ($catalog->isBonusQuizFormation($quizFormation) ? 'bonus' : 'required');
        $questionRows = [];
        foreach ($questions->findBy(['quiz' => $quiz], ['ordre' => 'ASC', 'id' => 'ASC']) as $question) {
            $choiceRows = [];
            foreach ($choices->findBy(['question' => $question], ['ordre' => 'ASC', 'id' => 'ASC']) as $choice) {
                $choiceRows[] = ['text' => $choice->getTexte(), 'correct' => $choice->isEstCorrect()];
            }
            $questionRows[] = ['text' => $question->getTexte(), 'choices' => $choiceRows];
        }

        return [
            'title' => $quizFormation?->getTitre() ?: ($quiz->getSection()?->getTitre() ?: 'Quiz'),
            'type' => $type,
            'sectionId' => (string) ($quiz->getSection()?->getId() ?? ''),
            'noteMinimale' => $quiz->getNoteMinimale(),
            'questions' => $questionRows !== [] ? $questionRows : [[
                'text' => '',
                'choices' => [
                    ['text' => '', 'correct' => true],
                    ['text' => '', 'correct' => false],
                ],
            ]],
        ];
    }

    /** @param mixed[] $submitted */
    private function normalizeSubmittedQuestions(array $submitted): array
    {
        $result = [];
        foreach ($submitted as $question) {
            if (!is_array($question)) {
                continue;
            }
            $questionText = trim((string) ($question['text'] ?? ''));
            $choices = [];
            foreach (($question['choices'] ?? []) as $choice) {
                if (!is_array($choice)) {
                    continue;
                }
                $choiceText = trim((string) ($choice['text'] ?? ''));
                if ($choiceText === '') {
                    continue;
                }
                $choices[] = [
                    'text' => $choiceText,
                    'correct' => in_array((string) ($choice['correct'] ?? ''), ['1', 'true', 'on'], true),
                ];
            }
            if ($questionText !== '' || $choices !== []) {
                $result[] = ['text' => $questionText, 'choices' => $choices];
            }
        }

        return $result;
    }

    /** @param list<array{text:string,choices:list<array{text:string,correct:bool}>}> $questions */
    private function validateQuestions(array $questions): array
    {
        $errors = [];
        if ($questions === []) {
            return ['Ajoutez au moins une question au quiz.'];
        }

        foreach ($questions as $index => $question) {
            $number = $index + 1;
            if ($question['text'] === '') {
                $errors[] = 'Le texte de la question ' . $number . ' est obligatoire.';
            }
            if (count($question['choices']) < 2) {
                $errors[] = 'La question ' . $number . ' doit contenir au moins deux réponses.';
            }
            if (count(array_filter($question['choices'], static fn (array $choice): bool => $choice['correct'])) < 1) {
                $errors[] = 'Sélectionnez au moins une bonne réponse pour la question ' . $number . '.';
            }
        }

        return $errors;
    }

    private function sectionAlreadyHasQuiz(
        Formation $formation,
        Section $section,
        FormationRepository $formations,
        QuizRepository $quizzes,
        QuizCatalogService $catalog,
        ?Quiz $except = null,
    ): bool {
        foreach ($formations->findQuizFormationsForParent($formation->getId() ?? 0) as $quizFormation) {
            if (!$catalog->isSectionQuizFormation($quizFormation) || $catalog->getParentSectionId($quizFormation) !== $section->getId()) {
                continue;
            }

            foreach ($quizzes->findBy(['formation' => $quizFormation]) as $candidate) {
                if (!$except instanceof Quiz || $candidate->getId() !== $except->getId()) {
                    return true;
                }
            }
        }

        return false;
    }

    private function quizBelongsToFormation(Quiz $quiz, Formation $formation, QuizCatalogService $catalog): bool
    {
        if ($quiz->getFormation()?->getId() === $formation->getId()) {
            return true;
        }

        return $catalog->getParentFormationId($quiz->getFormation()) === $formation->getId();
    }

    private function buildQuizMarkers(Formation $parent, string $type, ?Section $section, string $key): string
    {
        $markers = [
            'FABOS_PARENT_FORMATION_ID=' . $parent->getId(),
            'FABOS_QUIZ_CONTEXT=' . ($type === 'section' ? 'section' : 'page'),
            'FABOS_BONUS=' . ($type === 'bonus' ? '1' : '0'),
            'FABOS_QUIZ_KEY=' . ($key !== '' ? $key : 'quiz'),
        ];

        if ($type === 'section' && $section?->getId() !== null) {
            $markers[] = 'FABOS_SECTION_ID=' . $section->getId();
        }

        return implode(';', $markers) . ';';
    }

    /** @return array<string, string> */
    private function buildLabelsPayload(Request $request): array
    {
        return [
            'descriptionTitle' => $this->requiredText($request, 'descriptionTitle', 'Description détaillée'),
            'objectivesTitle' => $this->requiredText($request, 'objectivesTitle', 'Objectifs pédagogiques'),
            'prerequisitesTitle' => $this->requiredText($request, 'prerequisitesTitle', 'Prérequis'),
            'materialTitle' => $this->requiredText($request, 'materialTitle', 'Matériel fourni'),
        ];
    }

    /** @return array<string, mixed> */
    private function buildJourneyPayload(Request $request): array
    {
        $cards = [];
        for ($index = 1; $index <= 3; ++$index) {
            $cards[] = [
                'title' => $this->requiredText($request, 'card' . $index . 'Title', 'Étape ' . $index),
                'text' => $this->requiredText($request, 'card' . $index . 'Text', ''),
            ];
        }

        return [
            'kicker' => $this->requiredText($request, 'kicker', 'Parcours guidé'),
            'title' => $this->requiredText($request, 'title', 'Parcours guidé'),
            'intro' => $this->requiredText($request, 'intro', ''),
            'cards' => $cards,
        ];
    }

    /** @return array<string, mixed> */
    private function buildProgramPayload(Request $request): array
    {
        return [
            'title' => $this->requiredText($request, 'title', 'Programme horaire'),
            'items' => $this->parsePipeLines((string) $request->request->get('items'), 3, ['time', 'title', 'description']),
        ];
    }

    /** @return array<string, mixed> */
    private function buildSessionsPayload(Request $request): array
    {
        $items = $this->parsePipeLines((string) $request->request->get('items'), 4, ['date', 'time', 'status', 'label']);
        foreach ($items as &$item) {
            $item['status'] = in_array($item['status'], ['available', 'full', 'cancelled'], true) ? $item['status'] : 'available';
        }
        unset($item);

        return [
            'title' => $this->requiredText($request, 'title', 'Prochaines sessions'),
            'items' => $items,
        ];
    }

    /** @return array<string, string> */
    private function buildPracticalPayload(Request $request): array
    {
        return [
            'title' => $this->requiredText($request, 'title', 'Formation pratique'),
            'requiredLabel' => $this->requiredText($request, 'requiredLabel', 'Validation pratique nécessaire'),
            'requiredDescription' => $this->requiredText($request, 'requiredDescription', ''),
            'requiredStatus' => $this->requiredText($request, 'requiredStatus', 'Présentiel à valider'),
            'optionalLabel' => $this->requiredText($request, 'optionalLabel', 'Formation pratique non nécessaire'),
            'optionalDescription' => $this->requiredText($request, 'optionalDescription', ''),
            'optionalStatus' => $this->requiredText($request, 'optionalStatus', 'Parcours en ligne suffisant'),
        ];
    }

    /** @return array<string, mixed> */
    private function buildRelatedPayload(Request $request): array
    {
        $items = [];
        for ($index = 1; $index <= 2; ++$index) {
            $items[] = [
                'badge' => $this->requiredText($request, 'item' . $index . 'Badge', ''),
                'title' => $this->requiredText($request, 'item' . $index . 'Title', ''),
                'description' => $this->requiredText($request, 'item' . $index . 'Description', ''),
                'button' => $this->requiredText($request, 'item' . $index . 'Button', ''),
            ];
        }

        return [
            'title' => $this->requiredText($request, 'title', 'Formations similaires'),
            'items' => $items,
        ];
    }

    /** @return list<array<string, string>> */
    private function parsePipeLines(string $value, int $expectedParts, array $keys): array
    {
        $rows = [];
        foreach (preg_split('/\R/u', trim($value)) ?: [] as $line) {
            $line = trim($line);
            if ($line === '') {
                continue;
            }
            $parts = array_map('trim', explode('|', $line, $expectedParts));
            $parts = array_pad($parts, $expectedParts, '');
            $row = [];
            foreach ($keys as $index => $key) {
                $row[$key] = $parts[$index] ?? '';
            }
            if (count(array_filter($row, static fn (string $item): bool => $item !== '')) > 0) {
                $rows[] = $row;
            }
        }

        return $rows;
    }

    /** @return list<array{type:string,title:string,text:string}> */
    private function parseCallouts(string $value): array
    {
        $rows = $this->parsePipeLines($value, 3, ['type', 'title', 'text']);
        foreach ($rows as &$row) {
            $row['type'] = in_array($row['type'], ['tip', 'warning', 'check', 'info'], true) ? $row['type'] : 'tip';
        }
        unset($row);

        return $rows;
    }

    /** @return list<string> */
    private function parseSimpleLines(string $value): array
    {
        return array_values(array_filter(array_map(
            static fn (string $line): string => trim($line),
            preg_split('/\R/u', trim($value)) ?: [],
        ), static fn (string $line): bool => $line !== ''));
    }

    /** @return array<string, mixed> */
    private function decodeJson(?string $value): array
    {
        if (!is_string($value) || trim($value) === '') {
            return [];
        }
        try {
            $decoded = json_decode($value, true, 512, JSON_THROW_ON_ERROR);
        } catch (\JsonException) {
            return ['intro' => trim($value), 'objectives' => [], 'steps' => [], 'callouts' => []];
        }

        return is_array($decoded) ? $decoded : [];
    }

    private function findVisibleFormation(int $id, FormationRepository $formations): Formation
    {
        $formation = $formations->find($id);
        if (!$formation instanceof Formation || TrainingQualificationService::isInternalCategory($formation->getCategorie())) {
            throw $this->createNotFoundException('Formation introuvable.');
        }

        return $formation;
    }

    private function assertCsrf(string $id, string $token): void
    {
        if (!$this->isCsrfTokenValid($id, $token)) {
            throw $this->createAccessDeniedException('Jeton de sécurité invalide. Rechargez la page puis réessayez.');
        }
    }

    private function requiredText(Request $request, string $key, string $fallback): string
    {
        $value = trim((string) $request->request->get($key));

        return $value !== '' ? $value : $fallback;
    }

    private function nullableText(mixed $value): ?string
    {
        $value = trim((string) $value);

        return $value !== '' ? $value : null;
    }

    private function limitedNullableText(mixed $value, int $maxLength): ?string
    {
        $value = $this->nullableText($value);
        if ($value === null) {
            return null;
        }

        return mb_substr($value, 0, $maxLength);
    }

    private function linesToStoredText(mixed $value, string $separator): ?string
    {
        $lines = $this->parseSimpleLines((string) $value);

        return $lines !== [] ? implode($separator, $lines) : null;
    }

    private function slugify(string $value): string
    {
        $value = mb_strtolower(trim($value));
        $value = iconv('UTF-8', 'ASCII//TRANSLIT//IGNORE', $value) ?: $value;
        $value = preg_replace('/[^a-z0-9]+/', '-', $value) ?? '';

        return trim($value, '-');
    }
}
