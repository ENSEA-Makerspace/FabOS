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
use App\Form\FormationContent\FormationGeneralType;
use App\Form\FormationContent\FormationSectionType;
use App\Form\FormationContent\FormationLabelsType;
use App\Form\FormationContent\FormationPracticalType;
use App\Service\FormationPageContentService;
use App\Service\QuizCatalogService;
use App\Service\TrainingQualificationService;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\Form\FormError;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;

#[Route('/admin/formations')]
#[IsGranted('ROLE_ADMIN')]
final class FormationContentAdminController extends AbstractController
{
    /**
     * 🔴 **S149 — les neuf cartes sont repliées, et l'écran ouvre sur le choix.**
     * `tools/form_quality.py` mesurait 35 champs visibles à l'arrivée, réparties
     * sur sept formulaires : le pire écran du produit, et de loin. La règle 1 de
     * `docs/FORM-DESIGN.md` demande le cas courant ; cet éditeur n'en a pas un
     * seul, donc il ouvre sur la liste de ce qu'on peut modifier.
     *
     * ⚠️ **`ouvrir` est le SEUL moyen d'ouvrir un repli depuis le serveur.** Un
     * fragment (`#sessions`) ne quitte jamais le navigateur : le contrôleur ne
     * peut pas le lire, et un gabarit qui prétendrait le faire mentirait. Les
     * deux chemins qui reviennent ici après une action — la redirection qui suit
     * un enregistrement, et le refus qui re-rend la page — passent donc par ce
     * paramètre ou par `$submitted`, jamais par l'ancre.
     */
    #[Route('/{id}/content', name: 'app_admin_formation_content', requirements: ['id' => '\\d+'], methods: ['GET'])]
    public function editor(
        int $id,
        Request $request,
        FormationRepository $formations,
        SectionRepository $sections,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        FormationPageContentService $pageContent,
        QuizCatalogService $catalog,
    ): Response {
        $formation = $this->findVisibleFormation($id, $formations);

        return $this->renderEditor(
            $formation,
            $formations,
            $sections,
            $quizzes,
            $questions,
            $pageContent,
            $catalog,
            [],
            (string) $request->query->get('ouvrir'),
        );
    }

    /**
     * ⚠️ **S147, J-22 — la carte « Général » passe par un `FormType`.**
     *
     * 🔴 **Ce que la conversion change, c'est le refus.** Avant, un titre vide ou
     * trop long posait un flash en haut de page et **redirigeait** : les dix
     * autres champs repartaient chercher leur valeur en base, donc une description
     * qu'on venait d'écrire était perdue. Maintenant le formulaire est re-rendu
     * tel que soumis, l'erreur est sur le champ, et rien n'est à retaper.
     *
     * ⚠️ Trois valeurs étaient TRONQUÉES en silence (`categorie`, `duree`,
     * `formateur` passaient par `mb_substr()`). Elles sont refusées, pas coupées.
     */
    #[Route('/{id}/content/general', name: 'app_admin_formation_content_general', requirements: ['id' => '\\d+'], methods: ['POST'])]
    public function updateGeneral(
        int $id,
        Request $request,
        FormationRepository $formations,
        SectionRepository $sections,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        FormationPageContentService $pageContent,
        QuizCatalogService $catalog,
        EntityManagerInterface $entityManager,
    ): Response {
        $formation = $this->findVisibleFormation($id, $formations);

        $form = $this->createForm(FormationGeneralType::class, $this->generalData($formation, $pageContent));
        $form->handleRequest($request);

        if (!$form->isSubmitted() || !$form->isValid()) {
            return $this->renderEditor($formation, $formations, $sections, $quizzes, $questions, $pageContent, $catalog, ['general' => $form]);
        }

        /** @var array<string, mixed> $data */
        $data = $form->getData();

        $formation
            ->setTitre(trim((string) $data['titre']))
            ->setDescription($this->nullableText($data['description']))
            ->setCategorie($this->nullableText($data['categorie']))
            ->setNiveau($data['niveau'] !== null ? (int) $data['niveau'] : null)
            ->setDuree($this->nullableText($data['duree']))
            ->setFormateur($this->nullableText($data['formateur']))
            ->setPlacesTotales($data['placesTotales'] !== null ? (int) $data['placesTotales'] : null)
            ->setObjectifs($this->linesToStoredText($data['objectifs'], '. '))
            ->setPrerequis($this->linesToStoredText($data['prerequis'], "\n"))
            ->setMaterielFourni($this->linesToStoredText($data['materielFourni'], ', '));

        $entityManager->flush();
        $this->addFlash('success', 'flash.les_informations_generales_de_la_formation');

        // ⚠️ `ouvrir` en plus du fragment : depuis S149 la carte est repliée, et le
        // fragment seul ramènerait l'opérateur devant un repli fermé sur ce qu'il
        // vient d'enregistrer. Le fragment reste pour le défilement.
        return $this->redirectToRoute('app_admin_formation_content', ['id' => $id, 'ouvrir' => 'general', '_fragment' => 'general'], Response::HTTP_SEE_OTHER);
    }

    /**
     * ⚠️ **Deux blocs sur six ont un `FormType`, et pas les quatre autres.**
     *
     * `labels` et `practical` sont des listes de champs — un nom fixe par valeur —
     * donc ils se convertissent. Les quatre autres ne sont pas des questionnaires :
     * `journey` et `related` construisent leur nom dans une boucle (`card2Title`,
     * `item1Badge`), `program` et `sessions` compressent une table à trois ou
     * quatre colonnes dans un textarea au format `a | b | c`. Un `FormType` posé
     * dessus tel quel rendrait la table en liste plate et casserait l'écran, ce qui
     * est précisément l'erreur que la classification de J-22 a écrite noir sur
     * blanc pour `/admin/features`. Ils restent lus à la main, sciemment.
     */
    #[Route('/{id}/content/block/{block}', name: 'app_admin_formation_content_block', requirements: ['id' => '\\d+', 'block' => 'labels|journey|program|sessions|practical|related'], methods: ['POST'])]
    public function updateBlock(
        int $id,
        string $block,
        Request $request,
        FormationRepository $formations,
        SectionRepository $sections,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        FormationPageContentService $pageContent,
        QuizCatalogService $catalog,
    ): Response {
        $formation = $this->findVisibleFormation($id, $formations);

        $type = match ($block) {
            'labels' => FormationLabelsType::class,
            'practical' => FormationPracticalType::class,
            default => null,
        };

        if ($type !== null) {
            $form = $this->createForm($type, $pageContent->getContent($formation)[$block]);
            $form->handleRequest($request);

            if (!$form->isSubmitted() || !$form->isValid()) {
                return $this->renderEditor($formation, $formations, $sections, $quizzes, $questions, $pageContent, $catalog, [$block => $form]);
            }

            /** @var array<string, mixed> $data */
            $data = $form->getData();
            $payload = $block === 'labels' ? $this->buildLabelsPayload($data) : $this->buildPracticalPayload($data);
        } else {
            $this->assertCsrf('formation_content_block_' . $id . '_' . $block, (string) $request->request->get('_token'));

            $payload = match ($block) {
                'journey' => $this->buildJourneyPayload($request),
                'program' => $this->buildProgramPayload($request),
                'sessions' => $this->buildSessionsPayload($request),
                'related' => $this->buildRelatedPayload($request),
                default => throw $this->createNotFoundException('Bloc de contenu inconnu.'),
            };
        }

        $pageContent->saveBlock($formation, $block, $payload);
        $this->addFlash('success', 'flash.le_bloc_de_texte_a_ete');

        return $this->redirectToRoute('app_admin_formation_content', ['id' => $id, 'ouvrir' => $block, '_fragment' => $block], Response::HTTP_SEE_OTHER);
    }

    /**
     * Le rendu de l'éditeur, appelé par la page ET par les trois handlers.
     *
     * 🔴 **C'est ce qui permet de ne pas rediriger sur un refus.** Un handler qui
     * refuse doit rendre la MÊME page avec SON formulaire soumis à la place du
     * formulaire vierge — d'où `$submitted`, indexé par bloc.
     *
     * 🔴 **Et c'est ce qui décide quel repli est ouvert.** Depuis S149 les neuf
     * cartes sont fermées à l'arrivée ; un refus qui ne rouvrirait pas la sienne
     * cacherait à la fois la saisie et l'erreur, ce que la règle 1 interdit en
     * toutes lettres. Le bloc refusé gagne donc sur `$requested` : c'est le seul
     * cas où l'opérateur n'a pas le choix de ce qu'il regarde.
     *
     * ⚠️ **Le drapeau part d'ici, pas du gabarit.** `form.vars.submitted` lu dans
     * le Twig marcherait en `dev` et vaudrait `null` en `prod`, faute de
     * `strict_variables` — un repli fermé sans un mot, sur des erreurs invisibles.
     *
     * @param array<string, \Symfony\Component\Form\FormInterface> $submitted
     */
    private function renderEditor(
        Formation $formation,
        FormationRepository $formations,
        SectionRepository $sections,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        FormationPageContentService $pageContent,
        QuizCatalogService $catalog,
        array $submitted = [],
        string $requested = '',
    ): Response {
        $content = $pageContent->getContent($formation);

        $forms = [
            'general' => $submitted['general'] ?? $this->createForm(FormationGeneralType::class, $this->generalData($formation, $pageContent)),
            'labels' => $submitted['labels'] ?? $this->createForm(FormationLabelsType::class, $content['labels']),
            'practical' => $submitted['practical'] ?? $this->createForm(FormationPracticalType::class, $content['practical']),
        ];

        return $this->render('site/admin-formation-content.html.twig', [
            'formation' => $formation,
            'pageContent' => $content,
            'forms' => array_map(static fn ($form) => $form->createView(), $forms),
            'sections' => $sections->findJourneySections($formation),
            'quizRows' => $this->buildQuizRows($formation, $formations, $quizzes, $questions, $catalog),
            'openBlock' => array_key_first($submitted) ?? $this->foldableBlock($requested),
        ]);
    }

    /**
     * Le nom du repli à ouvrir, ou `null`.
     *
     * ⚠️ **La liste est blanche, et elle est ici plutôt que dans le gabarit.** Le
     * Twig ne fait que comparer `openBlock` à neuf littéraux, donc rien ne fuit
     * dans le HTML ; mais une valeur libre venue de l'URL qui traverse le
     * contrôleur sans jamais être nommée est une invitation à l'y afficher un
     * jour. Les neuf identifiants de carte sont écrits une fois, ici.
     */
    private function foldableBlock(string $requested): ?string
    {
        $blocks = ['general', 'labels', 'journey', 'program', 'sessions', 'practical', 'related', 'sections', 'quizzes'];

        return in_array($requested, $blocks, true) ? $requested : null;
    }

    /**
     * ⚠️ Les trois listes sont éditées **une par ligne** et stockées recollées avec
     * trois séparateurs différents. C'est pour ça que la carte a son propre type et
     * ne peut pas réutiliser `FormationAdminType`, qui édite les mêmes colonnes en
     * texte libre.
     *
     * @return array<string, mixed>
     */
    private function generalData(Formation $formation, FormationPageContentService $pageContent): array
    {
        $lists = $pageContent->getContent($formation)['lists'];

        return [
            'titre' => $formation->getTitre(),
            'description' => $formation->getDescription(),
            'categorie' => $formation->getCategorie(),
            'niveau' => $formation->getNiveau(),
            'duree' => $formation->getDuree(),
            'formateur' => $formation->getFormateur(),
            'placesTotales' => $formation->getPlacesTotales(),
            'objectifs' => implode("\n", $lists['objectives']),
            'prerequis' => implode("\n", $lists['prerequisites']),
            'materielFourni' => implode("\n", $lists['material']),
        ];
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
            $this->addFlash('warning', 'flash.ce_quiz_historique_est_directement_rattache');
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
        // ⚠️ **S148, J-22.** L'écran gardait déjà ce qui avait été tapé — c'est un des
        // rares qui le faisait — mais ses cinq refus étaient des phrases françaises
        // en dur listées EN HAUT de page : l'opérateur devait relire la liste pour
        // deviner lequel des sept champs était en cause. Ils sont sur le champ.
        $form = $this->createForm(FormationSectionType::class, $this->sectionFormData($section));
        $form->handleRequest($request);

        if ($form->isSubmitted() && $form->isValid()) {
            /** @var array<string, mixed> $data */
            $data = $form->getData();

            // 🔴 Le seul refus qui ne peut PAS être une contrainte de champ : il
            // dépend du résultat du découpage, pas du texte. Il se pose quand même
            // sur `steps`, là où l'opérateur regarde.
            $steps = $this->parsePipeLines(trim((string) ($data['steps'] ?? '')), 2, ['title', 'text']);
            if ($steps === []) {
                $form->get('steps')->addError(new FormError('Ajoutez au moins une étape au format Titre | Texte.'));
            }

            if ($form->isValid()) {
                $content = [
                    'intro' => trim((string) $data['intro']),
                    'objectives' => $this->parseSimpleLines(trim((string) ($data['objectives'] ?? ''))),
                    'steps' => $steps,
                    'callouts' => $this->parseCallouts(trim((string) ($data['callouts'] ?? ''))),
                ];

                $section
                    ->setTitre(trim((string) $data['titre']))
                    ->setOrdre(max(1, (int) $data['ordre']))
                    ->setVideoUrl($this->nullableText($data['videoUrl'] ?? null))
                    ->setContenu(json_encode($content, JSON_UNESCAPED_UNICODE | JSON_UNESCAPED_SLASHES | JSON_THROW_ON_ERROR));

                if ($isNew) {
                    $entityManager->persist($section);
                }
                $entityManager->flush();

                $this->addFlash('success', $isNew ? 'flash.section_creee' : 'flash.section_mise_a_jour');

                return $this->redirectToRoute('app_admin_formation_content', ['id' => $formation->getId(), 'ouvrir' => 'sections', '_fragment' => 'sections'], Response::HTTP_SEE_OTHER);
            }
        }

        return $this->render('site/admin-formation-section-form.html.twig', [
            'formation' => $formation,
            'section' => $section,
            'form' => $form->createView(),
            'isNew' => $isNew,
        ], $form->isSubmitted() ? new Response(status: Response::HTTP_UNPROCESSABLE_ENTITY) : null);
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

                $this->addFlash('success', $isNew ? 'flash.quiz_cree' : 'flash.quiz_mis_a_jour');

                return $this->redirectToRoute('app_admin_formation_content', ['id' => $formation->getId(), 'ouvrir' => 'quizzes', '_fragment' => 'quizzes'], Response::HTTP_SEE_OTHER);
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

    /**
     * ⚠️ **Vider un titre ne le refuse pas, il le remet par défaut** — c'était déjà
     * vrai avant la conversion et ça le reste. Seule la source change : les valeurs
     * viennent du formulaire validé, plus de la requête brute.
     *
     * @param array<string, mixed> $data
     *
     * @return array<string, string>
     */
    private function buildLabelsPayload(array $data): array
    {
        return [
            'descriptionTitle' => $this->textOrDefault($data, 'descriptionTitle', 'Description détaillée'),
            'objectivesTitle' => $this->textOrDefault($data, 'objectivesTitle', 'Objectifs pédagogiques'),
            'prerequisitesTitle' => $this->textOrDefault($data, 'prerequisitesTitle', 'Prérequis'),
            'materialTitle' => $this->textOrDefault($data, 'materialTitle', 'Matériel fourni'),
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

    /**
     * ⚠️ **S149 — le programme n'arrive plus en `heure | titre | description`.**
     * L'écran présentait une table à trois colonnes sous la forme d'un textarea
     * dont il fallait retenir l'ordre des colonnes. Il envoie maintenant des
     * lignes indexées (`items[0][time]`, …) et le découpage au tuyau disparaît
     * pour ce bloc. `parsePipeLines()` reste : l'éditeur de SECTION s'en sert
     * toujours pour ses gestes clés et ses encadrés.
     *
     * @return array<string, mixed>
     */
    private function buildProgramPayload(Request $request): array
    {
        return [
            'title' => $this->requiredText($request, 'title', 'Programme horaire'),
            'items' => $this->rowsFromRequest($request, ['time', 'title', 'description'], ['time', 'title', 'description']),
        ];
    }

    /**
     * 🔴 **Le repli sur `available` était une perte de donnée silencieuse.** Le
     * statut se tapait à la main dans la troisième colonne du textarea, et toute
     * valeur non reconnue — `annulé`, `cancelled ` avec une espace, `canceled` —
     * devenait `available` sans un mot : une session annulée réapparaissait
     * ouverte sur la page publique. La liste déroulante du gabarit rend le cas
     * impossible ; le repli reste ici parce qu'un POST n'est pas un formulaire,
     * et qu'un champ de choix se falsifie.
     *
     * ⚠️ **`status` ne compte PAS pour décider qu'une ligne est vide.** Un
     * `<select>` renvoie toujours une valeur : sans cette distinction, les trois
     * lignes vides offertes à chaque rendu seraient enregistrées comme trois
     * sessions fantômes « disponibles » à chaque passage.
     *
     * @return array<string, mixed>
     */
    private function buildSessionsPayload(Request $request): array
    {
        $items = $this->rowsFromRequest($request, ['date', 'time', 'status', 'label'], ['date', 'time', 'label']);
        foreach ($items as &$item) {
            $item['status'] = in_array($item['status'], ['available', 'full', 'cancelled'], true) ? $item['status'] : 'available';
        }
        unset($item);

        return [
            'title' => $this->requiredText($request, 'title', 'Prochaines sessions'),
            'items' => $items,
        ];
    }

    /**
     * Les lignes d'un éditeur de table, lues depuis `items[n][colonne]`.
     *
     * ⚠️ **`InputBag::all('items')` lève une `BadRequestException` quand la valeur
     * n'est pas un tableau**, et la valeur postée par une page ouverte AVANT ce
     * changement est encore le textarea, donc une chaîne. Un 400 nu sur une
     * soumission par ailleurs légitime est un mystère pour l'opérateur ; on lit
     * `all()` puis on vérifie, ce qui donne le même résultat qu'un envoi vide.
     *
     * ⚠️ Une ligne dont toutes les colonnes `$meaningful` sont vides est jetée :
     * c'est ce qui permet d'offrir des lignes neuves à chaque rendu, et c'est
     * aussi la façon de supprimer une ligne — on l'efface. Faute de contrôleur
     * Stimulus « ajouter une ligne », un bouton de suppression serait soit un
     * `disabled` (interdit), soit un aller-retour serveur de plus.
     *
     * @param list<string> $keys       toutes les colonnes de la ligne
     * @param list<string> $meaningful celles dont le vide signifie « ligne vide »
     *
     * @return list<array<string, string>>
     */
    private function rowsFromRequest(Request $request, array $keys, array $meaningful): array
    {
        $posted = $request->request->all();
        $submitted = isset($posted['items']) && is_array($posted['items']) ? $posted['items'] : [];

        $rows = [];
        foreach ($submitted as $line) {
            if (!is_array($line)) {
                continue;
            }

            $row = [];
            foreach ($keys as $key) {
                $value = $line[$key] ?? '';
                $row[$key] = is_scalar($value) ? trim((string) $value) : '';
            }

            foreach ($meaningful as $key) {
                if ($row[$key] !== '') {
                    $rows[] = $row;
                    break;
                }
            }
        }

        return $rows;
    }

    /**
     * @param array<string, mixed> $data
     *
     * @return array<string, string>
     */
    private function buildPracticalPayload(array $data): array
    {
        return [
            'title' => $this->textOrDefault($data, 'title', 'Formation pratique'),
            'requiredLabel' => $this->textOrDefault($data, 'requiredLabel', 'Validation pratique nécessaire'),
            'requiredDescription' => $this->textOrDefault($data, 'requiredDescription', ''),
            'requiredStatus' => $this->textOrDefault($data, 'requiredStatus', 'Présentiel à valider'),
            'optionalLabel' => $this->textOrDefault($data, 'optionalLabel', 'Formation pratique non nécessaire'),
            'optionalDescription' => $this->textOrDefault($data, 'optionalDescription', ''),
            'optionalStatus' => $this->textOrDefault($data, 'optionalStatus', 'Parcours en ligne suffisant'),
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

    /**
     * La même règle que `requiredText`, mais sur les données d'un formulaire validé.
     *
     * @param array<string, mixed> $data
     */
    private function textOrDefault(array $data, string $key, string $fallback): string
    {
        $value = trim((string) ($data[$key] ?? ''));

        return $value !== '' ? $value : $fallback;
    }

    private function nullableText(mixed $value): ?string
    {
        $value = trim((string) $value);

        return $value !== '' ? $value : null;
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
