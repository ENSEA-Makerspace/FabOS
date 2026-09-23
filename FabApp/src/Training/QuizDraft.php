<?php

namespace App\Training;

use App\Entity\Choix;
use App\Entity\Question;

/**
 * Ce que le constructeur de quiz envoie, lu et vérifié (S182d).
 *
 * ⚠️ **Sorti du contrôleur pour être ÉPROUVABLE.** Ces deux méthodes décident de
 * ce qui est enregistré comme bonne réponse ; tant qu'elles étaient privées dans
 * `FormationContentAdminController`, rien ne pouvait les tester sans écrire un
 * quiz dans la base d'une installation réelle.
 *
 * ⚠️ **Le type de QUESTION s'appelle `kind`**, parce que le formulaire porte déjà
 * un `type` au niveau du quiz (obligatoire, section, bonus). Deux champs `type`
 * dans le même formulaire, c'est le prochain bug.
 *
 *   choice  à choix : une case « bonne réponse » par ligne ; unique ou multiple
 *           se DÉDUIT du nombre de cases cochées, comme avant
 *   order   remettre dans l'ordre : les lignes sont saisies DANS le bon ordre ;
 *           toutes sont « justes », c'est leur rang qui compte
 *   short   réponse courte : chaque ligne est une réponse ACCEPTÉE ; l'auteur
 *           liste les variantes qu'il admet (voir `QuizScorer::normalise`)
 */
final class QuizDraft
{
    public const CHOICE = 'choice';
    public const KINDS = [self::CHOICE, QuizScorer::ORDER, QuizScorer::SHORT];

    /**
     * @param mixed[] $submitted
     *
     * @return list<array{text: string, kind: string, choices: list<array{text: string, correct: bool}>}>
     */
    public function normalise(array $submitted): array
    {
        $result = [];
        foreach ($submitted as $question) {
            if (!is_array($question)) {
                continue;
            }

            $kind = (string) ($question['kind'] ?? self::CHOICE);
            if (!in_array($kind, self::KINDS, true)) {
                $kind = self::CHOICE;
            }

            $text = trim((string) ($question['text'] ?? ''));
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
                    // ⚠️ Pour `order` et `short`, TOUTES les lignes sont justes :
                    // une case à cocher n'y a pas de sens, et l'interface la cache.
                    'correct' => $kind !== self::CHOICE
                        || in_array((string) ($choice['correct'] ?? ''), ['1', 'true', 'on'], true),
                ];
            }

            if ($text !== '' || $choices !== []) {
                $result[] = ['text' => $text, 'kind' => $kind, 'choices' => $choices];
            }
        }

        return $result;
    }

    /**
     * @param list<array{text: string, kind: string, choices: list<array{text: string, correct: bool}>}> $questions
     *
     * @return list<string>
     */
    public function validate(array $questions): array
    {
        if ($questions === []) {
            return ['Ajoutez au moins une question au quiz.'];
        }

        $errors = [];
        foreach ($questions as $index => $question) {
            $n = $index + 1;
            if ($question['text'] === '') {
                $errors[] = 'Le texte de la question ' . $n . ' est obligatoire.';
            }

            $count = \count($question['choices']);
            switch ($question['kind']) {
                case QuizScorer::ORDER:
                    if ($count < 2) {
                        $errors[] = 'La question ' . $n . ' doit contenir au moins deux étapes à remettre dans l’ordre.';
                    }
                    break;
                case QuizScorer::SHORT:
                    if ($count < 1) {
                        $errors[] = 'La question ' . $n . ' doit contenir au moins une réponse acceptée.';
                    }
                    break;
                default:
                    if ($count < 2) {
                        $errors[] = 'La question ' . $n . ' doit contenir au moins deux réponses.';
                    }
                    if (\count(array_filter($question['choices'], static fn (array $c): bool => $c['correct'])) < 1) {
                        $errors[] = 'Sélectionnez au moins une bonne réponse pour la question ' . $n . '.';
                    }
            }
        }

        return $errors;
    }

    /**
     * La valeur écrite dans `QUESTION.type`.
     *
     * ⚠️ `choice` s'écrit `single` ou `multiple` selon les cases cochées — la
     * même règle qu'avant S182d, et celle que `QuizScorer` relit : la colonne et
     * la déduction concordent sur les 185 questions existantes.
     *
     * @param array{kind: string, choices: list<array{correct: bool}>} $question
     */
    public function storedType(array $question): string
    {
        if ($question['kind'] !== self::CHOICE) {
            return $question['kind'];
        }

        $correct = \count(array_filter($question['choices'], static fn (array $c): bool => $c['correct']));

        return $correct > 1 ? QuizScorer::MULTIPLE : QuizScorer::SINGLE;
    }

    /**
     * Une question enregistrée, telle que le constructeur doit la réafficher.
     *
     * @param list<Choix> $choices
     *
     * @return array{text: string, kind: string, choices: list<array{text: string, correct: bool}>}
     */
    public function fromQuestion(Question $question, array $choices): array
    {
        $type = QuizScorer::effectiveType($question, $choices);

        return [
            'text' => $question->getTexte(),
            'kind' => in_array($type, [QuizScorer::ORDER, QuizScorer::SHORT], true) ? $type : self::CHOICE,
            'choices' => array_map(
                static fn (Choix $c): array => ['text' => $c->getTexte(), 'correct' => $c->isEstCorrect()],
                $choices,
            ),
        ];
    }
}
