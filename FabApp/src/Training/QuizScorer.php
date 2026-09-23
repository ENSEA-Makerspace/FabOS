<?php

namespace App\Training;

use App\Entity\Choix;
use App\Entity\Question;
use App\Entity\Quiz;
use App\Repository\ChoixRepository;
use App\Repository\QuestionRepository;

/**
 * Ce qu'est une question de quiz, ce que la page en reçoit, et comment on la
 * corrige — **en un seul endroit** (S182c).
 *
 * 🔴 **Le défaut qui a fait naître ce fichier : les bonnes réponses partaient
 * dans la page.** `buildStoredQuizData()` mettait `'correct' => true/false` sur
 * chaque réponse, sérialisé en JSON dans `<script id="quiz-data">`. Mesuré le
 * 2026-09-23 : un visiteur NON CONNECTÉ recevait les trois bonnes réponses du
 * quiz 1 dans le code source. Ces quiz valident la théorie qui ouvre l'accès aux
 * machines ; le serveur recorrigeait bien — mais contre des réponses que le
 * navigateur avait déjà.
 * ✅ **Désormais le serveur est le SEUL à connaître les réponses.** `payload()`
 * n'en porte aucune, et c'est `score()` qui dit, APRÈS une tentative, quelles
 * questions sont justes.
 *
 * 🔴 **Et la correction ne révèle PAS les bonnes réponses**, d'après la planche
 * de référence `lms-quiz-result-retry` : elle liste les points À REVOIR, pas la
 * solution. Avant, le quiz affichait la réponse attendue de chaque question
 * ratée — on échouait une fois, on lisait, on repassait à 100 %.
 * 🅿️ Ce qui reste vrai, et qu'il faut dire : avec des reprises illimitées et une
 * correction question par question, un apprenant obstiné finit par trouver par
 * élimination. Le quiz mesure qu'on a relu, pas qu'on a compris ; la limite des
 * tentatives est une décision de pédagogie, pas de code.
 *
 * ⚠️ **Quatre types, d'après la planche `lms-quiz-question-types`**, et tous
 * tiennent dans le schéma existant — aucune migration :
 *   single    un choix ;             juste si l'unique bonne réponse est cochée
 *   multiple  plusieurs choix ;      juste si l'ensemble coché = l'ensemble attendu
 *   order     remettre dans l'ordre ; juste si la suite = les réponses triées
 *             par `CHOIX.ordre` ; la page les reçoit MÉLANGÉES
 *   short     réponse courte ;       juste si le texte, normalisé, égale l'une des
 *             réponses acceptées ; la page ne reçoit AUCUN choix — ce sont les
 *             réponses elles-mêmes
 *
 * ⚠️ **`single`/`multiple` restent DÉDUITS des bonnes réponses**, comme avant :
 * mesuré le 2026-09-23, la colonne `QUESTION.type` et la déduction concordent
 * sur les 185 questions. Seuls `order` et `short` se lisent dans la colonne, parce
 * qu'ils ne se déduisent de rien.
 */
final class QuizScorer
{
    public const SINGLE = 'single';
    public const MULTIPLE = 'multiple';
    public const ORDER = 'order';
    public const SHORT = 'short';

    public const TYPES = [self::SINGLE, self::MULTIPLE, self::ORDER, self::SHORT];

    public function __construct(
        private readonly QuestionRepository $questions,
        private readonly ChoixRepository $choices,
    ) {
    }

    /** @param list<Choix> $choices */
    public static function effectiveType(Question $question, array $choices): string
    {
        $stored = $question->getType();
        if ($stored === self::ORDER || $stored === self::SHORT) {
            return $stored;
        }

        $correct = \count(array_filter($choices, static fn (Choix $c): bool => $c->isEstCorrect()));

        return $correct > 1 ? self::MULTIPLE : self::SINGLE;
    }

    /**
     * Les questions telles que la PAGE doit les recevoir — sans une seule réponse.
     *
     * @return list<array{id: string, order: int, text: string, type: string, choices: list<array{id: string, text: string}>}>
     */
    public function payload(Quiz $quiz): array
    {
        $rows = [];
        foreach ($this->questions->findBy(['quiz' => $quiz], ['ordre' => 'ASC']) as $question) {
            $choices = $this->choices->findBy(['question' => $question], ['ordre' => 'ASC']);
            if ($choices === []) {
                continue;
            }

            $type = self::effectiveType($question, $choices);
            $public = array_map(
                static fn (Choix $c): array => ['id' => (string) $c->getId(), 'text' => $c->getTexte()],
                $choices,
            );

            if ($type === self::ORDER) {
                // 🔴 Mélangées CÔTÉ SERVEUR : envoyées dans l'ordre de la base,
                // elles arriveraient déjà dans la bonne suite.
                $public = self::shuffledAway($public);
            } elseif ($type === self::SHORT) {
                // 🔴 Aucun choix : pour une réponse courte, les « choix » SONT les
                // réponses acceptées.
                $public = [];
            }

            $rows[] = [
                'id' => (string) $question->getId(),
                'order' => $question->getOrdre(),
                'text' => $question->getTexte(),
                'type' => $type,
                'choices' => $public,
            ];
        }

        return $rows;
    }

    /**
     * Corrige une tentative.
     *
     * ⚠️ `review` dit, pour chaque question, si elle est juste — et RIEN d'autre.
     * Pas de réponse attendue : voir la note de classe.
     *
     * @param array<string, mixed> $answers id de question => réponse (liste d'ids, ou texte)
     *
     * @return array{questionCount: int, correctCount: int, score: int, review: list<array{id: string, correct: bool}>}
     */
    public function score(Quiz $quiz, array $answers): array
    {
        $review = [];
        $correctCount = 0;

        foreach ($this->questions->findBy(['quiz' => $quiz], ['ordre' => 'ASC']) as $question) {
            $choices = $this->choices->findBy(['question' => $question], ['ordre' => 'ASC']);
            if ($choices === []) {
                continue;
            }

            $answer = $answers[(string) $question->getId()] ?? [];
            $ok = $this->isCorrect(self::effectiveType($question, $choices), $choices, $answer);
            if ($ok) {
                ++$correctCount;
            }
            $review[] = ['id' => (string) $question->getId(), 'correct' => $ok];
        }

        $count = \count($review);

        return [
            'questionCount' => $count,
            'correctCount' => $correctCount,
            'score' => $count === 0 ? 0 : (int) round(($correctCount / $count) * 100),
            'review' => $review,
        ];
    }

    /**
     * Le verdict d'UNE question.
     *
     * ⚠️ Public pour une raison précise : la sonde S182c doit pouvoir éprouver
     * `order` et `short` sur des choix construits en mémoire, sans écrire de
     * question dans la base d'une installation réelle.
     *
     * @param list<Choix> $choices
     */
    public function isCorrect(string $type, array $choices, mixed $answer): bool
    {
        if ($type === self::SHORT) {
            $given = self::normalise(is_array($answer) ? (string) ($answer[0] ?? '') : (string) $answer);
            if ($given === '') {
                return false;
            }
            foreach ($choices as $choice) {
                if ($choice->isEstCorrect() && self::normalise($choice->getTexte()) === $given) {
                    return true;
                }
            }

            return false;
        }

        $ids = array_map(static fn (Choix $c): string => (string) $c->getId(), $choices);
        $given = array_values(array_filter(
            array_map('strval', is_array($answer) ? $answer : [$answer]),
            static fn (string $id): bool => in_array($id, $ids, true),
        ));

        if ($type === self::ORDER) {
            // La suite attendue est celle de `CHOIX.ordre` — `$choices` est déjà
            // trié ainsi. Une suite incomplète ou répétée n'est pas « presque » juste.
            return $given === $ids;
        }

        $expected = array_map(
            static fn (Choix $c): string => (string) $c->getId(),
            array_values(array_filter($choices, static fn (Choix $c): bool => $c->isEstCorrect())),
        );
        $given = array_values(array_unique($given));
        sort($expected, SORT_STRING);
        sort($given, SORT_STRING);

        return $expected === $given;
    }

    /**
     * ⚠️ **Casse, accents, espaces et ponctuation finale ignorés.** « Lunettes de
     * protection. » et « lunettes de protection » sont la même réponse ; les
     * refuser apprendrait à l'apprenant que le quiz note la typographie.
     * 🅿️ Ce qui n'est PAS toléré : une faute de frappe. Une distance d'édition
     * accepterait aussi « lunettes » pour « lunettes de protection » dans d'autres
     * cas ; c'est à l'auteur d'ajouter les variantes qu'il accepte.
     */
    public static function normalise(string $text): string
    {
        $text = mb_strtolower(trim($text));
        $text = \Normalizer::normalize($text, \Normalizer::FORM_D) ?: $text;
        $text = (string) preg_replace('/\p{Mn}+/u', '', $text);
        $text = (string) preg_replace('/[\s\x{00A0}]+/u', ' ', $text);

        return trim($text, " .!?;:,'\"«»");
    }

    /**
     * Un mélange qui n'est JAMAIS la bonne suite (dès qu'il y a deux éléments).
     *
     * ⚠️ Un `shuffle()` nu rend la bonne suite une fois sur n! — sur trois
     * éléments, une fois sur six. La question serait alors résolue d'avance.
     *
     * @template T
     * @param list<T> $items
     * @return list<T>
     */
    private static function shuffledAway(array $items): array
    {
        if (\count($items) < 2) {
            return $items;
        }

        $original = $items;
        do {
            shuffle($items);
        } while ($items === $original);

        return $items;
    }
}
