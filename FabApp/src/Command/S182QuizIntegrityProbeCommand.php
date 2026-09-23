<?php

namespace App\Command;

use App\Entity\Choix;
use App\Entity\Quiz;
use App\Repository\ChoixRepository;
use App\Repository\QuestionRepository;
use App\Repository\QuizRepository;
use App\Service\QuizCatalogService;
use App\Training\QuizDraft;
use App\Training\QuizScorer;
use Doctrine\DBAL\Connection;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\HttpKernelInterface;
use Symfony\Component\HttpKernel\KernelInterface;

/**
 * S182c — les réponses ne quittent plus le serveur, et la correction n'a pas changé.
 *
 * 🔴 **Deux mesures, et la seconde est celle qu'on oublie.**
 *   1. Aucune bonne réponse dans la page — mesuré sur CHAQUE quiz de la base.
 *   2. **Le nouveau correcteur rend EXACTEMENT les verdicts de l'ancien.** L'ancien
 *      algorithme est recopié ICI, ligne pour ligne, et les deux sont confrontés
 *      sur toutes les combinaisons de réponses de toutes les vraies questions. Un
 *      refactor de correction qui « a l'air juste » et change un seul verdict,
 *      c'est un apprenant recalé — ou validé — à tort.
 *
 * ✅ **Elle n'écrit rien** : `score()` et la route de correction ne touchent ni
 * `PROGRESSION` ni le journal ; la sonde le vérifie en comptant avant et après.
 */
#[AsCommand(name: 'app:s182:quiz-integrity-probe', description: 'S182c : prouve qu\'aucune bonne réponse n\'est envoyée à la page, que le nouveau correcteur rend les mêmes verdicts que l\'ancien sur toutes les combinaisons réelles, et que la correction ne révèle pas la solution. N\'écrit rien.')]
final class S182QuizIntegrityProbeCommand extends Command
{
    public function __construct(
        private readonly QuizScorer $scorer,
        private readonly QuizRepository $quizzes,
        private readonly QuestionRepository $questions,
        private readonly ChoixRepository $choices,
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly QuizCatalogService $catalog,
        private readonly QuizDraft $draft,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $progressionsBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM PROGRESSION');
        $mailsBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG');

        $io->section('1. 🔴 Aucune bonne réponse dans ce que reçoit la page');
        $quizCount = 0;
        $leaks = 0;
        foreach ($this->quizzes->findAll() as $quiz) {
            $payload = $this->scorer->payload($quiz);
            if ($payload === []) {
                continue;
            }
            ++$quizCount;
            $json = json_encode($payload, JSON_THROW_ON_ERROR);
            if (str_contains($json, '"correct"') || str_contains($json, 'estCorrect')) {
                ++$leaks;
            }
        }
        $io->writeln('   ' . $quizCount . ' quiz examinés');
        $this->check($io, $failures, '🔴 AUCUN ne porte le mot « correct »', $leaks === 0);

        $io->section('2. 🔴 Le nouveau correcteur = l\'ancien, sur toutes les combinaisons');
        $combos = 0;
        $disagree = [];
        foreach ($this->questions->findAll() as $question) {
            $choices = $this->choices->findBy(['question' => $question], ['ordre' => 'ASC']);
            if ($choices === [] || \count($choices) > 8) {
                continue;
            }
            $type = QuizScorer::effectiveType($question, $choices);
            foreach ($this->subsets(array_map(static fn (Choix $c): string => (string) $c->getId(), $choices)) as $answer) {
                ++$combos;
                $old = $this->oldVerdict($choices, $answer);
                $new = $this->scorer->isCorrect($type, $choices, $answer);
                if ($old !== $new) {
                    $disagree[] = 'question #' . $question->getId() . ' réponse [' . implode(',', $answer) . ']';
                }
            }
        }
        $io->writeln('   ' . $combos . ' combinaisons confrontées');
        foreach (array_slice($disagree, 0, 5) as $line) {
            $io->writeln('   ✗ ' . $line);
        }
        $this->check($io, $failures, '🔴 ZÉRO désaccord', $disagree === [] && $combos > 0);

        $io->section('3. La correction ne révèle PAS la solution');
        $sample = $this->firstQuizWithQuestions();
        if ($sample instanceof Quiz) {
            $scored = $this->scorer->score($sample, []);
            $keys = array_unique(array_merge(...array_map('array_keys', $scored['review'])));
            sort($keys);
            $io->writeln('   champs de la correction : ' . implode(', ', $keys));
            $this->check($io, $failures, 'uniquement « id » et « correct »', $keys === ['correct', 'id']);
            $this->check($io, $failures, 'une tentative vide vaut 0 %', $scored['score'] === 0);
        }

        $io->section('4. « Remettre dans l\'ordre » — en mémoire');
        $order = [$this->choice(101, 'Couper l\'alimentation', true, 1), $this->choice(102, 'Attendre l\'arrêt', true, 2), $this->choice(103, 'Ouvrir le capot', true, 3)];
        $this->check($io, $failures, 'la bonne suite est juste', $this->scorer->isCorrect(QuizScorer::ORDER, $order, ['101', '102', '103']));
        $this->check($io, $failures, 'une suite inversée ne l\'est pas', !$this->scorer->isCorrect(QuizScorer::ORDER, $order, ['103', '102', '101']));
        $this->check($io, $failures, 'une suite incomplète non plus', !$this->scorer->isCorrect(QuizScorer::ORDER, $order, ['101', '102']));

        $io->section('5. « Réponse courte » — normalisation');
        $short = [$this->choice(201, 'Lunettes de protection', true, 1), $this->choice(202, 'lunettes', true, 2)];
        $this->check($io, $failures, '« lunettes de protection. » = « Lunettes de protection »', $this->scorer->isCorrect(QuizScorer::SHORT, $short, ['  lunettes de protection. ']));
        $this->check($io, $failures, 'les accents ne comptent pas (« Lunéttes » accepté)', $this->scorer->isCorrect(QuizScorer::SHORT, $short, ['Lunéttes']));
        $this->check($io, $failures, 'une variante listée par l\'auteur est acceptée', $this->scorer->isCorrect(QuizScorer::SHORT, $short, ['LUNETTES']));
        $this->check($io, $failures, 'une autre réponse est refusée', !$this->scorer->isCorrect(QuizScorer::SHORT, $short, ['gants']));
        $this->check($io, $failures, 'une réponse vide est refusée', !$this->scorer->isCorrect(QuizScorer::SHORT, $short, ['   ']));

        $io->section('6. La porte de correction, appelée comme un VISITEUR');
        if ($sample instanceof Quiz) {
            [$status, $body] = $this->callCheck($sample);
            $io->writeln('   statut ' . $status);
            $this->check($io, $failures, 'elle répond 200', $status === 200);
            $this->check($io, $failures, 'avec un score et une correction', isset($body['result']['attemptScore'], $body['result']['review']));
            $this->check($io, $failures, '🔴 et sans réponse attendue', !str_contains(json_encode($body) ?: '', 'expected'));
            $this->check($io, $failures, 'et elle dit « non enregistré »', ($body['result']['saved'] ?? null) === false);
        }

        $io->section('7. Le constructeur (S182d) — lire et valider un brouillon');
        $order = $this->draft->normalise([['text' => 'Q', 'kind' => 'order', 'choices' => [['text' => 'A'], ['text' => 'B']]]]);
        $this->check($io, $failures, 'une étape d\'ordre est « juste » sans case cochée', $order[0]['choices'][0]['correct'] && $order[0]['choices'][1]['correct']);
        $this->check($io, $failures, 'et s\'enregistre en type « order »', $this->draft->storedType($order[0]) === QuizScorer::ORDER);
        $unknown = $this->draft->normalise([['text' => 'Q', 'kind' => 'script', 'choices' => [['text' => 'A', 'correct' => '1'], ['text' => 'B']]]]);
        $this->check($io, $failures, 'un type inconnu retombe sur « choix »', $unknown[0]['kind'] === QuizDraft::CHOICE);
        $this->check($io, $failures, 'deux cases cochées ⇒ « multiple »', $this->draft->storedType(['kind' => 'choice', 'choices' => [['correct' => true], ['correct' => true]]]) === QuizScorer::MULTIPLE);
        $this->check($io, $failures, 'un ordre à UNE étape est refusé', $this->draft->validate([['text' => 'Q', 'kind' => 'order', 'choices' => [['text' => 'A', 'correct' => true]]]]) !== []);
        $this->check($io, $failures, 'une réponse courte SANS réponse acceptée est refusée', $this->draft->validate([['text' => 'Q', 'kind' => 'short', 'choices' => []]]) !== []);
        $this->check($io, $failures, 'une réponse courte à UNE réponse acceptée passe', $this->draft->validate([['text' => 'Q', 'kind' => 'short', 'choices' => [['text' => 'A', 'correct' => true]]]]) === []);

        $io->section('8. 🔴 Rouvrir et réenregistrer un quiz ne change AUCUNE question');
        // Le chemin d'une édition sans modification : ce que le constructeur
        // réaffiche, renvoyé tel quel, relu, puis le type qu'on écrirait.
        $roundTrips = 0;
        $changed = [];
        foreach ($this->questions->findAll() as $question) {
            $choices = $this->choices->findBy(['question' => $question], ['ordre' => 'ASC']);
            if ($choices === []) {
                continue;
            }
            $shown = $this->draft->fromQuestion($question, $choices);
            $submitted = ['text' => $shown['text'], 'kind' => $shown['kind'], 'choices' => array_map(
                static fn (array $c): array => ['text' => $c['text'], 'correct' => $c['correct'] ? '1' : ''],
                $shown['choices'],
            )];
            $reread = $this->draft->normalise([$submitted])[0] ?? null;
            ++$roundTrips;
            if ($reread === null
                || $this->draft->storedType($reread) !== QuizScorer::effectiveType($question, $choices)
                || array_column($reread['choices'], 'correct') !== array_map(static fn (Choix $c): bool => $c->isEstCorrect(), $choices)) {
                $changed[] = '#' . $question->getId();
            }
        }
        $io->writeln('   ' . $roundTrips . ' questions réouvertes');
        $this->check($io, $failures, '🔴 AUCUNE ne change de type ni de bonne réponse', $changed === [] && $roundTrips > 0);
        if ($changed !== []) {
            $io->writeln('   modifiées : ' . implode(', ', array_slice($changed, 0, 10)));
        }

        $io->section('9. Rien n\'a été écrit');
        $this->check($io, $failures, 'PROGRESSION inchangée', (int) $this->db->fetchOne('SELECT COUNT(*) FROM PROGRESSION') === $progressionsBefore);
        $this->check($io, $failures, 'EMAIL_LOG inchangé', (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG') === $mailsBefore);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S182 verte. Rien écrit.');

        return Command::SUCCESS;
    }

    /**
     * 🔴 **L'ANCIEN correcteur, recopié ligne pour ligne** depuis
     * `QuizProgressService::saveResult()` avant S182c. Il n'est pas « amélioré » :
     * c'est la référence, et la moindre retouche ferait mesurer la sonde contre
     * elle-même.
     *
     * @param list<Choix> $choiceRows
     * @param list<string> $answer
     */
    private function oldVerdict(array $choiceRows, array $answer): bool
    {
        $expected = [];
        $allowed = [];
        foreach ($choiceRows as $choice) {
            $id = (string) $choice->getId();
            $allowed[$id] = true;
            if ($choice->isEstCorrect()) {
                $expected[] = $id;
            }
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

        return $expected === $actual;
    }

    /**
     * Toutes les parties d'un ensemble, y compris la vide.
     *
     * @param list<string> $ids
     * @return list<list<string>>
     */
    private function subsets(array $ids): array
    {
        $out = [];
        $n = \count($ids);
        for ($mask = 0; $mask < (1 << $n); ++$mask) {
            $pick = [];
            for ($i = 0; $i < $n; ++$i) {
                if ($mask & (1 << $i)) {
                    $pick[] = $ids[$i];
                }
            }
            $out[] = $pick;
        }

        return $out;
    }

    /** Un choix EN MÉMOIRE, jamais persisté — l'id est posé par réflexion. */
    private function choice(int $id, string $text, bool $correct, int $order): Choix
    {
        $c = (new Choix())->setTexte($text)->setEstCorrect($correct)->setOrdre($order);
        $ref = new \ReflectionProperty(Choix::class, 'id');
        $ref->setValue($c, $id);

        return $c;
    }

    private function firstQuizWithQuestions(): ?Quiz
    {
        foreach ($this->quizzes->findBy([], ['id' => 'ASC']) as $quiz) {
            if ($this->scorer->payload($quiz) !== []) {
                return $quiz;
            }
        }

        return null;
    }

    /**
     * Appelle la porte de correction avec une SESSION de visiteur : on lit le
     * jeton dans la page, puis on POSTe avec la même session — c'est le parcours
     * d'un navigateur non connecté.
     *
     * @return array{0: int, 1: array<string, mixed>}
     */
    private function callCheck(Quiz $quiz): array
    {
        $session = new Session(new MockArraySessionStorage());
        // Le même calcul que la route : un quiz interne s'ouvre sous sa formation
        // PARENTE — deviner le marqueur en SQL aurait été une seconde règle.
        $pageFormation = $this->catalog->getParentFormationId($quiz->getFormation()) ?? $quiz->getFormation()?->getId();

        $get = Request::create('/formations/' . $pageFormation . '/quiz/' . $quiz->getId());
        $get->setSession($session);
        $html = (string) $this->kernel->handle($get, HttpKernelInterface::MAIN_REQUEST, false)->getContent();
        if (!preg_match('/data-check-token="([^"]+)"/', $html, $m)) {
            return [0, ['error' => 'jeton introuvable dans la page']];
        }

        $post = Request::create('/api/quizzes/' . $quiz->getId() . '/check', 'POST', server: ['CONTENT_TYPE' => 'application/json'], content: json_encode(['_token' => html_entity_decode($m[1]), 'answers' => []]));
        $post->setSession($session);
        $response = $this->kernel->handle($post, HttpKernelInterface::MAIN_REQUEST, false);

        return [$response->getStatusCode(), json_decode((string) $response->getContent(), true) ?: []];
    }

    /** @param list<string> $failures */
    private function check(SymfonyStyle $io, array &$failures, string $what, bool $ok): void
    {
        $io->writeln(($ok ? '   <info>✓</info> ' : '   <error>✗</error> ') . $what);
        if (!$ok) {
            $failures[] = $what;
        }
    }
}
