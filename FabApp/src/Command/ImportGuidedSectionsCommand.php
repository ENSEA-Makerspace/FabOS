<?php

namespace App\Command;

use Doctrine\DBAL\Connection;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\DependencyInjection\Attribute\Autowire;

#[AsCommand(
    name: 'app:import-guided-sections',
    description: 'Importe les sections guidées, leurs mini-quiz et les quiz bonus sans modifier le schéma.',
)]
final class ImportGuidedSectionsCommand extends Command
{
    public function __construct(
        private readonly Connection $connection,
        #[Autowire('%kernel.project_dir%')]
        private readonly string $projectDir,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $path = $this->projectDir . '/resources/database/formation_sections_guided.sql';

        if (!is_file($path)) {
            $io->error('Fichier SQL introuvable : ' . $path);
            return Command::FAILURE;
        }

        $sql = file_get_contents($path);
        if (!is_string($sql) || trim($sql) === '') {
            $io->error('Le fichier SQL est vide ou illisible.');
            return Command::FAILURE;
        }

        $statements = array_values(array_filter(
            $this->splitStatements($sql),
            static fn (string $statement): bool => preg_match('/^(?:START\s+TRANSACTION|BEGIN|COMMIT|ROLLBACK)\b/i', trim($statement)) !== 1,
        ));
        if ($statements === []) {
            $io->error('Aucune requête SQL exploitable n’a été trouvée.');
            return Command::FAILURE;
        }

        $executed = 0;
        $this->connection->beginTransaction();
        try {
            foreach ($statements as $statement) {
                $this->connection->executeStatement($statement);
                ++$executed;
            }
            $this->connection->commit();
        } catch (\Throwable $exception) {
            if ($this->connection->isTransactionActive()) {
                $this->connection->rollBack();
            }

            $io->error([
                'Import interrompu après ' . $executed . ' requête(s).',
                $exception->getMessage(),
            ]);

            return Command::FAILURE;
        }

        $sectionQuizCount = (int) $this->connection->fetchOne(
            "SELECT COUNT(*) FROM FORMATION WHERE categorie = 'Quiz interne' AND prerequis LIKE '%FABOS_QUIZ_CONTEXT=section;%'",
        );
        $bonusQuizCount = (int) $this->connection->fetchOne(
            "SELECT COUNT(*) FROM FORMATION WHERE categorie = 'Quiz interne' AND prerequis LIKE '%FABOS_BONUS=1;%'",
        );
        $requiredPageQuizCount = (int) $this->connection->fetchOne(
            "SELECT COUNT(*) FROM FORMATION WHERE categorie = 'Quiz interne' AND prerequis LIKE '%FABOS_PARENT_FORMATION_ID=%' AND prerequis NOT LIKE '%FABOS_QUIZ_CONTEXT=section;%' AND prerequis NOT LIKE '%FABOS_BONUS=1;%'",
        );

        $io->success([
            'Parcours guidés importés ou mis à jour.',
            $executed . ' requête(s) exécutée(s).',
            $sectionQuizCount . ' mini-quiz de section et ' . $bonusQuizCount . ' quiz bonus détectés.',
            'Aucune table ni colonne n’a été créée ou modifiée.',
        ]);

        if ($sectionQuizCount < 28 || $bonusQuizCount < 7) {
            $io->warning('Certaines formations attendues n’ont pas été trouvées par leur titre. Vérifiez les titres des 7 formations visibles dans la base.');
        }
        if ($requiredPageQuizCount === 0) {
            $io->warning('Aucun quiz obligatoire de la page Quiz n’est détecté. Importez d’abord resources/database/quiz_persistence_personnalises.sql si ce script n’a jamais été exécuté.');
        }

        return Command::SUCCESS;
    }

    /** @return list<string> */
    private function splitStatements(string $sql): array
    {
        $statements = [];
        $buffer = '';
        $length = strlen($sql);
        $quote = null;
        $escaped = false;
        $lineComment = false;
        $blockComment = false;

        for ($index = 0; $index < $length; ++$index) {
            $char = $sql[$index];
            $next = $index + 1 < $length ? $sql[$index + 1] : '';

            if ($lineComment) {
                if ($char === "\n") {
                    $lineComment = false;
                    $buffer .= $char;
                }
                continue;
            }

            if ($blockComment) {
                if ($char === '*' && $next === '/') {
                    $blockComment = false;
                    ++$index;
                }
                continue;
            }

            if ($quote === null) {
                if ($char === '-' && $next === '-' && ($index + 2 >= $length || ctype_space($sql[$index + 2]))) {
                    $lineComment = true;
                    ++$index;
                    continue;
                }
                if ($char === '#') {
                    $lineComment = true;
                    continue;
                }
                if ($char === '/' && $next === '*') {
                    $blockComment = true;
                    ++$index;
                    continue;
                }
                if ($char === "'" || $char === '"' || $char === '`') {
                    $quote = $char;
                    $buffer .= $char;
                    continue;
                }
                if ($char === ';') {
                    $statement = trim($buffer);
                    if ($statement !== '') {
                        $statements[] = $statement;
                    }
                    $buffer = '';
                    continue;
                }

                $buffer .= $char;
                continue;
            }

            $buffer .= $char;

            if ($quote === '`') {
                if ($char === '`') {
                    if ($next === '`') {
                        $buffer .= $next;
                        ++$index;
                    } else {
                        $quote = null;
                    }
                }
                continue;
            }

            if ($escaped) {
                $escaped = false;
                continue;
            }

            if ($char === '\\') {
                $escaped = true;
                continue;
            }

            if ($char === $quote) {
                if ($next === $quote) {
                    $buffer .= $next;
                    ++$index;
                } else {
                    $quote = null;
                }
            }
        }

        $statement = trim($buffer);
        if ($statement !== '') {
            $statements[] = $statement;
        }

        return $statements;
    }
}
