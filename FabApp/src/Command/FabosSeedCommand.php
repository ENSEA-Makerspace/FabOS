<?php

namespace App\Command;

use App\Entity\Machine;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Input\InputOption;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

#[AsCommand(name: 'app:fabos:seed', description: 'Recharge la base FABOS depuis le SQL MariaDB historique fourni.')]
final class FabosSeedCommand extends Command
{
    public function __construct(private readonly EntityManagerInterface $em)
    {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('reset', null, InputOption::VALUE_NONE, 'Supprime/recrée les tables FABOS puis réimporte les données de test du SQL.');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);

        if (!$input->getOption('reset') && $this->em->getRepository(Machine::class)->count([]) > 0) {
            $io->warning('Des machines existent déjà. Utilise --reset si tu veux remplacer la base par le SQL historique.');
            return Command::SUCCESS;
        }

        $connection = $this->em->getConnection();
        $sqlPath = dirname(__DIR__, 2) . '/resources/database/fabos_legacy_schema.sql';
        $sql = file_get_contents($sqlPath);
        if ($sql === false) {
            $io->error('Impossible de lire le fichier SQL: ' . $sqlPath);
            return Command::FAILURE;
        }

        $sql = preg_replace('/^\s*--.*$/m', '', $sql) ?? $sql;
        foreach (array_filter(array_map('trim', explode(';', $sql))) as $statement) {
            $connection->executeStatement($statement);
        }

        $io->success('Base FABOS recréée/importée depuis resources/database/fabos_legacy_schema.sql.');
        return Command::SUCCESS;
    }
}
