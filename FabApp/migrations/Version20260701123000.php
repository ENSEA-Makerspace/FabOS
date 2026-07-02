<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

final class Version20260701123000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Crée la base FABOS selon le SQL MariaDB historique fourni.';
    }

    public function up(Schema $schema): void
    {
        $this->executeSqlFile(__DIR__ . '/sql/fabos_legacy_schema.sql');
    }

    public function down(Schema $schema): void
    {
        $tables = [
            'CHOIX', 'QUESTION', 'QUIZ', 'SECTION', 'PROGRESSION', 'LOG_UTILISATION',
            'ACCESS_RFID_LOG', 'RESERVATION', 'MACHINE_UTILISATEUR', 'UTILISATEUR_BADGE',
            'UTILISATEUR_ROLE', 'FORMATION', 'MACHINE', 'UTILISATEUR', 'ROLE', 'BADGE',
        ];

        $this->addSql('SET FOREIGN_KEY_CHECKS = 0');
        foreach ($tables as $table) {
            $this->addSql('DROP TABLE IF EXISTS ' . $table);
        }
        $this->addSql('SET FOREIGN_KEY_CHECKS = 1');
    }

    private function executeSqlFile(string $path): void
    {
        $sql = file_get_contents($path);
        if ($sql === false) {
            throw new \RuntimeException('Impossible de lire le fichier SQL: ' . $path);
        }

        $sql = preg_replace('/^\s*--.*$/m', '', $sql) ?? $sql;
        foreach (array_filter(array_map('trim', explode(';', $sql))) as $statement) {
            $this->addSql($statement);
        }
    }
}
