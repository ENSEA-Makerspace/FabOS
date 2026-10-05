<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S206 — stocks de consommables, sous-fonction activable de Matériaux.
 *
 * MATERIAL_STOCK : une ligne par matériau suivi (quantité, unité, seuil bas).
 * MATERIAL_STOCK_MOVE : le journal des entrées / sorties (delta signé, note, qui).
 *
 * ⚠️ Tables NEUVES, ni la table ni l'entité `Material` ne bougent : le code se
 * déploie avant elles et se tait tant qu'elles manquent. Pas de clé étrangère
 * (même règle que S191/S202/S204) ; un matériau archivé garde ses lignes.
 */
final class Version20261005091000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S206: MATERIAL_STOCK (quantity, unit, low threshold per material) and MATERIAL_STOCK_MOVE (in/out journal). New tables only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS MATERIAL_STOCK (
                materialId INT NOT NULL,
                quantity DECIMAL(12, 3) DEFAULT 0 NOT NULL,
                unit VARCHAR(16) NOT NULL,
                lowThreshold DECIMAL(12, 3) DEFAULT NULL,
                updatedAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                PRIMARY KEY(materialId)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS MATERIAL_STOCK_MOVE (
                id INT AUTO_INCREMENT NOT NULL,
                materialId INT NOT NULL,
                delta DECIMAL(12, 3) NOT NULL,
                note VARCHAR(255) DEFAULT NULL,
                userId INT DEFAULT NULL,
                createdAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                INDEX IDX_MATERIAL_STOCK_MOVE_MATERIAL (materialId, createdAt),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS MATERIAL_STOCK_MOVE');
        $this->addSql('DROP TABLE IF EXISTS MATERIAL_STOCK');
    }
}
