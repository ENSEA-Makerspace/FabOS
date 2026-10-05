<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S205 — signaler une panne, sans compte, depuis un QR code collé sur la machine.
 *
 * MACHINE_REPORT : un signalement par ligne. Il NE change PAS l'état de la
 * machine (c'est l'équipe qui décide) ; il attend d'être résolu.
 *   - `status` : open | resolved ;
 *   - `ipHash` : empreinte salée de l'adresse de l'émetteur, SEULEMENT pour la
 *     limite de débit de la page publique (jamais l'adresse elle-même) ;
 *   - `resolvedBy` : l'identifiant du compte qui a résolu (pas de clé étrangère,
 *     même règle que S191/S202/S204).
 *
 * ⚠️ Table NEUVE, aucune colonne sur une entité existante : le code se déploie
 * avant elle et se tait tant qu'elle manque. Horodatages en UTC.
 */
final class Version20261005090000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S205: MACHINE_REPORT (breakdown reports from a public QR page, resolved by the team). New table only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS MACHINE_REPORT (
                id INT AUTO_INCREMENT NOT NULL,
                machineId INT NOT NULL,
                description LONGTEXT NOT NULL,
                photo VARCHAR(255) DEFAULT NULL,
                contact VARCHAR(190) DEFAULT NULL,
                status VARCHAR(16) DEFAULT 'open' NOT NULL,
                resolutionNote LONGTEXT DEFAULT NULL,
                ipHash VARCHAR(64) DEFAULT NULL,
                createdAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                resolvedAt DATETIME DEFAULT NULL,
                resolvedBy INT DEFAULT NULL,
                INDEX IDX_MACHINE_REPORT_MACHINE (machineId, status),
                INDEX IDX_MACHINE_REPORT_STATUS (status, createdAt),
                INDEX IDX_MACHINE_REPORT_IP (ipHash, createdAt),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS MACHINE_REPORT');
    }
}
