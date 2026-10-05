<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S207 — Check-in à paliers : présence, visiteur sans compte, motif, note de projet.
 *
 * CHECKIN : une visite. `userId` nul = visiteur sans compte (palier 2, `visitorName`
 * et `visitorType`). `endedAt` nul = encore présent ; à la lecture, une visite ouverte
 * dont le lieu a fermé est considérée finie (aucune tâche planifiée).
 * CHECKIN_REASON : la courte liste de motifs, réglable par le lab (palier 3).
 *
 * ⚠️ Tables NEUVES, sans clé étrangère (même règle que S191/S202/S204) : le code se
 * déploie avant elles et se tait tant qu'elles manquent. Horodatages en UTC.
 */
final class Version20261005092000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S207: CHECKIN (visits) and CHECKIN_REASON (the lab\'s short list of visit reasons, with four defaults). New tables only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS CHECKIN (
                id INT AUTO_INCREMENT NOT NULL,
                userId INT DEFAULT NULL,
                visitorName VARCHAR(120) DEFAULT NULL,
                visitorType VARCHAR(30) DEFAULT NULL,
                venueId INT DEFAULT NULL,
                reason VARCHAR(120) DEFAULT NULL,
                projectNote VARCHAR(500) DEFAULT NULL,
                source VARCHAR(10) NOT NULL,
                startedAt DATETIME NOT NULL,
                endedAt DATETIME DEFAULT NULL,
                INDEX IDX_CHECKIN_USER (userId, endedAt),
                INDEX IDX_CHECKIN_STARTED (startedAt),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS CHECKIN_REASON (
                id INT AUTO_INCREMENT NOT NULL,
                label VARCHAR(120) NOT NULL,
                position INT DEFAULT 0 NOT NULL,
                active TINYINT(1) DEFAULT 1 NOT NULL,
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
        // Les motifs par défaut : une table neuve est vide, jamais deux fois remplie.
        $this->addSql("INSERT INTO CHECKIN_REASON (label, position, active) SELECT * FROM (SELECT 'Projet personnel' AS label, 1 AS position, 1 AS active UNION ALL SELECT 'Cours / TP', 2, 1 UNION ALL SELECT 'Atelier', 3, 1 UNION ALL SELECT 'Visite', 4, 1) AS d WHERE NOT EXISTS (SELECT 1 FROM CHECKIN_REASON)");
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS CHECKIN');
        $this->addSql('DROP TABLE IF EXISTS CHECKIN_REASON');
    }
}
