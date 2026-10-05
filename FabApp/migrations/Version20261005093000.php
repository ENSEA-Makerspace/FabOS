<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S208 + S209 — avertissements par usager, et acceptation de la charte.
 *
 * USER_WARNING / WARNING_REASON : un REGISTRE tenu par l'équipe (aucun effet
 * automatique sur les droits). Les motifs sont du contenu que le lab règle
 * lui-même ; la migration en sème trois, en français.
 *
 * CHARTER_ACCEPTANCE : « cette personne a accepté CETTE version de la charte ».
 * La version est une empreinte du texte courant : changer le texte rend les
 * anciennes lignes obsolètes sans rien effacer.
 *
 * ⚠️ Tables NEUVES, sans clé étrangère (même règle que S191/S202/S204) : le code
 * se déploie avant elles et se tait tant qu'elles manquent.
 */
final class Version20261005093000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S208/S209: WARNING_REASON, USER_WARNING (warnings register) and CHARTER_ACCEPTANCE (safety charter). New tables only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS WARNING_REASON (
                id INT AUTO_INCREMENT NOT NULL,
                label VARCHAR(120) NOT NULL,
                position INT DEFAULT 0 NOT NULL,
                active TINYINT(1) DEFAULT 1 NOT NULL,
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS USER_WARNING (
                id INT AUTO_INCREMENT NOT NULL,
                userId INT NOT NULL,
                reasonId INT DEFAULT NULL,
                note TEXT DEFAULT NULL,
                issuedBy INT DEFAULT NULL,
                createdAt DATETIME NOT NULL,
                liftedAt DATETIME DEFAULT NULL,
                INDEX IDX_USER_WARNING_USER (userId),
                INDEX IDX_USER_WARNING_LIFTED (liftedAt),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS CHARTER_ACCEPTANCE (
                userId INT NOT NULL,
                version VARCHAR(64) NOT NULL,
                acceptedAt DATETIME NOT NULL,
                PRIMARY KEY(userId, version)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
        // Les motifs de départ : à l'équipe de les renommer ou les retirer.
        $this->addSql(<<<'SQL'
            INSERT INTO WARNING_REASON (label, position, active)
            SELECT t.label, t.position, 1 FROM (
                SELECT 'Sécurité' AS label, 1 AS position
                UNION ALL SELECT 'Comportement', 2
                UNION ALL SELECT 'Matériel non rendu ou abîmé', 3
            ) t
            WHERE NOT EXISTS (SELECT 1 FROM WARNING_REASON)
            SQL);
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS CHARTER_ACCEPTANCE');
        $this->addSql('DROP TABLE IF EXISTS USER_WARNING');
        $this->addSql('DROP TABLE IF EXISTS WARNING_REASON');
    }
}
