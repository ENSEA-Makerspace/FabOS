<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S202 — attribuer et retirer un badge à la main : QUI, QUAND, POURQUOI.
 *
 * BADGE_GRANT — le JOURNAL des badges donnés et retirés à la main.
 *   Le badge DÉTENU reste la ligne de `UTILISATEUR_BADGE` (c'est elle que lit
 *   le lecteur) ; cette table dit d'où il vient et, s'il a été retiré, par qui
 *   et pourquoi. 🔴 Décision opérateur : un badge n'est jamais effacé sans
 *   trace — le retrait est une ligne qui RESTE ; un retrait n'est jamais
 *   « réactivé » : redonner le badge écrit une ligne neuve.
 *   `origin` : `manual` (donné ici), `formation` (retiré alors qu'il venait
 *   d'une formation — la ligne est écrite au retrait), `federated` (S203).
 *
 * ⚠️ **Table NEUVE, aucune colonne sur une entité existante** (la roadmap
 * prévoyait des colonnes sur `UTILISATEUR_BADGE` ; une table à part évite
 * qu'une entité hydratée partout casse avant la migration). Le code se
 * déploie avant elle et se tait tant qu'elle manque. Pas de clé étrangère :
 * même règle que S191, un compte est anonymisé, jamais supprimé.
 */
final class Version20260924150000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S202: BADGE_GRANT (who granted or revoked a badge by hand, when and why). New table only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS BADGE_GRANT (
                id INT AUTO_INCREMENT NOT NULL,
                userId INT NOT NULL,
                badgeId INT NOT NULL,
                origin VARCHAR(20) NOT NULL,
                grantedById INT DEFAULT NULL,
                grantedAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                reason TEXT DEFAULT NULL,
                revokedById INT DEFAULT NULL,
                revokedAt DATETIME DEFAULT NULL,
                revokeReason TEXT DEFAULT NULL,
                INDEX IDX_BADGE_GRANT_USER (userId, badgeId, id),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS BADGE_GRANT');
    }
}
