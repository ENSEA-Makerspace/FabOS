<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S210 — « Signaler un problème » : la boîte des retours des usagers.
 *
 * FEEDBACK : un bug, une gêne d'ergonomie ou une idée, avec la page, le compte,
 * le navigateur et la version de FabOS joints à l'envoi. `status` : open | done.
 *
 * ⚠️ Table NEUVE, aucune colonne sur une entité existante : le code se déploie
 * avant elle et se tait tant qu'elle manque. Pas de clé étrangère : un compte
 * supprimé laisse son retour lisible (userId reste la trace).
 * Horodatages en UTC.
 */
final class Version20261005094000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S210: FEEDBACK (bug / usability / idea reports sent from any page). New table only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS FEEDBACK (
                id INT AUTO_INCREMENT NOT NULL,
                userId INT DEFAULT NULL,
                kind VARCHAR(10) NOT NULL,
                message TEXT NOT NULL,
                pageUrl VARCHAR(500) NOT NULL,
                userAgent VARCHAR(255) DEFAULT NULL,
                appVersion VARCHAR(40) DEFAULT NULL,
                status VARCHAR(10) DEFAULT 'open' NOT NULL,
                createdAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                doneAt DATETIME DEFAULT NULL,
                INDEX IDX_FEEDBACK_STATUS (status, createdAt),
                INDEX IDX_FEEDBACK_USER (userId, createdAt),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS FEEDBACK');
    }
}
