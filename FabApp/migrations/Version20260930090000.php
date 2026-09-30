<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S204 — une pièce peut exiger un badge (comme une machine).
 *
 * PLACE_BADGE : les badges qui ouvrent une pièce. Une pièce SANS ligne se
 * comporte comme avant (réservation libre, porte ouverte sur une réservation
 * en cours). Avec des lignes : réserver exige l'un de ces badges, et la porte
 * s'ouvre au badge seul pendant les heures d'ouverture (décision de
 * l'opérateur, 2026-09-30).
 *
 * ⚠️ Table NEUVE, aucune colonne sur une entité existante : le code se déploie
 * avant elle et se tait tant qu'elle manque. Pas de clé étrangère (même règle
 * que S191/S202) ; un badge ou une pièce archivés restent des lignes.
 */
final class Version20260930090000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S204: PLACE_BADGE (badges that open a room: required to book it, and enough at its door during opening hours). New table only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS PLACE_BADGE (
                placeId INT NOT NULL,
                badgeId INT NOT NULL,
                createdAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                INDEX IDX_PLACE_BADGE_BADGE (badgeId),
                PRIMARY KEY(placeId, badgeId)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS PLACE_BADGE');
    }
}
