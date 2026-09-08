<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S165 — la médiathèque d'identité : poser un logo sans toucher au serveur.
 *
 * 🔴 **Le défaut qu'elle répare.** `site_logo_path` était une CHAÎNE — un nom de
 * fichier qui devait DÉJÀ se trouver dans `public/images/`. Rien ne téléversait :
 * mettre un logo demandait un accès SSH au serveur, ce qui n'est pas un thème,
 * c'est un déploiement. Un réglage qu'on ne peut pas régler depuis l'écran qui le
 * propose est une affordance morte avec une base de données derrière.
 *
 * ⚠️ **Une table NEUVE, donc le code se déploie AVANT elle sans risque.**
 * `SiteMediaLibrary` est en DBAL et sonde l'existence de la table une fois par
 * processus : sans elle, la médiathèque est vide et le site sert le logo livré.
 * C'est le bon côté du partage de [[feedback-fabos-migration-hazard]].
 *
 * 🔴 **`mediaId` et pas le nom du fichier téléversé.** Le nom d'origine est
 * conservé pour l'affichage, jamais pour le chemin : deux personnes qui envoient
 * `logo.png` ne doivent pas s'écraser, et un nom choisi par l'utilisateur ne doit
 * jamais atteindre le système de fichiers. Le fichier sur disque s'appelle
 * `<mediaId>.<ext>`, et rien d'autre.
 *
 * 🅿️ **Aucune reprise de données, et c'est mesuré** : le 2026-09-08, la table
 * `SITE_SETTING` ne contient AUCUNE ligne `site_logo_path`, et le brouillon de
 * thème porte `logoPath: ""`. Il n'existe donc pas une seule valeur héritée à
 * convertir — d'où l'absence de branche de compatibilité dans le code, qui aurait
 * été du code sans cas d'usage.
 */
final class Version20260908140000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S165: SITE_MEDIA — the identity media library (upload, server-side naming, stable id). New table only; no row created.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS SITE_MEDIA (
                id INT AUTO_INCREMENT NOT NULL,
                mediaId VARCHAR(32) NOT NULL,
                filename VARCHAR(80) NOT NULL,
                originalName VARCHAR(255) DEFAULT NULL,
                mimeType VARCHAR(60) NOT NULL,
                width INT DEFAULT NULL,
                height INT DEFAULT NULL,
                bytes INT DEFAULT NULL,
                uploadedAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                UNIQUE INDEX UNIQ_SITE_MEDIA (mediaId),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);
    }

    /**
     * ⚠️ **Le retour en arrière NE SUPPRIME PAS les fichiers.** Ils restent dans
     * `public/uploads/identity/`, orphelins mais intacts : effacer des images
     * qu'un opérateur a téléversées pour défaire un changement de schéma serait
     * une perte de données déguisée en migration.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE SITE_MEDIA');
    }
}
