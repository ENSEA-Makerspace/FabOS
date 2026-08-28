<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S152 : les documents attachés à une machine — guide d'usage, fiche de
 * sécurité… Demandé par l'opérateur le 2026-08-27, à faire avant la Phase H.
 *
 * 🔴 **Cette migration CRÉE une table et ne touche à rien d'existant.** Aucun
 * code déployé ne la nomme au moment où elle passe : elle est donc sans risque à
 * lancer, et sans effet tant que le code qui la lit n'est pas là. C'est l'ordre
 * imposé par [[feedback-fabos-migration-hazard]] — la migration d'abord, le code
 * ensuite — et l'inverse a déjà coûté deux pages en 500 (`LOANABLE_ITEM.archivedAt`).
 *
 * ⚠️ **`ON DELETE CASCADE` sur la machine, et c'est voulu.** Un document sans
 * machine n'est plus un document, c'est un fichier orphelin dont plus rien ne dit
 * à quoi il servait. ⚠️ Le FICHIER, lui, n'est pas effacé : supprimer la ligne
 * laisse l'octet dans `public/uploads/machine-documents/`, exactement comme pour
 * les avatars — c'est écrit dans `services.yaml` depuis longtemps et ça reste
 * vrai ici. Une purge est un travail à part, qui doit pouvoir se relire avant de
 * détruire.
 *
 * ⚠️ `CREATE TABLE IF NOT EXISTS` : la commande doit pouvoir être relancée sans
 * casser, parce qu'elle le sera — c'est l'opérateur qui la passe à la main.
 */
final class Version20260828100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S152: MACHINE_DOCUMENT — files attached to a machine (usage guide, safety sheet).';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(
            "CREATE TABLE IF NOT EXISTS MACHINE_DOCUMENT (
                id INT AUTO_INCREMENT NOT NULL,
                machineId INT NOT NULL,
                label VARCHAR(180) NOT NULL COMMENT 'Ce que le membre lit : « Guide d''usage »…',
                storedName VARCHAR(255) NOT NULL COMMENT 'Le nom sur le disque, horodate et slugifie.',
                originalName VARCHAR(255) NOT NULL COMMENT 'Le nom televerse, propose au telechargement.',
                mimeType VARCHAR(120) NOT NULL COMMENT 'Constate par le serveur, jamais annonce par le client.',
                sizeBytes INT NOT NULL,
                position INT NOT NULL DEFAULT 0,
                uploadedAt DATETIME NOT NULL COMMENT '(DC2Type:datetime_immutable)',
                INDEX IDX_MACHINE_DOCUMENT_MACHINE (machineId),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB",
        );

        $this->addSql(
            'ALTER TABLE MACHINE_DOCUMENT
             ADD CONSTRAINT FK_MACHINE_DOCUMENT_MACHINE
             FOREIGN KEY (machineId) REFERENCES MACHINE (id) ON DELETE CASCADE',
        );
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS MACHINE_DOCUMENT');
    }
}
