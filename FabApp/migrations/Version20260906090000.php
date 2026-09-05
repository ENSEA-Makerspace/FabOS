<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S175 — un boîtier peut enfin garder une PORTE, pas seulement une machine.
 *
 * 🔴 **Le fait de modèle qui commandait toute la Phase P.** `RFID_READER.machineId`
 * était `NOT NULL` : représenter une porte imposait d'inventer une machine
 * fictive. C'est le genre de contournement qui se paie deux ans plus tard —
 * chaque écran qui liste les machines aurait montré « Porte d'entrée », chaque
 * réservation aurait pu la viser, chaque statistique l'aurait comptée.
 *
 * ✅ C'est aussi la réponse au todo de l'opérateur du 2026-09-03 : des boîtiers
 * identiques à ceux des machines, branchés sur des gâches électriques.
 *
 * ⚠️ **Purement ADDITIVE, et c'est ce qui la rend sûre.** Une nouvelle table, une
 * colonne nullable, et une contrainte QUI SE DESSERRE (`NOT NULL` → `NULL`).
 * Aucune ligne existante n'est touchée : le seul lecteur de cette installation
 * garde `machineId = 1` et reçoit `accessPointId = NULL`. Rien ne change pour lui
 * au moment où elle passe — c'est la mesure de sortie de S175.
 *
 * 🔴 **« Exactement un des deux » n'est PAS dans le schéma, et c'est délibéré.**
 * Un `CHECK` sur `(machineId IS NULL) <> (accessPointId IS NULL)` n'existe pas en
 * MariaDB avant 10.2 et se comporte différemment selon la version ; une base qui
 * refuse silencieusement d'appliquer une contrainte est pire qu'une contrainte
 * absente, parce qu'on croit l'avoir. La règle vit dans `RfidReader::target()` et
 * une sonde la vérifie sur les données réelles. ⚠️ Le jour où le socle minimal
 * est relevé, le `CHECK` est la bonne place — c'est écrit ici pour qu'on y pense.
 *
 * ⚠️ **Collation déclarée explicitement**, comme toutes les migrations de ce
 * dépôt : les tables issues de l'import legacy portent le défaut plus récent de
 * cette MariaDB (`utf8mb4_uca1400_ai_ci`), et un `JOIN` entre les deux jette
 * `1267 Illegal mix of collations` — qui fait échouer la migration ENTIÈRE.
 */
final class Version20260906090000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S175: ACCESS_POINT (door/gate/locker/zone) + RFID_READER may point at one instead of a machine. Additive; machineId becomes nullable.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS ACCESS_POINT (
                id INT AUTO_INCREMENT NOT NULL,
                nom VARCHAR(150) NOT NULL,
                kind VARCHAR(30) NOT NULL DEFAULT 'door',
                description TEXT DEFAULT NULL,
                localisation VARCHAR(150) DEFAULT NULL,
                venueId INT NOT NULL,
                placeId INT DEFAULT NULL,
                createdAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                archivedAt DATETIME DEFAULT NULL,
                INDEX IDX_ACCESS_POINT_VENUE (venueId),
                INDEX IDX_ACCESS_POINT_PLACE (placeId),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);

        // ⚠️ `RESTRICT` sur le lieu, `SET NULL` sur l'espace : un point d'accès
        // appartient à un site (le supprimer avec ses portes serait une perte
        // silencieuse), mais l'espace qu'il ouvre est une information, pas une
        // dépendance — une porte survit à la disparition de la salle.
        $this->addSql('ALTER TABLE ACCESS_POINT ADD CONSTRAINT FK_ACCESS_POINT_VENUE FOREIGN KEY (venueId) REFERENCES VENUE (id) ON DELETE RESTRICT');
        $this->addSql('ALTER TABLE ACCESS_POINT ADD CONSTRAINT FK_ACCESS_POINT_PLACE FOREIGN KEY (placeId) REFERENCES PLACE (id) ON DELETE SET NULL');

        // 🔴 Le desserrage. `CHANGE` conserve le type et les données ; les lignes
        // existantes gardent leur `machineId`.
        $this->addSql('ALTER TABLE RFID_READER CHANGE machineId machineId INT DEFAULT NULL');
        $this->addSql('ALTER TABLE RFID_READER ADD accessPointId INT DEFAULT NULL');
        $this->addSql('CREATE INDEX IDX_RFID_READER_ACCESS_POINT ON RFID_READER (accessPointId)');
        $this->addSql('ALTER TABLE RFID_READER ADD CONSTRAINT FK_RFID_READER_ACCESS_POINT FOREIGN KEY (accessPointId) REFERENCES ACCESS_POINT (id) ON DELETE CASCADE');
    }

    /**
     * 🔴 **Le retour en arrière n'est PAS symétrique, et il faut le dire avant de
     * le lancer.** Remettre `machineId NOT NULL` échoue s'il existe un lecteur
     * de porte — sa colonne est `NULL` par construction. C'est voulu : la base
     * refuse de perdre des lignes plutôt que de les rendre absurdes. Il faut
     * d'abord décider quoi faire de ces lecteurs-là, à la main.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE RFID_READER DROP FOREIGN KEY FK_RFID_READER_ACCESS_POINT');
        $this->addSql('DROP INDEX IDX_RFID_READER_ACCESS_POINT ON RFID_READER');
        $this->addSql('ALTER TABLE RFID_READER DROP accessPointId');
        $this->addSql('ALTER TABLE RFID_READER CHANGE machineId machineId INT NOT NULL');
        $this->addSql('DROP TABLE ACCESS_POINT');
    }
}
