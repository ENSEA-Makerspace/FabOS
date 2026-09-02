<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S159f — CONTRACT : `UTILISATEUR_ROLE` disparaît, les groupes portent les rôles.
 *
 * 🔴 **C'est une étape de CONTRACT, et elle vient APRÈS son code.** Le code
 * déployé ne lit plus cette table pour décider d'un accès :
 * `Utilisateur::getRoles()` construit ses rôles depuis l'appartenance aux cinq
 * groupes intégrés, et plus rien ne l'écrit. Jouer cette migration avant ce code
 * aurait retiré tous les rôles d'un coup — c'est l'ordre que
 * `feedback_fabos_migration_hazard` interdit d'inverser.
 *
 * **Le chemin qui a mené ici**, parce qu'il est la raison de croire que c'est sûr :
 *
 *  1. **S158a** — l'écran des groupes : la table `USER_GROUP_MEMBER` cesse d'être
 *     une table que rien n'écrit.
 *  2. **S158c** — le backfill, purement additif : chaque appartenance qu'un rôle
 *     produisait reçoit sa ligne. La commande annulait tout si une seule réponse
 *     changeait ; aucune n'a changé.
 *  3. **S159b** — `getRoles()` rend l'UNION des deux sources. Une union ne peut
 *     qu'ajouter, et la passe d'ombre l'a mesuré : neutre.
 *  4. **S159e/f** — les écritures passent aux groupes, puis la lecture de
 *     l'ancienne table est retirée. La passe d'ombre est restée neutre.
 *
 * ⚠️ **Vérifié personne par personne avant d'écrire cette migration** :
 * `admin` {1,3,4,5,7,9}, `staff` {5,7}, `trainer` {5,7,9} — identiques des deux
 * côtés. Les deux lignes de rôle `user` n'avaient pas d'équivalent en groupe et
 * n'en ont pas besoin : `ROLE_USER` est accordé à tout compte sans ligne, ce qui
 * est la définition même de l'audience résolue `user`.
 *
 * ⚠️ **`ROLE` n'est PAS supprimée ici.** Elle devient une table de référence que
 * plus personne ne joint ; la supprimer demande de retirer aussi ses entités et
 * son dépôt, ce qui n'a pas sa place dans la même migration que la coupure. Elle
 * ne coûte rien et son retrait est un ménage, pas un contract.
 */
final class Version20260902100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S159f: drop UTILISATEUR_ROLE. Roles now come from USER_GROUP_MEMBER. Reversible: down() rebuilds it from the groups.';
    }

    public function up(Schema $schema): void
    {
        // ⚠️ `IF EXISTS` : une installation qui n'a jamais eu la table, ou sur
        // laquelle la migration a déjà tourné, ne doit pas échouer ici.
        $this->addSql('DROP TABLE IF EXISTS UTILISATEUR_ROLE');
    }

    /**
     * 🔴 **Et le retour en arrière RECONSTRUIT les lignes, il ne rend pas une
     * table vide.** C'est possible précisément parce que les deux sources
     * disaient la même chose : l'appartenance aux groupes intégrés contient tout
     * ce que `UTILISATEUR_ROLE` contenait. Un `down()` qui recrée une coquille
     * vide serait un retour en arrière qui verrouille l'installation — la forme
     * de rollback la plus dangereuse, celle qui a l'air d'avoir marché.
     *
     * ⚠️ Les lignes de rôle `user` ne sont pas reconstruites : elles étaient déjà
     * redondantes avant la fusion, `ROLE_USER` étant accordé sans ligne.
     */
    public function down(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS UTILISATEUR_ROLE (
                utilisateurId INT NOT NULL,
                roleId INT NOT NULL,
                INDEX IDX_UTILISATEUR_ROLE_ROLE (roleId),
                PRIMARY KEY(utilisateurId, roleId),
                CONSTRAINT FK_UTILISATEUR_ROLE_USER FOREIGN KEY (utilisateurId)
                    REFERENCES UTILISATEUR (id) ON DELETE CASCADE,
                CONSTRAINT FK_UTILISATEUR_ROLE_ROLE FOREIGN KEY (roleId)
                    REFERENCES ROLE (id) ON DELETE CASCADE
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);

        // ⚠️ `trainers` (le groupe) ↔ `trainer` (la ligne de rôle) : la seule
        // différence de nommage entre les deux modèles, et elle est explicite
        // ici comme elle l'est dans `Utilisateur::ROLE_FOR_GROUP`.
        $this->addSql(<<<'SQL'
            INSERT IGNORE INTO UTILISATEUR_ROLE (utilisateurId, roleId)
            SELECT m.userId, r.id
            FROM USER_GROUP_MEMBER m
            INNER JOIN USER_GROUP g ON g.id = m.groupId AND g.builtin = 1 AND g.virtual = 0
            INNER JOIN ROLE r ON r.nom = CASE g.groupKey
                WHEN 'trainers' THEN 'trainer'
                ELSE g.groupKey
            END
        SQL);
    }
}
