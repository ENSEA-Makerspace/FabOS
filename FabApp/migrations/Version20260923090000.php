<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S183b — le fil privé entre un apprenant et l'équipe de formation.
 *
 * 🔴 **UN fil par (formation, apprenant), et un fil ne contient qu'UN apprenant.**
 * C'est ce qui rend l'invariant de la phase — « aucun message privé ne bascule
 * implicitement vers la cohorte » — STRUCTUREL plutôt que surveillé. Il n'existe
 * pas de fil à plusieurs apprenants dans ce schéma, donc pas de requête qui
 * pourrait en élargir un par erreur. Le même raisonnement qu'à S183 pour les
 * annonces : `queueToUser()` ne prend qu'un destinataire, donc il n'y a pas de
 * liste à oublier de masquer.
 * ⚠️ `UNIQUE(formationId, learnerId)` n'est pas une optimisation : c'est la base
 * qui garantit qu'un double clic sur « Écrire » n'ouvre pas deux fils parallèles
 * où l'équipe répondrait dans l'un pendant que l'apprenant attend dans l'autre.
 *
 * 🔴 **Qui répond : le groupe `trainers`, pas « les administrateurs ».** Mesuré le
 * 2026-09-23 : toutes les formations portent « Équipe FabLab » comme formateur —
 * un libellé, pas une personne. Le seul modèle fidèle est une boîte d'ÉQUIPE, et
 * l'équipe qui existe vraiment est le groupe `trainers` (→ `ROLE_TRAINER`), que
 * l'opérateur gère. Un administrateur qui n'est pas formateur ne lit pas les
 * messages privés des apprenants : administrer n'est pas un droit de lecture.
 * 🅿️ Le jour où une formation nomme ses formateurs, ce schéma ne change pas — on
 * restreint QUI VOIT les fils, sans déplacer une ligne.
 *
 * ⚠️ **Trois tables NEUVES, aucune colonne sur une entité existante** : le code
 * se déploie avant elles. Le dépôt est en DBAL et sonde leur existence ; sans
 * elles, le lien « Écrire à l'équipe » n'apparaît pas.
 *
 * ⚠️ **Pas de clé étrangère vers `UTILISATEUR`**, et c'est délibéré : un compte
 * n'est jamais supprimé ici, il est ANONYMISÉ, donc un `ON DELETE CASCADE` ne se
 * déclencherait jamais. C'est `AccountAnonymiser` qui efface les fils d'un
 * apprenant — le seul chemin qui passe réellement.
 */
final class Version20260923090000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S183b: FORMATION_THREAD, FORMATION_THREAD_MESSAGE, FORMATION_THREAD_READ — one private thread per (formation, learner), answered by the trainers group. New tables only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS FORMATION_THREAD (
                id INT AUTO_INCREMENT NOT NULL,
                formationId INT NOT NULL,
                learnerId INT NOT NULL,
                createdAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                lastMessageAt DATETIME DEFAULT NULL,
                UNIQUE INDEX UNIQ_FORMATION_THREAD (formationId, learnerId),
                INDEX IDX_FORMATION_THREAD_LAST (lastMessageAt),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);

        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS FORMATION_THREAD_MESSAGE (
                id INT AUTO_INCREMENT NOT NULL,
                threadId INT NOT NULL,
                authorId INT NOT NULL,
                body TEXT NOT NULL,
                createdAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                INDEX IDX_THREAD_MESSAGE (threadId, createdAt),
                PRIMARY KEY(id),
                CONSTRAINT FK_THREAD_MESSAGE_THREAD FOREIGN KEY (threadId) REFERENCES FORMATION_THREAD (id) ON DELETE CASCADE
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);

        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS FORMATION_THREAD_READ (
                threadId INT NOT NULL,
                userId INT NOT NULL,
                lastReadAt DATETIME NOT NULL,
                PRIMARY KEY(threadId, userId),
                CONSTRAINT FK_THREAD_READ_THREAD FOREIGN KEY (threadId) REFERENCES FORMATION_THREAD (id) ON DELETE CASCADE
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);
    }

    /**
     * ⚠️ **Le retour en arrière PERD toutes les conversations.** Des tables
     * déposées ne se récupèrent pas. À dire avant de le lancer, pas après.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE FORMATION_THREAD_READ');
        $this->addSql('DROP TABLE FORMATION_THREAD_MESSAGE');
        $this->addSql('DROP TABLE FORMATION_THREAD');
    }
}
