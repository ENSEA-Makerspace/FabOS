<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S191 — la sécurité du profil : les sessions (S191a) et la double
 * authentification (S191b), dans UNE migration pour ne demander qu'un geste à
 * l'opérateur.
 *
 * USER_SESSION — une ligne par connexion.
 *   🔴 **Jamais l'identifiant de session PHP.** À la connexion, FabOS tire une
 *   clé au hasard, la range dans la session et n'en garde ici que l'empreinte
 *   SHA-256 : lire cette table ne permet de reprendre la session de personne.
 *   ⚠️ L'adresse IP est TRONQUÉE avant d'être écrite (/24 en IPv4, /48 en IPv6) :
 *   assez pour reconnaître « chez moi » ou « au labo », pas pour localiser.
 *   Une ligne fermée ou inactive depuis 30 jours est effacée.
 *
 * USER_MFA — le second facteur d'un compte, s'il en a un.
 *   🔴 Le secret TOTP est CHIFFRÉ (libsodium, clé dérivée d'APP_SECRET) ; les
 *   codes de secours sont HACHÉS et à usage unique. `lastUsedStep` refuse de
 *   rejouer un code déjà servi dans sa fenêtre de 30 s.
 *   `enabledAt` NULL = enrôlement commencé mais pas confirmé : le second facteur
 *   n'est exigé qu'une fois un premier code accepté.
 *
 * ⚠️ **Deux tables NEUVES, aucune colonne sur une entité existante** : le code
 * se déploie avant elles et se tait tant qu'elles manquent. Pas de clé
 * étrangère vers `UTILISATEUR` : un compte est ANONYMISÉ, jamais supprimé, et
 * c'est `AccountAnonymiser` qui efface ces lignes.
 */
final class Version20260924090000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S191: USER_SESSION (visible, revocable sessions — key hash only, truncated IP) and USER_MFA (encrypted TOTP secret, hashed recovery codes). New tables only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS USER_SESSION (
                id INT AUTO_INCREMENT NOT NULL,
                userId INT NOT NULL,
                keyHash CHAR(64) NOT NULL,
                createdAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                lastSeenAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                revokedAt DATETIME DEFAULT NULL,
                ipPrefix VARCHAR(64) DEFAULT NULL,
                userAgent VARCHAR(255) DEFAULT NULL,
                UNIQUE INDEX UNIQ_USER_SESSION_KEY (keyHash),
                INDEX IDX_USER_SESSION_USER (userId, revokedAt),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS USER_MFA (
                userId INT NOT NULL,
                secretEncrypted TEXT NOT NULL,
                enabledAt DATETIME DEFAULT NULL,
                lastUsedStep BIGINT DEFAULT NULL,
                recoveryCodes TEXT DEFAULT NULL,
                createdAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                PRIMARY KEY(userId)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
            SQL);
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE IF EXISTS USER_MFA');
        $this->addSql('DROP TABLE IF EXISTS USER_SESSION');
    }
}
