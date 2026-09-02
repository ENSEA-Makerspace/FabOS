<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S159g — l'appartenance à un groupe peut être DATÉE.
 *
 * ✅ **Purement additive, et c'est ce qui la rend sûre.** Deux colonnes
 * nullables sur `USER_GROUP_MEMBER`. Une ligne existante les reçoit à `NULL`,
 * qui veut dire **sans limite** — donc les onze lignes écrites par le backfill de
 * S158c continuent d'accorder exactement ce qu'elles accordaient. Rien ne change
 * pour personne au moment où elle passe.
 *
 * 🔴 **`NULL` = sans limite, des DEUX côtés, et c'est le piège de cette étape.**
 * Un filtre qui traiterait `validFrom IS NULL` comme « pas encore commencée » ou
 * `validUntil IS NULL` comme « déjà finie » retirerait d'un coup les audiences
 * `staff`, `admin` et `trainers` de tout le monde — l'installation entière sans
 * aucun rôle. La clause est écrite une fois, dans `UserGroupSchema`, et jamais
 * recopiée.
 *
 * ⚠️ **Le code est déployé AVANT, et il tolère l'absence des colonnes.**
 * `UserGroupSchema` les sonde ; sans elles, une appartenance est sans limite,
 * c'est-à-dire le comportement d'aujourd'hui. Le repli va vers l'ancien, jamais
 * vers le neuf — même argument, mot pour mot, que `UsageGrantSchema`.
 *
 * 🅿️ **Ce que cette migration ne fait PAS, volontairement** : le JOURNAL des
 * appartenances. Il n'a de sens que le jour où une machine écrit — une commande
 * du module commerce, dont le remboursement doit pouvoir retirer exactement ce
 * qu'elle a donné. Tant que seul un humain écrit, une ligne datée suffit, et une
 * table de journal sans écrivain serait un demi-modèle de plus.
 *
 * ⚠️ Et un refus qui n'est pas dans le schéma mais dans le dépôt : **une
 * appartenance au groupe `admin` ne peut pas porter de date de fin.** La garde du
 * dernier administrateur juge une écriture ; elle ne peut rien contre l'horloge.
 */
final class Version20260902160000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S159g: dated group membership. Two nullable columns on USER_GROUP_MEMBER; NULL means no limit. Additive.';
    }

    public function up(Schema $schema): void
    {
        // ⚠️ Nullables et sans valeur par défaut : une ligne existante arrive à
        // `NULL`, donc sans limite, donc inchangée.
        $this->addSql('ALTER TABLE USER_GROUP_MEMBER ADD validFrom DATETIME DEFAULT NULL, ADD validUntil DATETIME DEFAULT NULL');

        // La lecture d'appartenance filtre désormais sur ces deux colonnes à
        // chaque requête d'accès ; l'index sert la question posée.
        $this->addSql('CREATE INDEX IDX_USER_GROUP_MEMBER_WINDOW ON USER_GROUP_MEMBER (validFrom, validUntil)');
    }

    /**
     * ⚠️ **Le retour en arrière PERD les dates, et il faut le dire.** Une
     * appartenance qui devait expirer redeviendra permanente. C'est le sens
     * permissif du repli — personne ne perd un accès — mais quelqu'un peut en
     * garder un qui aurait dû s'arrêter.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('DROP INDEX IDX_USER_GROUP_MEMBER_WINDOW ON USER_GROUP_MEMBER');
        $this->addSql('ALTER TABLE USER_GROUP_MEMBER DROP validFrom, DROP validUntil');
    }
}
