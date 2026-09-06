<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S160 — un exploitant peut réécrire le texte d'un e-mail, par langue.
 *
 * **Demandé par l'opérateur le 2026-09-04**, sur le dépouillement Fabmanager :
 * « Customize email templates », 10 votes.
 *
 * ✅ **Purement additive : une table neuve, et rien d'autre.** Aucune colonne
 * n'est ajoutée à une entité existante, donc le code qui la lit peut être
 * déployé AVANT elle — son dépôt est en DBAL et retombe sur « aucune surcharge »
 * si la table n'existe pas. C'est le partage documenté dans
 * [[feedback-fabos-migration-hazard]] : table neuve = sûr, colonne sur une
 * entité chargée partout = fatal.
 *
 * 🔴 **`locale` fait partie de la clé, et ce n'est pas négociable.** La règle de
 * la maison est « on traduit l'UI, jamais le contenu » — un texte réécrit par
 * l'exploitant EST du contenu. Une surcharge sans langue casserait les mails
 * anglais d'un labo bilingue qui n'aurait réécrit que le français. Les textes
 * livrés restent le repli, langue par langue.
 *
 * ⚠️ **`subject` et `body` sont du TEXTE, jamais du Twig.** La substitution se
 * limite aux champs connus du gabarit (`{{ event }}`), validée à
 * l'enregistrement. Accepter du Twig, ce serait offrir l'exécution de code
 * arbitraire à qui édite un e-mail.
 *
 * 🅿️ Aucune ligne n'est créée : sans surcharge, les mails rendent exactement ce
 * qu'ils rendaient. C'est la mesure de sortie de S160, et elle se vérifie par
 * `app:s160:mail-render-probe` avant et après.
 */
final class Version20260906180000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S160: EMAIL_TEMPLATE_OVERRIDE — operator-written mail text, per template AND per locale. New table only; no row created.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql(<<<'SQL'
            CREATE TABLE IF NOT EXISTS EMAIL_TEMPLATE_OVERRIDE (
                id INT AUTO_INCREMENT NOT NULL,
                templateKey VARCHAR(80) NOT NULL,
                locale VARCHAR(8) NOT NULL,
                subject VARCHAR(255) DEFAULT NULL,
                body TEXT DEFAULT NULL,
                updatedAt DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL,
                UNIQUE INDEX UNIQ_EMAIL_OVERRIDE (templateKey, locale),
                PRIMARY KEY(id)
            ) DEFAULT CHARACTER SET utf8mb4 COLLATE `utf8mb4_unicode_ci` ENGINE = InnoDB
        SQL);
    }

    /**
     * ⚠️ **Le retour en arrière PERD les textes écrits par l'exploitant.** Une
     * table déposée ne se récupère pas ; les mails reviennent simplement aux
     * textes livrés. À dire avant de le lancer, pas après.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('DROP TABLE EMAIL_TEMPLATE_OVERRIDE');
    }
}
