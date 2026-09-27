<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S197 — distinguer un compte CRÉÉ par un fournisseur d'un compte local LIÉ.
 *
 * `EXTERNAL_IDENTITY.provisioned` = 1 : le compte est né de cette identité ;
 * son mot de passe local est un aléa que personne ne connaît. « Mot de passe
 * oublié » répond alors « votre mot de passe est géré par X » au lieu de poser
 * un mot de passe local en douce. 0 : un compte local existant, lié ensuite
 * par sa propriétaire (preuve des deux côtés) — il garde son mot de passe.
 *
 * ⚠️ Expansion seulement : une colonne avec défaut, sur une table que seul DBAL
 * lit. Le code tourne avant elle (il ne distingue alors pas les deux cas et
 * garde l'ancien comportement du mot de passe oublié).
 */
final class Version20260927090000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S197: EXTERNAL_IDENTITY.provisioned (account created by the provider vs local account linked). Expand only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE EXTERNAL_IDENTITY ADD provisioned TINYINT(1) NOT NULL DEFAULT 0');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE EXTERNAL_IDENTITY DROP provisioned');
    }
}
