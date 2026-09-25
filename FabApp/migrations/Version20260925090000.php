<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S196 — le socle des modules de connexion.
 *
 * AUTH_PROVIDER gagne :
 *   - `kind` : le protocole du module (`oidc` aujourd'hui ; `ldap`, `ad`, `cas`,
 *     `saml` viendront avec S198–S200). Défaut `oidc` : les lignes existantes
 *     restent lisibles telles quelles.
 *   - `settingsJson` : les réglages propres au module — la CORRESPONDANCE des
 *     attributs (quel attribut est l'identifiant immuable, l'e-mail, le nom…) et
 *     la confiance accordée aux adresses du fournisseur. 🔴 Jamais un secret :
 *     les secrets restent des NOMS de variables d'environnement.
 *
 * ⚠️ **Expansion seulement** : deux colonnes neuves, avec défauts, sur une table
 * que seul DBAL lit (aucune entité ORM ne l'hydrate). Le code se déploie avant
 * ou après, sans casser : il lit `kind ?? 'oidc'`.
 */
final class Version20260925090000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S196: AUTH_PROVIDER.kind (default oidc) and settingsJson (attribute mapping, email trust). Expand only.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql("ALTER TABLE AUTH_PROVIDER ADD kind VARCHAR(20) NOT NULL DEFAULT 'oidc', ADD settingsJson LONGTEXT DEFAULT NULL");
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE AUTH_PROVIDER DROP kind, DROP settingsJson');
    }
}
