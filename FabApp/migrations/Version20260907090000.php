<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S162 — le journal des mails dit QUELLE VERSION a servi.
 *
 * 🔴 **C'est un critère de sortie de la Phase K**, pas un confort : sans cette
 * colonne, « pourquoi ce mail dit ça ? » est insoluble. Un exploitant qui a
 * réécrit un texte et reçoit le texte livré n'a aucun moyen de savoir si sa
 * surcharge ne s'applique pas, ou si elle s'est cassée et que le repli l'a
 * absorbée — et le repli, lui, est SILENCIEUX par construction.
 *
 * ⚠️ **Une colonne ajoutée à une table existante, mais SANS entité mappée.**
 * `EMAIL_LOG` est en DBAL pur (`MailLog`) : aucune entité Doctrine ne la
 * référence, donc aucune requête ne réclame la colonne au démarrage. Le danger
 * documenté dans [[feedback-fabos-migration-hazard]] — une colonne mappée sur
 * une entité chargée partout — ne s'applique pas ici, et l'écriture sonde la
 * présence de la colonne une fois par processus avant de la remplir.
 *
 * ⚠️ **`NULL` sur toutes les lignes existantes, et c'est la vérité.** On ne sait
 * pas rétroactivement quelle version a servi à un mail parti avant cette
 * colonne. Remplir « livré » par défaut serait une affirmation inventée : les
 * écrans disent « — » pour ces lignes-là.
 *
 * 🅿️ Additive et réversible sans perte de données utiles : le retour arrière ne
 * perd qu'une information de diagnostic, jamais un mail ni un texte.
 */
final class Version20260907090000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S162: EMAIL_LOG.renderedFrom — which version served (delivered / override / override_failed, plus the chrome parts). Nullable, no backfill.';
    }

    public function up(Schema $schema): void
    {
        // 64 caractères : la trace la plus longue possible est
        // « override_failed+header_failed+footer_failed », soit 43.
        $this->addSql('ALTER TABLE EMAIL_LOG ADD renderedFrom VARCHAR(64) DEFAULT NULL');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE EMAIL_LOG DROP renderedFrom');
    }
}
