<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S163 — un événement se souvient d'avoir été annoncé.
 *
 * 🔴 **Sans cette trace, « annoncer » n'est pas idempotent — et un e-mail parti
 * ne se rattrape pas.** Deux clics, un rechargement de page, un doublon de
 * soumission : sans marque en base, chacun réécrit à tout le labo. C'est la
 * mesure de sortie de la session, mot pour mot : « deux clics sur Annoncer
 * n'envoient qu'une fois ».
 *
 * ⚠️ **`announcedAt` est CLAIMÉ par un `UPDATE … WHERE announcedAt IS NULL`**,
 * pas par « lire puis écrire ». Deux requêtes laisseraient entre elles une
 * fenêtre où deux clics simultanés passent tous les deux — exactement le cas
 * qu'on prétend fermer.
 *
 * 🔴 **DEUX COLONNES SUR UNE ENTITÉ CHARGÉE PARTOUT : cette migration passe
 * AVANT le code, jamais après.** `Event` est mappée et lue par les pages
 * publiques ; déployer le code d'abord ferait 500 sur tout ce qui touche un
 * événement. C'est le sens exact du partage de
 * [[feedback-fabos-migration-hazard]] — une table neuve se déploie avant sa
 * migration, une colonne mappée jamais.
 *
 * 🅿️ **`announcedCount` est un COMPTE FIGÉ, pas un calcul.** Il dit combien de
 * personnes ont été écrites CE JOUR-LÀ. Le recalculer plus tard donnerait le
 * nombre de membres d'aujourd'hui, qui n'est pas ce qui s'est passé.
 * ⚠️ `NULL` sur les événements existants, et c'est la vérité : ils n'ont pas été
 * annoncés, ils sont d'avant la fonctionnalité. `0` voudrait dire « annoncé à
 * personne », ce qui est un fait différent.
 */
final class Version20260908090000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S163: EVENEMENT.announcedAt + announcedCount — the per-event trace that makes "announce to members" idempotent. Nullable, no backfill.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE EVENEMENT ADD announcedAt DATETIME DEFAULT NULL COMMENT \'(DC2Type:datetime_immutable)\', ADD announcedCount INT DEFAULT NULL');
    }

    /**
     * ⚠️ **Le retour en arrière PERD la trace, donc l'idempotence.** Un événement
     * déjà annoncé redeviendrait annonçable, et un second courrier partirait à
     * tout le labo. À dire avant de le lancer, pas après.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE EVENEMENT DROP announcedAt, DROP announcedCount');
    }
}
