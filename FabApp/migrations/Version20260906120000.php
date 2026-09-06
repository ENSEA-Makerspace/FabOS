<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * S180b — une formation DIT si elle exige une validation pratique.
 *
 * 🔴 **Le défaut qu'elle corrige.** Jusqu'ici, savoir si une formation exige une
 * validation pratique se décidait par une liste de MOTS-CLÉS FRANÇAIS codée en
 * dur — `laser`, `soudure`, `fraiseuse`, `cnc`, `brodeuse` — cherchés dans le
 * titre et la catégorie (`TrainingPolicyService::PHYSICAL_FORMATION_KEYWORDS`).
 *
 * ⚠️ **C'est une garde de SÉCURITÉ décidée par une correspondance de chaîne.**
 * Un labo qui nomme son cours « Découpe au CO2 », « Plasma », « Tour à métaux »,
 * ou qui travaille en anglais, n'obtenait **aucune** exigence pratique —
 * silencieusement, sur un écran qui a l'air correct. Même famille que le repli
 * `['PLA','PETG','TPU','Support']` retiré en S174 : une liste en dur qui tient
 * lieu de donnée.
 *
 * ✅ **Rien ne change au moment où elle passe, et c'est le but.** La colonne est
 * remplie depuis les mots-clés eux-mêmes : chaque formation reçoit exactement la
 * valeur que le code calculait pour elle une seconde plus tôt. Ce qui change est
 * qu'à partir de maintenant, **quelqu'un peut la corriger**.
 *
 * ⚠️ **Nullable, et le repli reste — pour un temps.** `NULL` veut dire « je n'ai
 * pas d'avis, demande aux mots-clés ». Le remplissage ci-dessous ne laisse aucun
 * `NULL` sur les lignes existantes ; le repli ne sert donc qu'aux lignes créées
 * par un code plus ancien. 🅿️ **L'étape de CONTRACTION** — passer la colonne en
 * `NOT NULL` et supprimer la lecture des mots-clés — se fait une fois que le
 * code qui écrit toujours la colonne a tourné un moment. Expand, soak, contract.
 *
 * ⚠️ **Re-jouable.** Doctrine n'enregistre la version que si la migration entière
 * réussit ; un échec à mi-chemin laisserait sinon des lignes remplies et une
 * version non enregistrée. Les deux `UPDATE` sont idempotents par construction.
 *
 * ⚠️ **Elle ne touche PAS les formations internes** (`[FABOS SECTION]`,
 * `[FABOS QUIZ]`, `Validation physique — …`). Ce sont des sous-objets du moteur,
 * pas des cours : leur donner un avis sur la pratique en ferait des candidats à
 * la file des validations.
 */
final class Version20260906120000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'S180b: FORMATION.requiresPractical — an explicit safety flag, backfilled from the hardcoded keyword list so behaviour is unchanged.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE FORMATION ADD requiresPractical TINYINT(1) DEFAULT NULL');

        /*
         * 🔴 **Le remplissage REPRODUIT exactement la règle d'aujourd'hui**, mots
         * pour mots. Toute différence entre cette expression et
         * `PHYSICAL_FORMATION_KEYWORDS` changerait le comportement au moment de
         * la migration — c'est-à-dire la seule chose qu'elle promet de ne pas
         * faire.
         * ⚠️ `LOWER()` des deux côtés : la comparaison du code se fait sur une
         * chaîne normalisée en minuscules.
         */
        $this->addSql(<<<'SQL'
            UPDATE FORMATION
               SET requiresPractical = 1
             WHERE requiresPractical IS NULL
               AND titre NOT LIKE '[FABOS%'
               AND titre NOT LIKE 'Validation physique%'
               AND LOWER(CONCAT(titre, ' ', IFNULL(categorie, ''))) REGEXP 'laser|soudure|fraiseuse|cnc|brodeuse'
        SQL);

        $this->addSql(<<<'SQL'
            UPDATE FORMATION
               SET requiresPractical = 0
             WHERE requiresPractical IS NULL
               AND titre NOT LIKE '[FABOS%'
               AND titre NOT LIKE 'Validation physique%'
        SQL);
    }

    /**
     * ⚠️ **Le retour en arrière PERD les corrections.** Une formation qu'un
     * exploitant aurait cochée à la main — « Tour à métaux », que les mots-clés
     * ne voient pas — redeviendrait sans exigence pratique. C'est le sens
     * PERMISSIF du repli, et c'est le pire sens pour une garde de sécurité : à
     * dire avant de le lancer, pas après.
     */
    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE FORMATION DROP requiresPractical');
    }
}
