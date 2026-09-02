<?php

declare(strict_types=1);

namespace App\UsageRights;

use Doctrine\DBAL\Connection;

/**
 * Cette base porte-t-elle les DATES d'appartenance ? (S159g)
 *
 * 🔴 **Pourquoi une sonde plutôt qu'une hypothèse.** L'appartenance à un groupe
 * décide désormais des rôles de sécurité : nommer une colonne que l'opérateur n'a
 * pas encore migrée ferait échouer la requête, et une requête d'appartenance qui
 * échoue rend une liste vide — c'est-à-dire **tout le monde sans aucun rôle**,
 * l'installation entière verrouillée entre un déploiement et sa migration.
 *
 * ⚠️ **La sonde replie vers l'ANCIEN comportement, jamais vers le neuf.** Sans
 * les colonnes, une appartenance est sans limite de temps — exactement ce
 * qu'étaient toutes les appartenances avant. C'est ce qui rend le déploiement du
 * code sûr avant la migration, et c'est le même argument, mot pour mot, que
 * `UsageGrantSchema`.
 *
 * ⚠️ **Mémoïsée par requête, pas au-delà.** Une sonde qui aurait survécu à la
 * migration continuerait de répondre « non » aussi longtemps que le processus
 * vit — sur un worker, jusqu'au prochain redémarrage.
 */
final class UserGroupSchema
{
    private ?bool $dateColumns = null;

    public function __construct(private readonly Connection $db)
    {
    }

    public function hasDateColumns(): bool
    {
        if ($this->dateColumns !== null) {
            return $this->dateColumns;
        }

        try {
            $this->db->fetchOne('SELECT validFrom, validUntil FROM USER_GROUP_MEMBER LIMIT 1');

            return $this->dateColumns = true;
        } catch (\Throwable) {
            // ⚠️ Une table vide répond sans erreur : la question est « le schéma
            // répond-il », pas « y a-t-il des lignes ».
            return $this->dateColumns = false;
        }
    }

    /**
     * Le fragment SQL qui limite une appartenance à l'instant demandé, ou une
     * chaîne vide quand la base ne sait pas encore dater.
     *
     * 🔴 **`NULL` vaut « sans limite », des deux côtés, et ce n'est pas un
     * détail.** Les onze lignes écrites par le backfill de S158c n'ont pas de
     * dates ; un filtre qui les traiterait comme expirées retirerait d'un coup
     * les audiences `staff`, `admin` et `trainers` de tout le monde.
     *
     * ⚠️ Le paramètre s'appelle `:membershipMoment` pour ne se cogner avec aucun
     * `:moment` déjà posé par l'appelant.
     */
    public function activeClause(string $alias = 'm'): string
    {
        if (!$this->hasDateColumns()) {
            return '';
        }

        return " AND ({$alias}.validFrom IS NULL OR {$alias}.validFrom <= :membershipMoment)"
            . " AND ({$alias}.validUntil IS NULL OR {$alias}.validUntil > :membershipMoment)";
    }

    /**
     * Les paramètres qu'`activeClause()` réclame — vides quand elle l'est.
     *
     * @return array<string, string>
     */
    public function activeParams(\DateTimeImmutable $at): array
    {
        return $this->hasDateColumns() ? ['membershipMoment' => $at->format('Y-m-d H:i:s')] : [];
    }
}
