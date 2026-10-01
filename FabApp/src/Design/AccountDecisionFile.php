<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use Doctrine\DBAL\Connection;

/**
 * « Dossier » d'un compte en attente pour la proposition `validation-inscription`
 * (2026-10-01, d'après `users/07-validation-inscription.png`).
 *
 * ⚠️ LECTURE SEULE. Rien de neuf : les comptes en attente sont ceux que la tuile
 * de `/admin/utilisateurs` et `AdminAttention` comptent déjà (`statut` =
 * `pending` / `en attente`) ; l'origine d'inscription vient de la table
 * `EXTERNAL_IDENTITY` que `ExternalIdentityService` écrit.
 *
 * Si aucun compte n'est en attente (le cas normal : aucun code de l'application
 * ne pose ce statut aujourd'hui), la proposition montre le premier compte non
 * vérifié, sinon le dernier inscrit — `fallback` le dit.
 */
final class AccountDecisionFile
{
    public function __construct(
        private readonly UtilisateurRepository $users,
        private readonly Connection $db,
    ) {
    }

    /**
     * @return array{user: Utilisateur|null, index: int, total: int, fallback: string|null, origin: array{kind: string, provider: string|null}}
     */
    public function build(int $index = 0): array
    {
        $pending = $this->users->findBy(['statut' => ['pending', 'en attente']], ['createdAt' => 'ASC', 'id' => 'ASC']);
        $fallback = null;
        $queue = $pending;

        if ($queue === []) {
            $unverified = $this->users->findBy(['isVerified' => false], ['createdAt' => 'ASC', 'id' => 'ASC'], 1);
            if ($unverified !== []) {
                $queue = $unverified;
                $fallback = 'unverified';
            } else {
                $queue = $this->users->findBy([], ['createdAt' => 'DESC', 'id' => 'DESC'], 1);
                $fallback = 'latest';
            }
        }

        $total = \count($queue);
        $index = $total === 0 ? 0 : max(0, min($index, $total - 1));
        $user = $queue[$index] ?? null;

        return [
            'user' => $user,
            'index' => $index,
            'total' => $total,
            'fallback' => $fallback,
            'origin' => $user === null ? ['kind' => 'form', 'provider' => null] : $this->origin($user),
        ];
    }

    /** @return array{kind: string, provider: string|null} `external` si un fournisseur d'identité a CRÉÉ le compte, `linked` s'il y est seulement lié, sinon `form`. */
    private function origin(Utilisateur $user): array
    {
        try {
            $row = $this->db->fetchAssociative(
                'SELECT providerKey, provisioned FROM EXTERNAL_IDENTITY WHERE userId = ? AND revokedAt IS NULL ORDER BY provisioned DESC, id LIMIT 1',
                [$user->getId()],
            );
        } catch (\Throwable) {
            // Avant la migration S197 (colonne `provisioned`) ou table absente : on ne sait pas, donc « formulaire ».
            return ['kind' => 'form', 'provider' => null];
        }

        if ($row === false) {
            return ['kind' => 'form', 'provider' => null];
        }

        return [
            'kind' => !empty($row['provisioned']) ? 'external' : 'linked',
            'provider' => isset($row['providerKey']) ? (string) $row['providerKey'] : null,
        ];
    }
}
