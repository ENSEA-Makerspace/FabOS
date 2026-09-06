<?php

namespace App\Mail;

use Doctrine\DBAL\Connection;

/**
 * Le texte qu'un exploitant a réécrit pour un e-mail, dans une langue (S160).
 *
 * 🔴 **Ce service ne peut PAS empêcher un mail de partir, et c'est son contrat
 * le plus important.** Un mot de passe oublié doit partir quelle que soit la
 * bêtise saisie dans l'éditeur — ou quelle que soit la panne de base. Toute
 * lecture est donc enveloppée : la moindre exception rend « aucune surcharge »,
 * et l'appelant retombe sur le texte livré.
 *
 * ⚠️ **DBAL et pas une entité ORM, délibérément.** C'est ce qui rend le code
 * déployable AVANT sa migration : sans la table, `find()` rend `null` et rien ne
 * change. Une entité mappée ferait 500 sur toute requête la touchant. C'est le
 * partage documenté dans [[feedback-fabos-migration-hazard]], et le même motif
 * que `SiteSettingService` et `MachineFavorite::isStorageReady()`.
 *
 * ⚠️ **Le corps est du TEXTE, jamais du Twig.** La substitution est faite ici,
 * sur une liste FERMÉE de champs — celle du contexte du mail. Passer le texte de
 * l'exploitant au compilateur Twig, ce serait offrir l'exécution de code
 * arbitraire à qui édite un e-mail.
 */
final class MailOverrides
{
    /**
     * ⚠️ **Sondée une fois par processus.** Un worker de mail traite des
     * centaines de messages ; interroger `information_schema` à chaque envoi
     * coûterait plus cher que le mail lui-même.
     */
    private ?bool $storageReady = null;

    public function __construct(private readonly Connection $db)
    {
    }

    /**
     * @return array{subject: ?string, body: ?string}|null `null` = pas de
     *         surcharge, ou toute raison de ne pas en avoir une
     */
    public function find(string $templateKey, string $locale): ?array
    {
        if (!$this->isStorageReady()) {
            return null;
        }

        try {
            $row = $this->db->fetchAssociative(
                'SELECT subject, body FROM EMAIL_TEMPLATE_OVERRIDE WHERE templateKey = ? AND locale = ?',
                [$templateKey, $locale],
            );
        } catch (\Throwable) {
            return null;
        }

        if ($row === false) {
            return null;
        }

        $subject = trim((string) ($row['subject'] ?? ''));
        $body = trim((string) ($row['body'] ?? ''));

        // ⚠️ Une surcharge VIDE n'est pas une surcharge : elle vaut « je n'ai
        // rien à dire de plus que le texte livré ». Rendre une chaîne vide
        // enverrait un mail sans objet ni corps, ce qui est pire que pas de
        // fonctionnalité du tout.
        if ($subject === '' && $body === '') {
            return null;
        }

        return ['subject' => $subject !== '' ? $subject : null, 'body' => $body !== '' ? $body : null];
    }

    /**
     * Remplace `{{ champ }}` par la valeur du contexte, et **rien d'autre**.
     *
     * 🔴 **Une liste FERMÉE, prise du contexte lui-même.** Un champ que le
     * gabarit ne fournit pas reste écrit tel quel, visible — plutôt que rendu
     * vide, ce qui produirait une phrase amputée que personne ne remarquerait.
     * ⚠️ Aucune boucle, aucune condition, aucun filtre : personne n'a demandé de
     * `{% for %}` dans un e-mail, et chaque construction acceptée est une
     * surface d'évasion de plus.
     *
     * @param array<string, mixed> $context
     */
    public function fill(string $text, array $context): string
    {
        return (string) preg_replace_callback(
            '/\{\{\s*([a-zA-Z_][a-zA-Z0-9_]*)\s*\}\}/',
            static function (array $m) use ($context): string {
                $value = $context[$m[1]] ?? null;

                // ⚠️ Seuls les scalaires sont substitués. Un objet rendrait
                // « Array » ou lèverait selon les cas ; le laisser visible dit à
                // l'exploitant que ce champ n'est pas utilisable ici.
                if (is_scalar($value)) {
                    return (string) $value;
                }

                return $m[0];
            },
            $text,
        );
    }

    /** Les champs qu'un gabarit met à disposition, pour l'éditeur de S161. */
    public function fieldsOf(array $context): array
    {
        $fields = [];
        foreach ($context as $key => $value) {
            if (is_scalar($value)) {
                $fields[] = $key;
            }
        }
        sort($fields);

        return $fields;
    }

    private function isStorageReady(): bool
    {
        if ($this->storageReady !== null) {
            return $this->storageReady;
        }

        try {
            return $this->storageReady = $this->db->createSchemaManager()->tablesExist(['EMAIL_TEMPLATE_OVERRIDE']);
        } catch (\Throwable) {
            return $this->storageReady = false;
        }
    }
}
