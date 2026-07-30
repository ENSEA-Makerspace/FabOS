<?php

namespace App\Portal;

use Doctrine\DBAL\Connection;

/**
 * Reads and writes the PORTAL table over raw DBAL, like the config stores it
 * serves (SiteFeatureService, SiteSettingService) — no entity, so nothing
 * hydrates it by accident.
 *
 * **Reads are fail-safe, writes are not.** A missing table resolves to "no
 * portals", which lands every caller on the global scope, i.e. today's
 * behaviour. A write that cannot happen must say so instead: silently not
 * creating a portal the admin just filled in a form for would be worse than an
 * error message.
 */
final class PortalRepository
{
    /** @var array<int, Portal|null> */
    private array $byId = [];

    public function __construct(private readonly Connection $db)
    {
    }

    public function findByHostname(string $hostname): ?Portal
    {
        return $this->fetchOnePortal(
            'SELECT id, slug, name, hostname, isDefault FROM PORTAL WHERE hostname = :h LIMIT 1',
            ['h' => $hostname],
        );
    }

    public function findBySlug(string $slug): ?Portal
    {
        return $this->fetchOnePortal(
            'SELECT id, slug, name, hostname, isDefault FROM PORTAL WHERE slug = :s LIMIT 1',
            ['s' => $slug],
        );
    }

    public function find(int $id): ?Portal
    {
        return $this->byId[$id] ??= $this->fetchOnePortal(
            'SELECT id, slug, name, hostname, isDefault FROM PORTAL WHERE id = :i',
            ['i' => $id],
        );
    }

    /**
     * The portal standing in for the global scope. Prefers the flagged default and
     * falls back to the oldest row, so a database whose flag was cleared by hand
     * still resolves to something rather than to null.
     */
    public function findDefault(): ?Portal
    {
        return $this->fetchOnePortal('SELECT id, slug, name, hostname, isDefault FROM PORTAL ORDER BY isDefault DESC, id ASC LIMIT 1');
    }

    /** @return Portal[] */
    public function all(): array
    {
        try {
            $rows = $this->db->fetchAllAssociative('SELECT id, slug, name, hostname, isDefault FROM PORTAL ORDER BY isDefault DESC, name ASC');
        } catch (\Throwable) {
            return [];
        }

        return array_map(Portal::fromRow(...), $rows);
    }

    /**
     * @throws \RuntimeException with a message meant for the admin to read
     *
     * @return int the new portal's id
     */
    public function create(string $name, string $slug, ?string $hostname): int
    {
        $slug = self::normaliseSlug($slug);
        $hostname = self::normaliseHostname($hostname);
        $this->assertFree($slug, $hostname, null);

        $this->db->executeStatement(
            'INSERT INTO PORTAL (slug, name, hostname, isDefault) VALUES (:s, :n, :h, 0)',
            ['s' => $slug, 'n' => $name, 'h' => $hostname],
        );
        $this->byId = [];

        return (int) $this->db->lastInsertId();
    }

    /**
     * Identity only. **`isDefault` is deliberately not editable here** — the
     * default portal is not "the important one", it *is* the global scope, and
     * every `portalId = 0` row in SITE_SETTING and SITE_MODULE means "the default
     * portal's value" because of it. Moving the flag would silently re-point all
     * of them at a different front door. If that is ever wanted it needs a
     * migration that moves the rows too, not a checkbox.
     *
     * @throws \RuntimeException with a message meant for the admin to read
     */
    public function update(int $id, string $name, string $slug, ?string $hostname): void
    {
        $slug = self::normaliseSlug($slug);
        $hostname = self::normaliseHostname($hostname);
        $this->assertFree($slug, $hostname, $id);

        $this->db->executeStatement(
            'UPDATE PORTAL SET slug = :s, name = :n, hostname = :h WHERE id = :i',
            ['s' => $slug, 'n' => $name, 'h' => $hostname, 'i' => $id],
        );
        $this->byId = [];
    }

    /**
     * Deletes a portal **and the config rows scoped to it**.
     *
     * There is no foreign key — the config stores are raw DBAL by design — so
     * there is no cascade, and this is the same rule reservations live under:
     * *no cascade means you clean up explicitly.* Leaving the rows behind would
     * be worse than untidy: `id` is AUTO_INCREMENT, so the next portal created
     * after enough deletions can be handed a retired tenant's feature switches
     * and settings, and nothing in the UI would explain why.
     *
     * @return int rows removed from the two config tables
     */
    public function delete(int $id): int
    {
        $portal = $this->find($id);
        if ($portal === null) {
            throw new \RuntimeException('Ce portail n\'existe pas.');
        }
        if ($portal->isDefault) {
            // Not squeamishness: portalId 0 *is* this row, so deleting it would
            // leave every global config row pointing at nothing.
            throw new \RuntimeException('Le portail par défaut ne peut pas être supprimé : il est la configuration globale du site.');
        }

        $removed = $this->db->executeStatement('DELETE FROM SITE_SETTING WHERE portalId = :i', ['i' => $id]);
        $removed += $this->db->executeStatement('DELETE FROM SITE_MODULE WHERE portalId = :i', ['i' => $id]);
        $this->db->executeStatement('DELETE FROM PORTAL WHERE id = :i', ['i' => $id]);
        $this->byId = [];

        return $removed;
    }

    /** @throws \RuntimeException when the slug or hostname is already spoken for */
    private function assertFree(string $slug, ?string $hostname, ?int $exceptId): void
    {
        if ($slug === '') {
            throw new \RuntimeException('L\'identifiant doit contenir au moins une lettre ou un chiffre.');
        }

        $clash = $this->fetchOnePortal(
            'SELECT id, slug, name, hostname, isDefault FROM PORTAL WHERE slug = :s AND id <> :e LIMIT 1',
            ['s' => $slug, 'e' => $exceptId ?? 0],
        );
        if ($clash !== null) {
            throw new \RuntimeException(sprintf('L\'identifiant « %s » est déjà pris par le portail « %s ».', $slug, $clash->name));
        }

        if ($hostname === null) {
            return;
        }

        $clash = $this->fetchOnePortal(
            'SELECT id, slug, name, hostname, isDefault FROM PORTAL WHERE hostname = :h AND id <> :e LIMIT 1',
            ['h' => $hostname, 'e' => $exceptId ?? 0],
        );
        if ($clash !== null) {
            throw new \RuntimeException(sprintf('Le nom de domaine « %s » est déjà utilisé par le portail « %s ».', $hostname, $clash->name));
        }
    }

    private static function normaliseSlug(string $slug): string
    {
        return trim(preg_replace('/[^a-z0-9-]+/', '-', mb_strtolower(trim($slug))) ?? '', '-');
    }

    /**
     * A bare hostname, or null.
     *
     * ⚠️ **Empty must become NULL, never `''`.** `hostname` carries a UNIQUE
     * index, and MySQL treats NULLs as all different but empty strings as equal —
     * so a second portal saved with a blank hostname would be rejected as a
     * duplicate of the first, which reads as a nonsense error about a field the
     * admin left alone.
     *
     * People paste URLs, so the scheme and everything after the host is dropped
     * rather than stored and never matched: resolution compares against
     * `Request::getHost()`, which is the bare host.
     */
    private static function normaliseHostname(?string $hostname): ?string
    {
        $hostname = mb_strtolower(trim((string) $hostname));
        $hostname = preg_replace('#^[a-z]+://#', '', $hostname) ?? $hostname;
        $hostname = explode('/', $hostname)[0];
        $hostname = explode(':', $hostname)[0];

        return $hostname === '' ? null : $hostname;
    }

    /** @param array<string, mixed> $params */
    private function fetchOnePortal(string $sql, array $params = []): ?Portal
    {
        try {
            $row = $this->db->fetchAssociative($sql, $params);
        } catch (\Throwable) {
            return null;
        }

        return is_array($row) ? Portal::fromRow($row) : null;
    }
}
