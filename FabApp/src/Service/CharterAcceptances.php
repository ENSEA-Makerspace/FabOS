<?php

declare(strict_types=1);

namespace App\Service;

use App\Feature\SiteFeatureService;
use Doctrine\DBAL\Connection;

/**
 * S209 — la charte de sécurité et qui l'a acceptée (`CHARTER_ACCEPTANCE`).
 *
 * Le TEXTE n'est pas nouveau : c'est le règlement du lab (`/reglement`, que
 * l'admin édite déjà). Sa VERSION est une empreinte de ce texte : le modifier
 * redemande l'accord, sans table de versions à tenir.
 *
 * ⚠️ « Disponible » veut dire : fonction allumée (`allowsSurface`), migration
 * passée ET un texte écrit. Un lab qui n'a rien écrit ne demande rien à personne.
 */
final class CharterAcceptances
{
    private ?bool $ready = null;

    public function __construct(
        private readonly Connection $db,
        private readonly SiteSettingService $settings,
        private readonly SiteFeatureService $features,
    ) {
    }

    public function isReady(): bool
    {
        if ($this->ready === null) {
            try {
                $this->db->fetchOne('SELECT 1 FROM CHARTER_ACCEPTANCE LIMIT 1');
                $this->ready = true;
            } catch (\Throwable) {
                $this->ready = false;
            }
        }

        return $this->ready;
    }

    public function isAvailable(): bool
    {
        return $this->features->allowsSurface('charter') && $this->isReady() && $this->html() !== '';
    }

    /** Le texte courant : celui du règlement du lab (HTML saisi par l'admin, de confiance). */
    public function html(): string
    {
        return trim($this->settings->getLabRulesHtml());
    }

    public function version(): string
    {
        return substr(hash('sha256', $this->html()), 0, 16);
    }

    /** Quand la personne a accepté la version COURANTE, ou null. */
    public function acceptedAt(int $userId): ?\DateTimeImmutable
    {
        if (!$this->isReady()) {
            return null;
        }
        $v = $this->db->fetchOne('SELECT acceptedAt FROM CHARTER_ACCEPTANCE WHERE userId = ? AND version = ?', [$userId, $this->version()]);

        return $v === false ? null : new \DateTimeImmutable((string) $v, new \DateTimeZone('UTC'));
    }

    /** Vrai tant qu'il reste quelque chose à accepter pour cette personne. */
    public function isPending(int $userId): bool
    {
        return $this->isAvailable() && $this->acceptedAt($userId) === null;
    }

    public function accept(int $userId): bool
    {
        if (!$this->isAvailable()) {
            return false;
        }

        return $this->db->executeStatement(
            'INSERT IGNORE INTO CHARTER_ACCEPTANCE (userId, version, acceptedAt) VALUES (?, ?, ?)',
            [$userId, $this->version(), (new \DateTimeImmutable('now', new \DateTimeZone('UTC')))->format('Y-m-d H:i:s')],
        ) > 0;
    }
}
