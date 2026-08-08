<?php

namespace App\Portal;

use Symfony\Component\HttpFoundation\RequestStack;

/**
 * Which portal the current request is being served through, and — the part every
 * config store actually needs — which portal scope its rows should be read and
 * written at.
 *
 * Resolution is by hostname (teachers.fabos.example → that portal). Nothing else
 * routes yet: with no PORTAL row matching the host, and with the seeded default
 * portal carrying no hostname, every request resolves to the global scope, so the
 * whole mechanism is invisible until someone points a hostname at a portal.
 *
 * The default portal is deliberately NOT an override scope: it *is* the global
 * scope, so its settings stay the plain portalId = 0 rows the site already has.
 */
final class PortalContext
{
    public const GLOBAL_SCOPE = 0;

    private bool $overrideResolved = false;
    private ?Portal $override = null;
    private bool $currentResolved = false;
    private ?Portal $current = null;

    public function __construct(
        private readonly RequestStack $requests,
        private readonly PortalRepository $portals,
    ) {
    }

    /**
     * The portal in effect, for branding and admin display — the hostname match,
     * or the default portal when nothing matches. Null only when no portal exists
     * at all (table missing, i.e. pre-migration).
     */
    public function current(): ?Portal
    {
        if ($this->currentResolved) {
            return $this->current;
        }

        $this->currentResolved = true;

        return $this->current = $this->resolveOverride() ?? $this->portals->findDefault();
    }

    /**
     * The portalId that config rows belong to for this request: an overriding
     * portal's id, or GLOBAL_SCOPE. Lookups read this scope and fall back to
     * global; writes land here.
     */
    public function scopeId(): int
    {
        return $this->resolveOverride()?->id ?? self::GLOBAL_SCOPE;
    }

    /** A hostname-matched, non-default portal — the only thing that overrides global config. */
    private function resolveOverride(): ?Portal
    {
        if ($this->overrideResolved) {
            return $this->override;
        }

        $this->overrideResolved = true;
        $host = $this->requests->getMainRequest()?->getHost();
        $matched = $host !== null && $host !== '' ? $this->portals->findByHostname($host) : null;

        return $this->override = $matched !== null && !$matched->isDefault ? $matched : null;
    }
}
