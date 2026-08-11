<?php

namespace App\Service;

use Symfony\Component\Translation\TranslatorBagInterface;
use Symfony\Contracts\Translation\LocaleAwareInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * Puts the installation's own words into every translated string.
 *
 * The catalogs used to hardcode "FabLab" and "ENSEA" in roughly 134 places
 * across five languages. Those are now `%venue%` and `%org%`, which means every
 * one of those strings needs two parameters at render time — and passing them at
 * each `|trans` call would have meant editing every call site and, worse, would
 * have kept working silently when someone forgot, printing a raw `%venue%` on a
 * public page.
 *
 * Decorating the translator instead makes it structural: the parameters are
 * always there, for templates, controllers, mail and the console alike. A string
 * that does not use them is unaffected — Symfony ignores parameters it has no
 * placeholder for.
 *
 * ⚠️ **Resolution is lazy and fail-safe, and it has to be.** This service sits in
 * front of the translator, so it is reachable extremely early and from contexts
 * with no database — cache warmup, the console, the mail worker. Reading settings
 * eagerly in the constructor would make a config store a hard dependency of
 * rendering any text at all. On any failure it falls back to the defaults, which
 * are this install's existing wording, so the worst case is the site reading
 * exactly as it did before S31.
 */
final class VocabularyTranslator implements TranslatorInterface, TranslatorBagInterface, LocaleAwareInterface
{
    /** @var array<string, string>|null */
    private ?array $vocabulary = null;

    public function __construct(
        private readonly TranslatorInterface&TranslatorBagInterface&LocaleAwareInterface $inner,
        private readonly SiteSettingService $settings,
    ) {
    }

    public function trans(string $id, array $parameters = [], ?string $domain = null, ?string $locale = null): string
    {
        // Caller-supplied parameters win: a string that wants to name a *different*
        // organisation still can.
        return $this->inner->trans($id, [...$this->vocabulary(), ...$parameters], $domain, $locale);
    }

    /** @return array<string, string> */
    private function vocabulary(): array
    {
        if ($this->vocabulary !== null) {
            return $this->vocabulary;
        }

        try {
            return $this->vocabulary = [
                '%org%' => $this->settings->getOrgName(),
                '%venue%' => $this->settings->getVenueLabel(),
            ];
        } catch (\Throwable) {
            // No database, or none yet. ⚠️ This used to fall back to 'ENSEA' — the
            // organisation this instance was first written for. FabOS is an
            // open-source project that other labs install, so shipping one lab's name
            // as the built-in default put someone else's identity on a fresh install
            // before the operator had entered anything. The generic words are the
            // honest fallback; the operator's real ones arrive from settings.
            return $this->vocabulary = ['%org%' => 'FabOS', '%venue%' => 'FabLab'];
        }
    }

    public function getCatalogue(?string $locale = null): \Symfony\Component\Translation\MessageCatalogueInterface
    {
        return $this->inner->getCatalogue($locale);
    }

    public function getCatalogues(): array
    {
        return $this->inner->getCatalogues();
    }

    public function setLocale(string $locale): void
    {
        $this->inner->setLocale($locale);
    }

    public function getLocale(): string
    {
        return $this->inner->getLocale();
    }
}
