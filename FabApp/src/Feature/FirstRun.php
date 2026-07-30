<?php

namespace App\Feature;

use App\Service\SiteSettingService;
use Doctrine\DBAL\Connection;

/**
 * Whether this install has ever been set up, and the questions to ask if not.
 *
 * **"Unconfigured" is an explicit flag, never an inference.** S25 left this open,
 * and the tempting shortcut — decide from whether settings happen to be empty —
 * is wrong in both directions: an operator who deliberately runs without a public
 * URL would be nagged forever, and one who fills in a single field would be
 * declared finished. `setup_completed_at` is written once, by finishing or by
 * dismissing, and nothing else moves it.
 *
 * ⚠️ **A missing flag is not enough to call an install fresh**, and this is the
 * part that matters for the deployment that already exists. Every install that
 * predates this code has no flag, so the flag alone would show a first-run wizard
 * to a lab that has been running for a year. Freshness therefore also requires
 * the install to *look* empty. The check is deliberately about content an
 * operator creates, not about users — an SSO or import can create accounts on a
 * site nobody has configured yet.
 *
 * The consequence is worth stating plainly: **on the live site this reports
 * "already set up" and the wizard never appears by itself.**
 */
final class FirstRun
{
    public const COMPLETED_KEY = 'setup_completed_at';

    /** Tables whose contents mean somebody has been using this install. */
    private const CONTENT_TABLES = ['MACHINE', 'EVENEMENT', 'FORMATION', 'PLACE', 'LAB_PAGE'];

    public function __construct(
        private readonly SiteSettingService $settings,
        private readonly Connection $db,
    ) {
    }

    /** True only for an install nobody has configured *and* nobody has used. */
    public function isFresh(): bool
    {
        if (($this->settings->get(self::COMPLETED_KEY) ?? '') !== '') {
            return false;
        }

        return !$this->hasContent();
    }

    public function completedAt(): ?string
    {
        $value = $this->settings->get(self::COMPLETED_KEY);

        return $value === null || $value === '' ? null : $value;
    }

    /** Records that the questions have been answered — or deliberately skipped. */
    public function markCompleted(): void
    {
        $this->settings->set(self::COMPLETED_KEY, (new \DateTimeImmutable())->format('Y-m-d H:i:s'));
    }

    /**
     * Whether anything an operator would have had to create exists yet.
     *
     * Fail-safe in the direction that protects a running site: if the tables
     * cannot be read at all, assume there *is* content, so a database hiccup
     * cannot make an established install look brand new.
     */
    private function hasContent(): bool
    {
        foreach (self::CONTENT_TABLES as $table) {
            try {
                if ((int) $this->db->fetchOne(sprintf('SELECT COUNT(*) FROM %s', $table)) > 0) {
                    return true;
                }
            } catch (\Throwable) {
                return true;
            }
        }

        return false;
    }
}
