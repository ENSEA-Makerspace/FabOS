<?php

namespace App\Reporting;

/** Atomic reporting permissions. Manage is the current authority for both. */
final class AnalyticsCapabilityRegistry
{
    public const VIEW = 'analytics.view';
    public const EXPORT = 'analytics.export';

    /** @return list<string> */
    public function all(): array { return [self::VIEW, self::EXPORT]; }
}
