<?php

declare(strict_types=1);

namespace App\UsageRights;

/** The only package-grant actions. Manage has reporting, never use. */
enum UsageGrantAction: string
{
    case Use = 'use';
    case Manage = 'manage';

    public function includesReporting(): bool
    {
        return $this === self::Manage;
    }

    public function covers(self $requested): bool
    {
        return $this === $requested;
    }
}
