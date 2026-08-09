<?php

namespace App\UsageRights;

/** Presentation and enforcement read the same answer, including its reason. */
final readonly class UsageRightVerdict
{
    /** @param list<string> $packages */
    public function __construct(
        public string $capability,
        public bool $allowed,
        public bool $enforced,
        public string $reason,
        public array $packages = [],
    ) {
    }

    public function tone(): string
    {
        return $this->allowed ? 'go' : ($this->reason === 'signin_required' ? 'muted' : 'caution');
    }
}
