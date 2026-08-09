<?php

namespace App\Reporting;

final readonly class ReportScope
{
    public function __construct(
        public string $workspace,
        public \DateTimeImmutable $from,
        public \DateTimeImmutable $until,
        public ?int $venueId = null,
    ) {}
}
