<?php

namespace App\Reporting;

final readonly class ReportData
{
    /** @param array<string, int|float> $summary @param list<array<string, mixed>> $daily @param list<array<string, mixed>> $top */
    public function __construct(public array $summary, public array $daily, public array $top) {}
}
