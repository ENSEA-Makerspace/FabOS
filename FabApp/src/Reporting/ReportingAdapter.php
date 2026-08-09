<?php

namespace App\Reporting;

use Symfony\Component\DependencyInjection\Attribute\AutoconfigureTag;

#[AutoconfigureTag('app.reporting_adapter')]
interface ReportingAdapter
{
    public function supports(string $workspace): bool;
    public function report(ReportScope $scope): ReportData;

    /** @return iterable<array<string, scalar|null>> */
    public function export(ReportScope $scope): iterable;
}
