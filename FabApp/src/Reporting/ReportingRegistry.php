<?php

namespace App\Reporting;

use Symfony\Component\DependencyInjection\Attribute\AutowireIterator;

final class ReportingRegistry
{
    /** @param iterable<ReportingAdapter> $adapters */
    public function __construct(#[AutowireIterator('app.reporting_adapter')] private readonly iterable $adapters) {}

    public function forWorkspace(string $workspace): ReportingAdapter
    {
        foreach ($this->adapters as $adapter) {
            if ($adapter->supports($workspace)) {
                return $adapter;
            }
        }

        throw new \InvalidArgumentException('Workspace reporting non pris en charge.');
    }
}
