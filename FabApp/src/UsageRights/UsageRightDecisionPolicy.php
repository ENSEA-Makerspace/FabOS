<?php

namespace App\UsageRights;

/** Pure precedence rules, separately testable from storage and HTTP context. */
final class UsageRightDecisionPolicy
{
    /** @param list<string> $packages */
    public function decide(
        string $capability,
        bool $supported,
        bool $enforced,
        bool $featureEnabled,
        bool $subjectPresent,
        bool $subjectIsAdmin,
        array $packages,
    ): UsageRightVerdict {
        if (!$supported) {
            return new UsageRightVerdict($capability, false, $enforced, 'unsupported');
        }
        if (!$enforced) {
            return new UsageRightVerdict($capability, true, false, 'not_enforced');
        }
        if (!$featureEnabled) {
            return new UsageRightVerdict($capability, false, true, 'feature_disabled');
        }
        if (!$subjectPresent) {
            return new UsageRightVerdict($capability, false, true, 'signin_required');
        }
        if ($subjectIsAdmin) {
            return new UsageRightVerdict($capability, true, true, 'admin_bypass');
        }

        return new UsageRightVerdict($capability, $packages !== [], true, $packages !== [] ? 'granted' : 'missing_package', $packages);
    }
}
