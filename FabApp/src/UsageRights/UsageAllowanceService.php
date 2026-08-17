<?php

declare(strict_types=1);

namespace App\UsageRights;

use App\Entity\Reservation;
use App\Entity\Utilisateur;
use App\Reservation\ReservableType;
use App\Service\SiteSettingService;
use Doctrine\DBAL\Connection;

/**
 * "You have used 8 of your 10 hours this week." (S144c)
 *
 * The third question in the booking path, and deliberately its own service:
 * certification asks *may you touch this*, quotas ask *how much may anyone of
 * your tier book*, and an allowance asks *how much did you buy*. Folding it into
 * `BookingPolicyService` would have put a per-package budget inside a class whose
 * whole shape is one all-nullable policy per (resource kind × tier) — and a lab
 * loosening a tier limit would then have been loosening what it sold.
 *
 * ⚠️ **Silent when nothing is configured.** No allowance rows, no enforcement, no
 * package: the check returns null before it counts anything. An install that
 * never opens this screen pays one indexed query per booking at most, and behaves
 * exactly as it did.
 *
 * ⚠️ **It never grants.** An allowance can only refuse a booking the rest of the
 * path already allowed; it is a ceiling on an entitlement, not an entitlement.
 * A member with no package therefore has no ceiling — their access was decided by
 * grants, and if they got this far nothing here narrows it.
 */
final class UsageAllowanceService
{
    public function __construct(
        private readonly Connection $db,
        private readonly SiteSettingService $settings,
        private readonly UsageRightsService $rights,
        private readonly UsageAllowanceRepository $allowances,
    ) {
    }

    /**
     * What this booking would spend against, and what is left.
     *
     * @return list<array{allowance:UsageAllowance,limit:int,used:int,remaining:int,exceededBy:int}>
     *         one row per budget — a 5 h/week and a 20 h/month limit are two
     *         answers and both have to hold
     */
    public function budgets(
        Utilisateur $user,
        ?string $featureKey,
        ?ReservableType $type,
        \DateTimeImmutable $start,
        int $requestedMinutes = 0,
        ?int $ignoreId = null,
    ): array {
        if (!$this->rights->isEnforced()) {
            return [];
        }

        $applicable = array_values(array_filter(
            $this->allowances->activeFor($user, $start),
            static fn (UsageAllowance $allowance): bool => $allowance->applies($featureKey, $type?->value),
        ));

        return $this->budgetsFrom($user, $applicable, $start, $requestedMinutes, $ignoreId);
    }

    /**
     * Every budget this member currently holds, spent and remaining (S144c).
     *
     * ⚠️ **Unfiltered on purpose.** `budgets()` answers "what would this booking
     * spend"; this answers "what do you have", which is the question a member has
     * before they book rather than after they are refused. Being told your limit
     * only at the moment of refusal is how a feature that was sold as generous
     * reads as arbitrary.
     *
     * @return list<array{allowance:UsageAllowance,limit:int,used:int,remaining:int,exceededBy:int}>
     */
    public function summaryFor(Utilisateur $user, ?\DateTimeImmutable $now = null): array
    {
        if (!$this->rights->isEnforced()) {
            return [];
        }

        $now ??= new \DateTimeImmutable('now', $this->labZone());

        return $this->budgetsFrom($user, $this->allowances->activeFor($user, $now), $now, 0, null);
    }

    /**
     * @param list<UsageAllowance> $applicable
     * @return list<array{allowance:UsageAllowance,limit:int,used:int,remaining:int,exceededBy:int}>
     */
    private function budgetsFrom(
        Utilisateur $user,
        array $applicable,
        \DateTimeImmutable $start,
        int $requestedMinutes,
        ?int $ignoreId,
    ): array {
        if ($applicable === []) {
            return [];
        }

        // ⚠️ Summed per budget, not per package. Two packages carrying 5 h a week
        // are 10 h a week, because no package may take away what another gave.
        $limits = [];
        foreach ($applicable as $allowance) {
            $key = $allowance->budgetKey();
            $limits[$key] ??= ['allowance' => $allowance, 'limit' => 0];
            $limits[$key]['limit'] += $allowance->amount;
        }

        $rows = [];
        foreach ($limits as $budget) {
            /** @var UsageAllowance $allowance */
            $allowance = $budget['allowance'];
            [$from, $until] = $allowance->windowAround($start);
            $used = $this->consumed($user, $allowance, $from, $until, $ignoreId);
            $requested = $allowance->unit === UsageAllowance::UNIT_MINUTES ? $requestedMinutes : ($requestedMinutes > 0 ? 1 : 0);
            $remaining = max(0, $budget['limit'] - $used);

            $rows[] = [
                'allowance' => $allowance,
                'limit' => $budget['limit'],
                'used' => $used,
                'remaining' => $remaining,
                'exceededBy' => max(0, $used + $requested - $budget['limit']),
            ];
        }

        return $rows;
    }

    /**
     * The first budget this booking would break, or null.
     *
     * ⚠️ Ordered by the caller's array, not by severity: the member is told about
     * one refusal, and the first configured budget is the one whose words the
     * operator wrote first. Ranking them by how badly they are exceeded would
     * make the message move around between attempts.
     *
     * @return array{allowance:UsageAllowance,limit:int,used:int,remaining:int,exceededBy:int}|null
     */
    public function firstExceeded(
        Utilisateur $user,
        ?string $featureKey,
        ?ReservableType $type,
        \DateTimeImmutable $start,
        \DateTimeImmutable $end,
        ?int $ignoreId = null,
    ): ?array {
        $minutes = max(0, (int) round(($end->getTimestamp() - $start->getTimestamp()) / 60));
        foreach ($this->budgets($user, $featureKey, $type, $start, $minutes, $ignoreId) as $budget) {
            if ($budget['exceededBy'] > 0) {
                return $budget;
            }
        }

        return null;
    }

    /**
     * What the member has already spent inside this budget's window.
     *
     * ⚠️ **Cancelled and declined bookings do not count**, the same
     * `INACTIVE_STATUSES` the quota counts use — an allowance that kept charging
     * for a cancelled slot would be a fine for changing your mind. Pending
     * requests DO count: the slot is held, and releasing it is what a decline is
     * for.
     *
     * ⚠️ `$ignoreId` is the booking being MOVED. Counting it would make every
     * reschedule of a member at their limit impossible, which is the same trap
     * `BookingPolicyService::check()` documents.
     */
    private function consumed(
        Utilisateur $user,
        UsageAllowance $allowance,
        ?\DateTimeImmutable $from,
        ?\DateTimeImmutable $until,
        ?int $ignoreId,
    ): int {
        if ($user->getId() === null) {
            return 0;
        }

        // ⚠️ The column is `userId`; the PROPERTY is `utilisateur`. This is raw
        // DBAL, so it must name the column — the Doctrine name would fail at
        // runtime and, inside this method's catch, would silently report every
        // member as having spent nothing.
        $where = ['r.userId = :user', 'r.statut NOT IN (:inactive)'];
        $params = ['user' => $user->getId(), 'inactive' => Reservation::INACTIVE_STATUSES];
        $types = ['inactive' => \Doctrine\DBAL\ArrayParameterType::STRING];

        if ($allowance->reservableType !== null) {
            $where[] = 'r.reservableType = :kind';
            $params['kind'] = $allowance->reservableType;
        }
        if ($from !== null) {
            $where[] = 'r.dateDebut >= :from';
            $params['from'] = $from->format('Y-m-d H:i:s');
        }
        if ($until !== null) {
            $where[] = 'r.dateDebut < :until';
            $params['until'] = $until->format('Y-m-d H:i:s');
        }
        if ($ignoreId !== null) {
            $where[] = 'r.id <> :ignore';
            $params['ignore'] = $ignoreId;
        }

        // TIMESTAMPDIFF over the stored bounds rather than a stored duration:
        // there is no duration column, and deriving it in PHP would mean loading
        // every reservation of the period to add up two numbers.
        $measure = $allowance->unit === UsageAllowance::UNIT_MINUTES
            ? 'COALESCE(SUM(TIMESTAMPDIFF(MINUTE, r.dateDebut, r.dateFin)), 0)'
            : 'COUNT(*)';

        try {
            return (int) $this->db->fetchOne(
                sprintf('SELECT %s FROM RESERVATION r WHERE %s', $measure, implode(' AND ', $where)),
                $params,
                $types,
            );
        } catch (\Throwable) {
            // ⚠️ Zero, not "refuse". A counting failure must not invent a member
            // who has spent their year: this service only ever narrows, so its
            // safe direction is to narrow nothing.
            return 0;
        }
    }

    /** The lab's clock, for windowing a period the way a human reads a calendar. */
    public function labZone(): \DateTimeZone
    {
        return new \DateTimeZone($this->settings->getTimezone());
    }
}
