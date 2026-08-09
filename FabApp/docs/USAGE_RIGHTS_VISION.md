# Usage Rights, audiences, scopes and booking quotas — target vision

**Status:** proposed architecture, not yet enforced · **Updated:** 2026-08-09

This document is the decision frame for the next Usage Rights sessions. The live implementation remains the feature-level S97–S99 system until each migration phase below is built, compared and explicitly activated.

## Product goal

An operator should be able to express one readable rule such as:

> Members may reserve 3D printers at the North workshop, up to four times per week and fourteen days ahead. Staff may use every workshop without those soft limits. Training and machine availability still apply.

The member should then see one answer explaining **whether**, **where**, **when** and **under which quota profile** an action is available.

## Keep five concepts separate

| Concept | Question | Source |
|---|---|---|
| Site feature | Does this installation expose the function at all? | `SiteFeatureRegistry` / portal feature state |
| Audience | Why does this person receive a package? | direct assignment, existing `ROLE`, authenticated users, or guests |
| Usage grant | Which action is opened? | package capability grant |
| Scope | Where and on which resources does the grant apply? | future physical venue, equipment category or one resource |
| Quota profile | How much and how far ahead? | named reusable booking-policy profile |

Badges, training, closures, overlap and capacity remain independent safety/availability rules. A package never turns them off.

## Audiences

Do not create a second enum containing `admin`, `staff`, `trainer` and `user`. Roles already live in the `ROLE` table and may be extended by an installation.

Proposed package sources:

- **Direct member assignment:** the existing time-bounded, audited assignment.
- **Role audience:** references an existing `ROLE.id`; membership changes take effect immediately.
- **Authenticated audience:** every active signed-in account. This is clearer than relying on the implicit `ROLE_USER` Symfony adds to every account.
- **Guest audience:** no pseudo-user row. It is valid only for capabilities that genuinely support anonymous use, currently guest event registration.

Effective packages are the union of all matching sources. There is no negative package or hidden deny priority. Direct packages add access; account suspension and operational bans remain separate explicit mechanisms. Administrator recovery access remains a system rule, not a fake “Admin package”.

## Physical venues are not portals

FabOS currently has one physical venue: one address, timezone and opening schedule. A `Portal` is a hostname/branding/feature façade over shared data; a `Place` is a reservable room or workstation. Neither is a second physical site.

Multi-site support therefore needs a new first-class **Venue** model before location-scoped rights:

- stable id, slug, name, address, timezone and active state;
- opening schedule ownership;
- machines, reservable places and relevant events assigned to a venue;
- bookable people either linked to one or more venues, or explicitly marked remote/venue-independent before person-booking grants can be venue-scoped;
- portal visibility may reference venues later, but portal and venue identities stay distinct.

The initial migration creates the current installation as one default venue and attaches existing resources to it. Nothing becomes multi-site merely because a second portal exists.

## Equipment categories

`Machine.categorySlug/categoryLabel` is useful presentation data today, but it has no referential integrity. Before a category becomes an authorization scope, create a canonical `MACHINE_CATEGORY` table and migrate existing slug/label/icon values into it. Machines then reference the category.

A category grant is useful: “all 3D printers” avoids maintaining one grant per machine. It should not imply pooled booking. “Any interchangeable machine” is a separate future reservation feature and must default off.

## Proposed grant shape

One package contains one or more grants. Each grant is one alternative path:

| Field | Meaning |
|---|---|
| capability | a key from `UsageCapabilityRegistry` |
| venue | optional physical venue |
| machine category | optional canonical category |
| resource | optional exact machine/place/person |
| schedule | anytime, opening hours, or a future custom window |
| quota profile | named profile, inherited audience profile, or explicitly unrestricted soft quotas |

Non-null scope dimensions combine with **AND** inside one grant: “3D printers at North workshop”. Multiple grants/packages combine with **OR**: either complete path may authorize the action.

## Quotas belong in reusable profiles

The existing `BOOKING_POLICY` matrix is keyed by reservable type and `BookingTier`. The target is portal-scoped, named profiles:

- `BOOKING_POLICY_PROFILE`: name, description, portal, active/audit state;
- `BOOKING_POLICY_RULE`: one complete rule per reservable type, carrying the current quota fields;
- a package grant references a profile or explicitly inherits its audience default.

Never merge fields from several profiles into a synthetic “best of each field” policy. That produces a policy nobody configured. Evaluate complete candidate profiles independently; a booking passes when one applicable grant and its complete profile pass.

Counts remain global for the member within the relevant reservable type unless a later product decision explicitly creates per-package credit wallets.

### Hard constraints versus soft quotas

These must be split before packages can safely reference quotas:

- **Hard operational/resource constraints:** explicit closure, slot alignment, turnaround buffer and safety-related duration rules. Every candidate must obey them.
- **Soft audience quotas:** notice, booking horizon, active booking cap, daily and weekly caps. A package profile or exceptional quota pass may change/waive these.

Ordinary opening hours are part of the grant/schedule model: an “opening hours” grant follows them, while a deliberately issued “anytime” grant may operate outside them. An explicit operational closure remains hard and cannot be bypassed by either schedule mode.

The current code has two known hazards to close first: type-specific quota cells currently count all reservation types together, and an exceptional access pass returns before alignment/buffer checks even though it is documented as a quota-only waiver.

## Deterministic evaluation order

1. Feature, venue and resource are enabled and not under an explicit operational closure.
2. Resolve direct, role, authenticated or guest package sources.
3. Find active grants covering capability, use interval and scope.
4. Apply badge, training and other safety gates.
5. Apply non-waivable scheduling/resource constraints.
6. Evaluate each complete candidate quota profile; an explicit quota pass can waive only this soft layer.
7. Apply overlap and capacity.

The refusal names the first actionable layer. A successful reservation records the winning package, grant and quota profile for explanation and audit. Existing reservations are grandfathered unless a separate reconciliation operation is explicitly run.

## Progressive delivery plan

### Phase 1 — repair the current quota foundation

- Make daily/weekly/active counts respect `ReservableType` and add regression tests.
- Separate hard constraints from soft quota checks.
- Make exceptional passes waive only soft quotas.

This adds no new package enforcement, but it intentionally changes some booking verdicts: type-specific counts may loosen a quota that was incorrectly shared across resource kinds, while hard alignment/buffer checks may tighten a slot previously admitted by an exceptional pass. Cover both changes with regression tests and surface the reason clearly.

### Phase 2 — canonical scopes

- Create the default physical Venue and attach machines/places/events.
- Create canonical machine categories and migrate current category data.
- Add admin CRUD using the shared list/detail patterns.

No package enforcement change yet.

### Phase 3 — audiences and profiles beside the legacy model

- Add package audiences referencing existing roles plus authenticated/guest audience types.
- Add named quota profiles/rules and import the four legacy member/trainer/staff/admin matrices.
- Add grant scopes and optional quota-profile references.
- Keep `BookingTier` authoritative while a shadow resolver compares decisions.

Legacy global rows become profiles in the global/default portal scope. Override portals initially inherit those profiles and may explicitly fork them; migration must not silently manufacture portal-local differences.

When a package inherits a quota profile, each matching audience binding supplies its own complete candidate profile. Those candidates are evaluated with OR like explicit package profiles; fields are never merged and there is no hidden “most specific audience” rule. The successful candidate is recorded. Administrator recovery bypasses the entitlement gate only and still follows the imported Admin quota profile plus all hard constraints; any future soft-quota bypass must be explicit and audited.

### Phase 4 — operator preflight and activation

- Show capability, venue/category and quota coverage per audience.
- Report uncovered members/guests, conflicting scopes and old/new verdict differences.
- Require explicit confirmation; never auto-create restrictive packages or silently enable enforcement.

### Phase 5 — switch and retire the legacy matrix

- Switch booking to the new resolver only after parity tests and live review.
- Keep legacy fallback for one release.
- Then remove the `BookingTier` policy editor; retain roles for authentication/administration.

## Decisions intentionally left open

- Whether custom member groups beyond roles are needed; add them only after a real use case.
- Whether venues can have different timezones in one installation.
- Whether category-level pooled booking is useful; it is not implied by authorization scope.
- Whether loans receive a package capability; their write chokepoint needs an audit first.
- RFID/physical-card enforcement remains deferred.

## Admin experience target

The normal package editor should remain a short path:

1. Name the package and choose its audiences.
2. Choose capabilities and optional scopes; “all venues/resources” is the default.
3. Choose an existing quota profile or “inherit audience default”.
4. Review one plain-language effective summary and impact count, then save.

Advanced scope intersections and custom schedules stay behind disclosure. A separate read-only simulator answers “why can this member reserve this resource at this time?” using the same verdict service as enforcement.
