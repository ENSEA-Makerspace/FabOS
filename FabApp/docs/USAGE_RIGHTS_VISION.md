# Access, groups, responsibilities, usage packages and quotas — target vision

**Status:** proposed architecture, not yet enforced · **Updated:** 2026-08-09

This document is the decision frame for the next Usage Rights sessions. The live implementation remains the feature-level S97–S99 system until each migration phase below is built, compared and explicitly activated.

## Product goal

An operator should be able to express two independent rules for the same people, such as:

> Volunteers may reserve workshop machines at any hour without soft quotas. They may also view Maintenance and record routine interventions, but they may not recommission a machine after a safety shutdown. Training and machine availability still apply.

The member and operator should see two explainable answers: **what the person may administer**, and **what resources the person may use, where, when and under which quota profile**.

## Keep eight concepts separate

| Concept | Question | Source |
|---|---|---|
| System account | Is this a normal user or the global recovery administrator? | authentication/security kernel |
| User group | Which people are managed together? | group membership; grants nothing by itself |
| Site feature | Does this installation expose the function at all? | `SiteFeatureRegistry` / portal feature state |
| Administrative responsibility | Which management action and admin page are permitted? | responsibility capability + scope |
| Usage package | Which member action is opened? | usage capability grant |
| Scope | Where and on which objects does that grant apply? | portal, future physical venue, equipment category or one resource, depending on the grant |
| Quota profile | How much and how far ahead? | named reusable booking-policy profile |
| Safety/availability | Is the operation safe and possible now? | badges, training, closures, overlap and capacity |

No layer silently replaces another. A group may assign both a responsibility and a usage package, but FabOS evaluates and audits them separately.

## System roles, groups and fine responsibilities

The target security kernel has only two concepts:

- **User:** the normal account for members, volunteers, trainers, portal managers and staff.
- **Global administrator:** installation-wide configuration and operational recovery. This is not a package audience or a convenient way to grant one feature.

Business categories such as Volunteers, Trainers, Event team and Loan managers become **user groups**, not an ever-growing set of security roles. A group grants nothing by itself. It can receive:

- one or more administrative responsibility sets;
- one or more usage packages;
- a validity window on each assignment.

Scope lives in each responsibility/package grant, never again on the group assignment: one owner prevents two filters from silently intersecting. Administrative capabilities are fine and code-defined, for example `maintenance.view`, `maintenance.plan`, `maintenance.intervention.create`, `.update` and `.complete`, with destructive operations and `maintenance.return_to_service` separately protected. Menu visibility reads the same effective responsibility as the voter/service enforcing the request; hiding a link is never the security boundary.

Responsibilities may be assigned directly to one user or through a group, using the same responsibility-set object and validity rules. Direct assignment is the exceptional path; groups remain the normal management path.

Usage-package sources are:

- **Direct member assignment:** the existing time-bounded, audited assignment.
- **Group audience:** membership changes take effect immediately.
- **Authenticated audience:** every active signed-in account. This is clearer than relying on the implicit `ROLE_USER` Symfony adds to every account.
- **Guest audience:** no pseudo-user row. It is valid only for capabilities that genuinely support anonymous use, currently guest event registration.

Existing `ROLE` rows are migration input: Staff/Trainer-style roles seed matching groups and memberships, then cease to be a second permanent business-authorization source. Effective responsibilities and packages are the union of active direct and group assignments. There is no negative group or hidden deny priority; account suspension and operational bans remain separate explicit mechanisms.

## Two authorization planes

Responsibilities and packages answer different questions and must never share one capability catalogue:

| | Administrative responsibility | Usage package |
|---|---|---|
| Example | manage routine maintenance | reserve machines 24/7 |
| Changes navigation | yes, for permitted admin pages | no |
| Enforced by | object/scope-aware voter or service; firewall only authenticates broadly | resource-use service chokepoint |
| Typical scope | admin domain, portal, category or object | venue, category/resource, schedule and quota |
| Must not grant | resource use | administration |

The user detail page may combine both in one effective summary, but it must preserve the source of each verdict rather than invent a “super volunteer” role.

## Physical venues are not portals

FabOS currently has one physical venue: one address, timezone and opening schedule. A `Portal` is a hostname/branding/feature façade over shared data; a `Place` is a reservable room or workstation. Neither is a second physical site.

Multi-site support therefore needs a new first-class **Venue** model before location-scoped rights:

- stable id, slug, name, address, timezone and active state;
- opening schedule ownership;
- machines, reservable places and relevant events assigned to a venue;
- bookable people either linked to one or more venues, or explicitly marked remote/venue-independent before person-booking grants can be venue-scoped;
- portal visibility may reference venues later, but portal and venue identities stay distinct.

The initial migration creates the current installation as one default venue and attaches existing resources to it. Nothing becomes multi-site merely because a second portal exists.

## Delegated portal administration

A portal manager remains a normal User with a responsibility scoped to one portal. With the current model, that responsibility may manage only presentation data genuinely owned by the portal: visual identity, homepage/banner, permitted feature overrides and safe portal-local settings. Hostname/domain routing stays Global-admin-only until domain verification or allowlisting exists. The current combined portal form must be split before delegation so a presentation manager cannot submit identity/routing fields. It cannot change users, machines, reservations, safety, other portals or global installation settings.

Portals currently expose shared events and resources; they do not own them. Delegating “events of this portal” therefore requires a real ownership/visibility relation on Event first. The same rule applies to loans and other shared data. Presentation scope must not pretend to be data ownership.

A resource published through two portals keeps one policy authority. Usage rights and quota counts follow the resource/venue authority, not the hostname used to reach it, so changing portal cannot bypass access or quotas.

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

For an administrative action:

1. Account is active and the feature exists.
2. Resolve direct and group responsibility assignments.
3. Require the exact administrative capability and scope.
4. Apply non-delegable domain invariants, audit and safety approvals.

For a usage action:

1. Feature, venue and resource are enabled and not under an explicit operational closure.
2. Resolve direct, group, authenticated or guest package sources.
3. Find active grants covering capability, use interval and scope.
4. Apply badge, training and other safety gates.
5. Apply non-waivable scheduling/resource constraints.
6. Evaluate each complete candidate quota profile; an explicit quota pass can waive only this soft layer.
7. Apply overlap and capacity.

The refusal names the first actionable layer. A successful reservation records the winning package, grant and quota profile for explanation and audit. Existing reservations are grandfathered unless a separate reconciliation operation is explicitly run.

**Target safety decision:** Global administrator bypasses package entitlement for recovery, but not machine badge/training requirements merely because the ambient account is admin. The current `ReservationService` administrator qualification bypass must be explicitly removed or replaced during migration. Any supervised emergency safety override is a separate, time-bounded and audited object; it is never implied by Global admin.

## Progressive delivery plan

### Phase 1 — repair the current quota foundation

- Make daily/weekly/active counts respect `ReservableType` and add regression tests.
- Separate hard constraints from soft quota checks.
- Make exceptional passes waive only soft quotas.

This adds no new package enforcement, but it intentionally changes some booking verdicts: type-specific counts may loosen a quota that was incorrectly shared across resource kinds, while hard alignment/buffer checks may tighten a slot previously admitted by an exceptional pass. Cover both changes with regression tests and surface the reason clearly.

### Phase 2 — groups and responsibilities beside the current roles

- Add user groups and audited memberships.
- Add a closed administrative-capability registry and reusable responsibility sets.
- Extract explicitly delegated routes from `AdminController`, whose class-level `#[IsGranted('ROLE_ADMIN')]` currently blocks every method. Their firewall rule requires an authenticated User only; an object/scope-aware voter or service decides each action. Global configuration stays in the Global-admin-only controller and every delegated write is service protected.
- Make navigation and server-side voters consume the same effective responsibility.
- Inventory every current Staff/Trainer consumer before migration: `/staff`, `StaffController`, tickets/passes, `BookingIdentityPolicy`, homepage visibility, directories, `BookingTier`, navigation and role columns. Seed groups from those roles, compare every consumer in shadow mode, then retire a role only after its full parity checklist passes.
- Keep User and Global administrator as the security-kernel concepts.

No existing administrator loses access during the shadow phase.

### Phase 3 — canonical scopes and ownership

- Create the default physical Venue and attach machines/places/events.
- Create canonical machine categories and migrate current category data.
- Add explicit portal ownership/visibility only to domains that need delegated portal content.
- Add admin CRUD using the shared list/detail patterns.

No package enforcement change yet.

### Phase 4 — package audiences and profiles beside the legacy model

- Add package audiences referencing groups plus authenticated/guest audience types.
- Add named quota profiles/rules and import the four legacy member/trainer/staff/admin matrices.
- Add grant scopes and optional quota-profile references.
- Keep `BookingTier` authoritative while a shadow resolver compares decisions.

Legacy global rows become profiles in the global/default portal scope. Override portals initially inherit those profiles and may explicitly fork them; migration must not silently manufacture portal-local differences.

When a package inherits a quota profile, each matching audience binding supplies its own complete candidate profile. Those candidates are evaluated with OR like explicit package profiles; fields are never merged and there is no hidden “most specific audience” rule. The successful candidate is recorded. Administrator recovery bypasses the entitlement gate only and still follows the imported Admin quota profile plus all hard constraints; any future soft-quota bypass must be explicit and audited.

### Phase 5 — operator preflight, activation and legacy retirement

- Show capability, venue/category and quota coverage per audience.
- Report uncovered members/guests, conflicting scopes and old/new verdict differences.
- Require explicit confirmation; never auto-create restrictive packages or silently enable enforcement.
- Switch booking to the new resolver only after parity tests and live review.
- Keep legacy fallback for one release.
- Then remove the `BookingTier` policy editor; retain only the User/Global-admin security-kernel distinction rather than business-role authorization.

## Decisions intentionally left open

- Whether venues can have different timezones in one installation.
- Whether category-level pooled booking is useful; it is not implied by authorization scope.
- Whether loans receive a package capability; their write chokepoint needs an audit first.
- RFID/physical-card enforcement remains deferred.

## Admin experience target

The normal group editor should remain a short path:

1. Name the group and choose its members.
2. Attach responsibility sets and their scopes.
3. Attach usage packages and their scopes.
4. Review a plain-language effective summary and impact count.

The normal package editor remains independent:

1. Name the package.
2. Choose capabilities and optional scopes; “all venues/resources” is the default.
3. Choose an existing quota profile or “inherit audience default”.
4. Review and save, then assign it to groups or members.

The user detail page shows Account, Groups, Responsibilities, Usage packages and explicit refusals. Advanced scope intersections and custom schedules stay behind disclosure. A separate read-only simulator answers both “why can this person administer this object?” and “why can this member use this resource at this time?” using the same services as enforcement.
