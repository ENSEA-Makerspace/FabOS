# FabOS roadmap — from fablab tool to modular platform

**Written:** 2026-07-24 · **Status of the app:** S1–S20 shipped and live. This document covers **S21 onwards**.

---

## The goal in one sentence

**A deployment should be able to be only what it needs to be** — only a machine-booking system, only an event platform, only a training system (LMS), only a lending library — without the operator having to delete features, and without a newcomer having to understand the parts they aren't using.

Everything below serves that, plus one standing quality bar: **it has to be friendly to someone who has never seen the app before.**

---

## Design principles for this phase

These are the rules the sessions below are held to. They came out of building S1–S20 and from decisions made 2026-07-24.

**1. The calendar is the spine; features are layers.**
FabOS is a calendar-based app. Events, machine use, room booking and door access all revolve around one calendar. The polymorphic reservation model already implements this. Nothing in this phase should create a second, parallel booking concept.

**2. Three tiers, and only one of them is negotiable.**

| Tier | Contents | Toggleable? |
|---|---|---|
| **Kernel** | auth, profile, admin frame, settings, portals, mail backbone, the reservation/booking engine | Never. Invisible infrastructure. |
| **Resource modules** | machines, places/spaces, people-appointments — each contributes a bookable layer to the calendar | Yes |
| **Feature modules** | events, training/LMS, badges, loans, materials, maintenance, leaderboard, projects, lab pages, staff & trainer directories, emails | Yes |

**3. Surfaces are derived, not configured.**
The calendar page appears when there is at least one resource module on. A menu group appears when it has contents. An empty section renders as nothing, not as an empty box. Nobody should have to curate a menu to make the app look coherent.

**4. Module toggle = access control. Menu = presentation.**
Disabling a module 404s its routes (`ModuleAccessSubscriber`). Hiding a menu entry hides a link and nothing more. These must never be conflated in the UI — label them distinctly, and never imply that a hidden link is a closed door.

**5. Defaults over options — the "secondary click" rule.**
Apple ships a mouse whose right-click is off until you go looking. Adopt that posture: the default path should be short and obvious, and power features should exist behind a clearly-marked *Advanced* disclosure rather than in the main flow. A first-time admin should be able to create something useful on a single screen without meeting a concept they don't need yet.

**6. Push admin into the page, not into the panel.**
The admin panel has grown large enough to be intimidating. Where an action belongs to a thing that is already on screen, offer it **on that page, visible only to those who may do it** — an "Edit" affordance on the event page beats hunting through `/admin/events`. The panel remains the home for genuinely global configuration.

**7. Empty means "as before".**
Every new setting must have an unset state that reproduces today's behaviour exactly. This has held for booking quotas, access passes, reminders and portal scoping; keep it.

**8. Don't add a field before its reader.**
A stored setting nothing consults is worse than a missing one — it lies to the operator. (`rappelReservation` was writable from two screens and read by nothing for months.)

---

## Phase A — Modularity

*This is the precondition for the whole goal. Do it first, in order.*

### S21 · Machines become a module

**Why.** Machines are currently kernel, not a module. That single fact makes an events-only or LMS-only deployment impossible: the machine pages and a machine-shaped calendar are unremovable.

**Scope.**
- Add `machines` to `ModuleService::MODULES`; label in the admin modules screen; route gate in `ModuleAccessSubscriber`; nav gating in `_header`/`_footer`.
- Make the **calendar's machine layer conditional**, exactly as the place layer already is (`buildCalendarResources()` / `buildCalendarResourceAccess()`).
- **Derive the calendar page's own visibility**: if no resource module is enabled, the calendar link and page stand down rather than rendering an empty grid.
- Audit for collateral: the homepage machine blocks, `/machines`, machine favourites, the machine kiosk, the RFID door path, and cert-gating (which is *about* machines but lives in the booking layer).

**Out of scope.** Touching the reservation model. This is a visibility and gating change, not a data change.

**Verify.** Boot the app four ways — everything on, machines-only, events-only, LMS-only — and confirm no 500s, no empty menu shells, no orphaned links. Confirm a disabled module 404s its routes rather than merely hiding them.

**Deploy.** No migration needed (`ModuleService` defaults unknown keys to enabled, so existing installs are unaffected). Pure code — safe to ship code-first.

---

### S22 · Menus assemble themselves

**Why.** Nav is hardcoded in `_header.html.twig` as two curated dropdowns with ~15 scattered `module_enabled()` checks. Every new module means editing the header by hand, and an all-disabled group still renders its wrapper.

**Scope.**
- Grow `ModuleService::MODULES` from a flat key list into entries carrying **nav metadata**: group, order, route, i18n label key, required role.
- Header and footer render from that registry: an entry shows if its module is on and the viewer's role allows; **a group with no visible children is not rendered at all.**
- Keep today's grouping and today's labels. Items stay in the submenus they are in now.

**Out of scope — deliberately.** Admin-editable menus, custom labels, reordering, arbitrary external links. The operator has explicitly deferred these; a menu builder is its own feature and the derived version may well be enough forever.

**Verify.** Every module off in turn: no empty dropdowns, no dead links, no missing anchor. Snapshot the rendered nav before and after with everything enabled — it should be **byte-identical**, proving the refactor changed no output.

---

### S23 · First-run setup with deployment presets

**Why.** This is the friendly front door to everything above, and the highest-leverage session in the phase. Modularity that requires knowing which of 14 checkboxes to untick is not usable by a newcomer. A fresh install should *ask what kind of place this is* and configure itself.

**Scope.**
- A first-run wizard, shown while the install is unconfigured, that asks for: organisation name, public URL, address, timezone/locale, and **what this deployment is for**.
- **Presets** that select a module set: *Makerspace* (everything) · *Event venue* (events + emails) · *Training organisation* (training + badges + emails) · *Lending library* (loans + materials) · *Coworking / rooms* (places) · *Custom*. A preset is a starting point, not a lock — every module stays individually toggleable afterwards.
- A **setup health panel** in the admin listing what is not yet configured and why it matters ("no sender account → no mail will go out", "no public URL → tickets carry no QR"). Cheap to build, and it turns silent degradation into a visible checklist. Several existing features degrade quietly by design; this is where that becomes discoverable.
- Optional **clearly-labelled sample data**, idempotent, with a one-click wipe.

**Out of scope.** Multi-tenant provisioning (that's portals, Phase B). Docker/one-command deploy (infra phase).

**Verify.** Each preset produces a coherent app: menus, calendar and homepage all consistent with the chosen modules, no 500s, no empty shells. Wipe returns the install to clean.

---

### S24 · Module dependencies, stated honestly

**Why.** Some modules lean on others. Cert-gating needs `badges`. Training awards badges. Events use `emails` for confirmations. Today, disabling a dependency degrades something silently somewhere else.

**Scope.** Let modules declare **soft dependencies**; the admin modules screen warns clearly when a disable will lame something else ("Machines require Badges for certification gating — disabling Badges will let anyone book any machine"). **Warn, don't block** — an operator may genuinely want an ungated workshop.

**Verify.** Each declared dependency produces its warning; nothing hard-fails.

---

## Phase B — Portals

*Multi-tenant, one install. The operator wants this working before revisiting event polish.*

### S25 · Portal admin CRUD + branding

**Why.** The portal *mechanism* exists (`Portal`, `PortalRepository`, `PortalContext`, hostname resolution, portal-scoped settings and modules) but there is **no UI to create or configure one**, so none of it is reachable.

**Scope.** Create/edit/delete portals; hostname binding; per-portal module subset; per-portal branding (logo, theme colours, sender identity). Resolution stays hostname-only — subdomain, not path prefix (settled).

**Watch.** The default portal **owns no rows — it *is* the global scope** (`scopeId()` → 0). The UI must not offer to "edit the default portal's overrides" as if they were rows, or it will look broken.

---

### S26 · Per-portal home page

**Why.** The stated driving case: a tenant who only wants events should have the events page as their front door.

**Scope.**
- One setting: home is **either** the block homepage (default) **or** one enabled module's landing page, chosen from an allow-list of route names. **Never a free-text path** — that is an open-redirect and a 500 generator.
- **302 redirect**, not an internal forward: no controller duplication and the URL bar stays honest.
- Portal-scope the existing `HomepageSectionVisibility` blocks, which are *already* ordered and role-gated — they just aren't per-portal yet.

**Open question to settle in this session.** A portal with its own front door probably wants its own hero/branding on it, which overlaps S25's branding work. Decide once, together.

---

## Phase C — Usability & consistency

### S27 · One admin layout

**Why.** Three incompatible admin skeletons exist, each with copy-pasted inline CSS across ~53 templates. This is the direct cause of several visible bugs: panels with no padding, buttons styled nowhere, radios inflated to full width, 37 pages rendering the sidebar as a bare link list.

**Scope.** One shared admin base template owning the chrome, panel padding, form controls, buttons and flash styling. Migrate all admin pages to it and **delete the per-page copies**. Re-run the class audit (cross-reference every `class="…"` against defined selectors) and fix what it finds.

**Verify.** Visual pass over every admin page in **both themes** — this is the one session that genuinely needs eyes on screens rather than status codes.

---

### S28 · Admin actions where the content is

**Why.** Principle 6. The panel is large and growing; most edits are about a thing the user is already looking at.

**Scope.** For admins/staff, surface contextual actions inline on public pages — edit this event, add a session, check someone in, edit this machine — with a consistent, unobtrusive affordance. Keep global configuration in the panel. **Server-side authorisation on every action**; an inline button is a convenience, never the permission.

---

### S29 · Neutral vocabulary + organisation identity

**Why.** FabOS is not fablab-only, but the wording assumes it is: the event admin says *"Au fablab"*, the kiosk footer says *"the fablab website"*, and the address setting is called `lab_address`.

**Scope.** An admin-set organisation name and venue label, used everywhere the interface currently hardcodes "fablab"; sweep all five catalogs for the same assumption. The mail sender name is already configurable — extend that idea to the UI.

---

## Phase D — Training / LMS, built for beginners

*The operator's note: "training looks complicated — as many simple pages as possible while hiding the complexity."*

### The key structural insight

**A training session is an event with a curriculum attached.** Do not build a parallel registration system. The event engine already provides — and has been verified to provide — enrolment, capacity, waitlists with automatic promotion, per-attendee tickets and QR codes, door check-in with attendance records, reminder mail, and organiser cancellation with a reason. Reusing it means the LMS inherits all of that on day one, and there is exactly one concept of "signing up for a thing at a time".

**Attendance already equals completion evidence.** The check-in timestamp built for events is what a trainer needs to mark someone as having attended.

### Progressive disclosure — four levels, three toggles

The default is level 1. Each level is one switch, and nothing above level 1 appears until asked for.

| Level | What the admin does | What they get |
|---|---|---|
| **1 · A page** *(default)* | Title, description, image | A readable catalogue entry. Genuinely useful alone. |
| **2 · Sign-ups** | Flip one toggle | A roster. Reuses event registration wholesale. |
| **3 · Scheduled sessions** | Flip one toggle | Dated instances with capacity, waitlist, reminders, check-in. |
| **4 · Advanced** *(collapsed)* | Open "Advanced" | Awards a badge on completion · prerequisites · quiz · assigned trainer |

### S30 · Training content (level 1)

Revive the training model as a plain content type: title, description, image, category, duration, objectives, prerequisites-as-text, materials provided. A catalogue and a detail page. `TrainingEnrollment` is a neutralised stub and `Formation` has no date — treat this as a fresh, careful model, not a resurrection.

### S31 · Enrolment and sessions (levels 2–3)

A `TrainingSession` = a scheduled instance of a training. Reuse the event registration engine for signing up. Decide explicitly whether a session also **occupies a room/trainer on the calendar** (it should — it is a resource booking, and the polymorphic model already supports it) or merely displays.

### S32 · Completion, badges and snapshots (level 4a)

Attendance or trainer confirmation completes an enrolment; completion may award a badge. **Credentials are immutable: snapshot on award.** Freeze title, description, badge image, date and awarding trainer into the award record so later edits to the live training never rewrite what somebody earned. This closes the loop with cert-gating — an earned badge is what unlocks a machine.

### S33 · Quizzes and prerequisites (level 4b)

Optional, hidden by default. Quiz model, pass/fail gating a badge, prerequisite chains. Only build this once levels 1–3 are in real use.

---

## Phase E — Membership & billing

### S34+ · Memberships, then payments

Tiered dues, renewal, expiry, and access gated on an active membership. Payments as **separate modules** per billable thing (memberships, event tickets, material purchases, machine time), designed against a provider but **never handling live payment credentials in this workflow** — the operator wires their own keys.

This is also the gate for the event **price / paid-attendance** work deliberately left out of S20, and for material/consumable billing.

---

## Later, unchanged in priority

- **Activity feed as a shared contract** — every module publishes events with a severity, one admin feed, optional public kiosk (privacy-filtered: no incident data).
- **Control-box / IoT** — MQTT, device drivers, relay/interlock, power monitoring → real run-hours, fail-safe offline cache. Belongs *after* the permission trio, which is done, so this is now unblocked.
- **Incident tracker** (staff-only, GDPR-sensitive), analytics/reports, public credentials page, LDAP login.
- **Open-source readiness** — Docker one-command deploy, community translation, GDPR export/delete, REST API + webhooks, accessibility pass, backup/restore.
- **Known small debts** — delete the dead `public/js/calendar.js` (967 lines, referenced nowhere); finish the half-wired mobile nav; add a machine-delete route that cancels its bookings (none exists, and there is no cascade).

---

## How these sessions are sized

Each `S##` is intended as **one self-contained, deployable session**: build, deploy to the live container, verify, commit. If a session cannot be verified end-to-end it is too big and should be split. Every session ends with the app running.
