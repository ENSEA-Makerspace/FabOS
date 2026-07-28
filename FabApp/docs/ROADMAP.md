# FabOS roadmap — from fablab tool to modular platform

**Written:** 2026-07-24 · **Status of the app:** S1–S20 shipped and live. This document covers **S21 onwards**.

---

## The goal in one sentence

**A deployment should be able to be only what it needs to be** — only equipment booking, only an event platform, only a training system, only a lending library — without the operator having to delete features, and without a newcomer having to understand the parts they aren't using.

Everything below serves that, plus one standing quality bar: **it has to be friendly to someone who has never seen the app before.**

---

## The central shift in this phase: capabilities, not modules

Today the admin sees a flat list of fourteen "modules" and has to work out which ones add up to the thing they actually want. That is backwards. It also asks the wrong question, because the word *module* is currently doing four unrelated jobs at once:

| What it really is | Examples today | The problem |
|---|---|---|
| A feature domain with its own data and pages | events, loans, materials, maintenance, formations, badges, projects | Fine — these are real |
| A bookable resource layer on the calendar | places (machines soon) | Different kind of thing entirely |
| Visibility over data that is already core | `staff`, `trainers` | The **people are kernel**; the module only decides whether a directory page exists |
| Infrastructure enablement | `emails` | Closer to kernel than to a feature |

So this phase introduces a clean four-layer model. **The admin only ever chooses at the capability layer. Modules become internal.**

| Layer | What lives here | Does the admin see it? |
|---|---|---|
| **Kernel** | auth, users & roles, profile, settings, portals, mail transport, the booking + calendar engine | No — this is just "the app" |
| **Capabilities** | *what this deployment does* — see the catalogue below | **Yes — these are the toggles** |
| **Modules** | the internal units a capability switches on; route gating, nav registration, data ownership | Only under *Advanced* |
| **Surfaces** | whether a given page or menu entry is shown | Per-capability, mostly derived |

**Worked example.** Turning on *Run events* activates the `events` module and leaves `machines` alone — an event venue gets registration, tickets, check-in and the kiosk, and never sees a piece of equipment. Turning on *Train people* activates `formations`, and pulls in `badges` only if the operator also wants credentials. Neither touches the other.

### Two rules that fall out of this, and matter

**Capabilities are not a partition of modules.** `emails` serves events *and* reminders *and* the LMS. So a capability declares which modules it **requires** and which it **recommends**, and the enabled set is the **union across enabled capabilities**. Module state is therefore *derived*, with the Advanced panel showing explicit **deviations** from what the capabilities imply — otherwise there are two sources of truth and you get the classic "I unticked it and it came back" bug.

**"Installed" and "visible" are different questions.** The clearest case is `staff`: the people, their roles and `ROLE_STAFF` authorisation are **kernel** and must never be switchable — the staff desk (pass issuing, ticket scanning) depends on them. What the `staff` module actually controls is *whether a public directory page and menu entry exist*. Same for `trainers`. Booking someone's time is a third, separate thing (the `user` resource layer, which already has a per-person `bookable` flag).

> This distinction has already caused one real bug: `ModuleAccessSubscriber` gated `app_staff*` by route prefix, so turning off the staff *directory* would also have 404'd the staff *desk*. Fixed by matching exactly — but the underlying conflation is what this phase removes.

---

## The capability catalogue

The list an admin actually sees. Names describe **what you can do**, never what kind of organisation you are — calling one of these "fablab" would reintroduce the assumption that S31 exists to remove.

> **Vocabulary rule, settled 2026-07-24: the user-facing word is "equipment", not "machine".** It covers a laser cutter, a sewing machine, a microscope and a projector without implying a workshop, and it survives the de-fablab sweep. **Internally the module key, entity and routes stay `machines`/`Machine`** — renaming a `SITE_MODULE` key would need a migration, and renaming the entity would touch the reservation model, for no functional gain. So: *equipment* in every label, help text and catalog; `machines` in the code. S31 owns the sweep.

### Resource capabilities — each adds a layer to the shared calendar

| Capability | Internal modules | Optional add-ons |
|---|---|---|
| **Book equipment** | `machines` *(internal key)* | Maintenance backlog · Materials & stock · Physical access control |
| **Book rooms & spaces** | `places` | Kiosk / door display |
| **Book people (appointments)** | `user` resource layer | — |

The calendar page exists if **at least one** of these is on, and stands down otherwise. This is the family the polymorphic reservation model was built for; they are siblings by construction, not by convention.

### Activity capabilities

| Capability | Internal modules | Optional add-ons |
|---|---|---|
| **Run events** | `events` | Tickets & QR check-in · Kiosk signage |
| **Lend equipment** | `loans` | — |
| **Train people (LMS)** | `formations` | Sign-ups · Scheduled classes · Quizzes & prerequisites |
| **Credentials & badges** | `badges` | — *(awarded by the LMS; required by equipment cert-gating **and** by physical access — see coupling 1b)* |
| **Project gallery** | `projects` | — |
| **Leaderboard** | `leaderboard` | — |
| **Materials & stock** | `materials` | — |
| **Content pages** | `lab_pages` | — |
| **Team directories** | `staff`, `trainers` | — |

### Enforcement capabilities

| Capability | Depends on | Note |
|---|---|---|
| **Physical access control** (RFID / control box) | a resource capability **+ Credentials & badges** | Enforces what booking and certification already decided. Must never become a way to *bypass* cert-gating. |

### Kernel — never a toggle

Users, roles and authorisation · authentication · profiles · settings · portals · mail transport · the booking and calendar engine · the admin frame.

---

### Five couplings that are real, verified in the code

These are the cases where a naive capability list would be wrong. Each one has been checked against the source, not assumed.

**1. Badges are not an LMS feature.** `MachineQualificationService` depends on `UtilisateurBadgeRepository` — **certification gating *is* badges**. An equipment workshop that requires certification but runs no courses still needs them. So *Credentials & badges* is its own capability: **required** by machine cert-gating, **awarded** by the LMS. Nesting it inside the LMS would break equipment safety gating for anyone who didn't want a training system.

**1b. …and badges are not part of physical access either.** *(Asked and settled 2026-07-24.)* Both paths consume badges **independently**: `MachineQualificationService` gates **booking** — pure web, no hardware in sight — and `MachineAccessService` gates the **RFID tap**. Putting badges under *Physical access control* would mean a deployment with no reader hardware, which is the current live state and the majority case, silently loses certification gating on booking. That is a safety regression, not a simplification.

So badges have **one owner and three consumers**: the LMS **awards** them, equipment cert-gating **requires** them, physical access **requires** them. Badges and `emails` are the two clearest reasons the enabled-module set has to be a *union* rather than a partition.

**The diploma framing, and why it does not make badges part of the LMS.** A badge really is a diploma-ish thing, and that analogy is the clearest way to hold the design: **a diploma is *issued* by a school but *recognised* by employers who have nothing to do with that school.** Badges are the same — the LMS issues them, while equipment cert-gating and the RFID door *recognise* them without the LMS being involved at all. So they belong to the training **domain** conceptually while remaining **structurally independent**.

**Resolution:** in the admin, **group badges visually under Training** — that is where an operator will look for them, and the diploma intuition is right. In the model, keep them a **separate capability**. Presentation grouping is not dependency; that is the same surfaces-versus-capabilities line drawn everywhere else in this document. An equipment workshop that certifies people but runs no courses turns on *Credentials & badges* alone, and nothing about that should feel odd.

⚠️ **Beware the word "badge" — it means two unrelated things here, and worse in French.** `Badge` / `UtilisateurBadge` / `MachineBadge` are **credentials** ("has completed laser training"). `Utilisateur.identifiantRfid`, `RfidReader` and `AccessRfidLog` are the **physical card** you tap. Nothing should ever merge them, and the UI must name them differently — *credential* / *certification* versus *access card*.

**2. Team directories ≠ booking people.** Three separate things share the word "staff": the people and their roles (**kernel** — the staff desk depends on `ROLE_STAFF`), the public directory page (**a surface**), and whether someone's time is bookable (**a resource capability**, and there is already a per-person `bookable` flag). An event venue may well want a "our team" page with nobody bookable.

**3. Materials sit across two capabilities.** `Material` has a `MACHINE_MATERIAL` join — materials are partly "what this piece of equipment accepts" and partly a standalone stock catalogue. Offered as an add-on of *Book equipment* **and** as a small capability of its own, resolving to the same module either way.

**4. Projects and the leaderboard are route-coupled — and the coupling is historical, not essential.** *(Settled 2026-07-24: they are two different features and must be independent.)* Every gallery route currently lives under `/leaderboard/creations*`, yet the two rank different things: `app_leaderboard` ranks **people** (presence and prints, by period), while `CreationVote` is a **rating on a project**, making `creations/ranking` a *best-rated-projects* view. Untangling them is S22's second half.

### The add-on pattern

Add-ons are the progressive-disclosure rule applied *inside* a capability: they **do not appear until the parent capability is on**, and each has a sensible default (tickets on for events; maintenance off until asked for). This is what keeps the front page to roughly ten choices while still allowing real depth — the operator's own instinct for the LMS, generalised.

**Later capabilities**, not in the first catalogue: *Membership & billing*, *Incident tracker*, *Activity feed & public kiosk*.

---

## Design principles for this phase

**1. The calendar is the spine; features are layers.**
FabOS is a calendar-based app. Events, machine use, room booking and door access all revolve around one calendar. The polymorphic reservation model already implements this. Nothing here should create a second, parallel booking concept.

**2. The admin chooses capabilities; modules are implementation.**
See above. A capability screen a newcomer can answer beats a module list only the author understands.

**3. Surfaces are derived, not configured.**
The calendar page appears when there is at least one resource capability on. A menu group appears when it has contents. An empty section renders as nothing, not as an empty box. Nobody should have to curate a menu to make the app look coherent.

**4. Capability toggle = access control. Menu = presentation.**
Disabling a capability 404s the routes of the modules it owned (`ModuleAccessSubscriber`). Hiding a menu entry hides a link and nothing more. Never conflate these in the UI, and never imply that a hidden link is a closed door.

**5. Defaults over options — the "secondary click" rule.**
Apple ships a mouse whose right-click is off until you go looking. Adopt that posture: the default path should be short and obvious, and power features live behind a clearly-marked *Advanced* disclosure. A first-time admin should create something useful on one screen without meeting a concept they don't need yet. **This applies to the capability screen itself** — capabilities up front, the module truth underneath for whoever wants it.

**6. Push admin into the page, not into the panel.**
The admin panel has grown large enough to be intimidating. Where an action belongs to something already on screen, offer it **on that page, visible only to those who may do it** — an "Edit" affordance on the event page beats hunting through `/admin/events`. The panel keeps genuinely global configuration.

**7. Empty means "as before".**
Every new setting must have an unset state reproducing today's behaviour exactly. This has held for booking quotas, access passes, reminders and portal scoping. Keep it.

**8. Don't add a field before its reader.**
A stored setting nothing consults is worse than a missing one — it lies to the operator. (`rappelReservation` was writable from two screens and read by nothing for months.)

---

## Phase A — Modularity

*This is the precondition for the whole goal. Do it in order.*

### S21 · Equipment becomes a resource module

**Why.** Equipment is currently kernel. That single fact makes an events-only or training-only deployment impossible: the equipment pages and an equipment-shaped calendar are unremovable. This session is deliberately first and deliberately narrow — no new concepts, just parity with how places already work.

**Scope.**
- Add `machines` to `ModuleService::MODULES`; admin label; route gate; nav gating.
- Make the **calendar's equipment layer conditional**, exactly as the place layer already is (`buildCalendarResources()` / `buildCalendarResourceAccess()`).
- **Derive the calendar page's visibility**: with no resource module enabled, the calendar link and page stand down rather than rendering an empty grid.
- Audit the collateral: homepage equipment blocks, `/machines`, favourites, the equipment kiosk, the RFID door path, and cert-gating (which is *about* equipment but lives in the booking layer and must keep working).

**Out of scope.** The reservation model. This is visibility and gating, not data.

**Verify.** Boot four ways — everything on, equipment-only, events-only, training-only — with no 500s, no empty menu shells, no orphaned links. Confirm a disabled module **404s** rather than merely hiding.

**Deploy.** No migration (`ModuleService` defaults unknown keys to enabled, so existing installs are unaffected). Pure code, safe to ship code-first.

---

### S22 · Untangle what is conflated

**Why.** Rule two of the model. Until this is done, capabilities would inherit today's conflation and the wrong things would become switchable.

**Scope.**
- **People and roles move firmly to kernel** — never switchable. `ROLE_STAFF`/`ROLE_TRAINER` authorisation, the staff desk, and role membership all stop depending on any module.
- `staff` and `trainers` are reduced to what they actually are: **directory surfaces** (page + menu entry).
- **People-booking becomes its own resource capability** (the `user` reservable type), independent of whether directories are shown.
- Audit every remaining module for the same conflation and write down which layer each one belongs to. Expected finding: **`emails` is kernel infrastructure, not a feature** — booking confirmations are transactional and an install should not be able to stop them by flipping a module. Replace it with a settings-level "send notification mail" switch; `Mailer::isOperational()` already degrades gracefully with no sender configured. *(Behaviour change — confirm before building.)*

#### Second half — split the project gallery from the leaderboard

Two unrelated features share one route namespace, which is the same failure as the staff one wearing different clothes.

- Move the gallery out of `/leaderboard/creations*` into its own namespace, and rename its routes off the `app_leaderboard_*` prefix. **Keep 301 redirects** from the old paths — they are public, linkable and may well be bookmarked or shared.
- Update the `ModuleAccessSubscriber` mapping. This also **removes a fragile ordering dependency**: today `app_leaderboard_creation…` must be matched *before* `app_leaderboard`, or the gallery would inherit the leaderboard's gate. That is precisely the prefix trap that caused the staff bug, sitting in the code waiting.
- The leaderboard page currently injects `CreationRepository` for a projects widget. Make it **conditional on the gallery capability**, or the leaderboard renders project content for a deployment that has no gallery.
- Sweep nav and templates for links to the old route names.
- Keep the module key `projects` (it is a `SITE_MODULE` row — renaming it would need a migration for no gain) and change only its **label** to "Project gallery".

**Why it matters beyond tidiness.** With `leaderboard` off and `projects` on, the gallery today still answers on `/leaderboard/...` — a URL path named after a feature the deployment has disabled.

**Verify.** With every directory surface off, the staff desk, pass issuing and ticket scanning still work; role-gated routes still authorise correctly. Gallery and leaderboard each work with the other disabled, old gallery URLs 301 to the new ones, and the leaderboard shows no project content when the gallery is off.

---

### S23 · Introduce capabilities

**Why.** The heart of the phase. This is what turns fourteen implementation checkboxes into a question a newcomer can answer.

**Scope.**
- A **capability registry**: key, label, description, the modules it requires, the modules it recommends, and which resource layer (if any) it contributes.
- Starting set: **the capability catalogue above**, with its add-ons. Resolve the four verified couplings listed there before wiring the registry — especially badges, which must be independently available to machine cert-gating.
- Enabled modules are the **union of what the enabled capabilities require**. Persist capability state; persist module rows **only as explicit deviations**, shown as such, with a "reset to what my capabilities imply" action.
- The admin modules screen becomes a **capability screen**: cards with plain-language descriptions of what each one gives you, and an *Advanced* disclosure revealing the derived module state.

**Out of scope.** Per-capability settings sprawl. A capability is on or off; its internals stay in their own admin screens.

**Verify.** Each capability alone produces a coherent app. Toggling a capability off does not disable a module another enabled capability still requires. A deviation survives a capability change and can be reset.

---

### S24 · Menus assemble themselves

**Why.** Nav is hardcoded in `_header.html.twig` as two curated dropdowns with ~15 scattered `module_enabled()` checks. Every new module means hand-editing the header, and an all-disabled group still renders its wrapper.

**Scope.**
- Grow module entries to carry **nav metadata**: group, order, route, i18n label key, required role.
- Header and footer render from the registry: an entry shows if its module is enabled and the viewer's role allows; **a group with no visible children is not rendered at all.**
- Keep today's grouping and labels. Items stay in the submenus they are in now.

**Out of scope — deliberately.** Admin-editable menus, custom labels, reordering, external links. Explicitly deferred by the operator; the derived version may be enough permanently.

**Verify.** With everything enabled, snapshot the rendered nav before and after — it should be **byte-identical**, proving the refactor changed no output. Then each capability off in turn: no empty dropdowns, no dead links.

---

### S25 · First-run setup

**Why.** Now nearly free, because the wizard just asks for capabilities — the same concept the admin manages forever after, not a separate preset system.

**Scope.**
- A first-run wizard while the install is unconfigured: organisation name, public URL, address, timezone/locale, then **capabilities**.
- A **setup health panel** listing what is not yet configured and why it matters ("no sender account → no mail goes out", "no public URL → tickets carry no QR"). Several existing features degrade silently *by design*; this is where that becomes discoverable.
- Optional **clearly-labelled sample data**, idempotent, with a one-click wipe.

**Verify.** Each capability combination produces a coherent app. Wipe returns the install to clean.

---

### S26 · Capability dependencies, stated honestly

**Why.** Capabilities lean on each other. Cert-gating needs badges. Training awards badges. Events want mail.

**Scope.** Capabilities declare *recommended* companions; the capability screen warns clearly when a choice will lame something ("Machine workshop uses Training & badges for certification gating — without it, anyone can book any machine"). **Warn, don't block** — an ungated community workshop is a legitimate choice.

**Verify.** Each declared relationship produces its warning; nothing hard-fails.

---

## Phase B — Portals

*Multi-tenant, one install. The operator wants this working before revisiting event polish.*

### S27 · Portal admin CRUD + branding

**Why.** The portal *mechanism* exists (`Portal`, `PortalRepository`, `PortalContext`, hostname resolution, portal-scoped settings and modules) but there is **no UI to create or configure one**, so none of it is reachable.

**Scope.** Create/edit/delete portals; hostname binding; **per-portal capabilities** (which is now the natural unit — a tenant picks what their portal is for); per-portal branding (logo, theme colours, sender identity). Resolution stays hostname-only — subdomain, not path prefix (settled).

**Watch.** The default portal **owns no rows — it *is* the global scope** (`scopeId()` → 0). The UI must not offer to "edit the default portal's overrides" as though they were rows, or it will look broken.

---

### S28 · Per-portal home page

**Why.** The driving case: a tenant who only wants events should have the events page as their front door.

**Scope.**
- One setting: home is **either** the block homepage (default) **or** one enabled capability's landing page, from an allow-list of route names. **Never a free-text path** — that is an open-redirect and a 500 generator.
- **302 redirect**, not an internal forward: no controller duplication, and the URL bar stays honest.
- Portal-scope the existing `HomepageSectionVisibility` blocks, which are *already* ordered and role-gated — they just aren't per-portal yet.

**Open question for this session.** A portal with its own front door probably wants its own hero/branding, overlapping S27. Decide once, together.

---

## Phase C — Usability & consistency

### S29 · One admin layout

**Why.** Three incompatible admin skeletons exist, each with copy-pasted inline CSS across ~53 templates. Direct cause of several visible bugs: panels with no padding, buttons styled nowhere, radios inflated to full width, 37 pages rendering the sidebar as a bare link list.

**Scope.** One shared admin base template owning chrome, panel padding, form controls, buttons and flash styling. Migrate every admin page to it and **delete the per-page copies**. Re-run the class audit (cross-reference every `class="…"` against defined selectors).

**Verify.** Visual pass over every admin page in **both themes** — the one session that genuinely needs eyes on screens, not status codes.

---

### S30 · Admin actions where the content is

**Why.** Principle 6. The panel is large and growing; most edits concern something the user is already looking at.

**Scope.** For admins and staff, surface contextual actions inline on public pages — edit this event, add a session, check someone in, edit this machine — with one consistent, unobtrusive affordance. Global configuration stays in the panel. **Server-side authorisation on every action**; an inline button is a convenience, never the permission.

---

### S31 · Neutral vocabulary + organisation identity

**Why.** FabOS is not fablab-only, but the wording assumes it is: the event admin says *"Au fablab"*, the kiosk footer says *"the fablab website"*, the address setting is `lab_address`.

**Scope.** An admin-set organisation name and venue label used wherever the interface hardcodes "fablab"; sweep all five catalogs for the same assumption, **and for "machine" in user-facing strings — the word is "equipment"** (see the vocabulary rule in the catalogue). The mail sender name is already configurable — extend that idea to the UI.

---

## Phase D — Training / LMS, built for beginners

*The operator's note: "training looks complicated — as many simple pages as possible while hiding the complexity."*

### The key structural insight

**A training session is an event with a curriculum attached.** Do not build a parallel registration system. The event engine already provides — and has been verified to provide — enrolment, capacity, waitlists with automatic promotion, per-attendee tickets and QR codes, door check-in with attendance records, reminder mail, and organiser cancellation with a reason. Reusing it means the LMS inherits all of that on day one, and there is exactly one concept of "signing up for a thing at a time".

**Attendance already equals completion evidence.** The check-in timestamp built for events is precisely what a trainer needs to mark someone as having attended.

### Progressive disclosure — four levels, three toggles

Default is level 1. Each level is one switch, and nothing above level 1 appears until asked for.

| Level | What the admin does | What they get |
|---|---|---|
| **1 · A page** *(default)* | Title, description, image | A readable catalogue entry. Genuinely useful alone. |
| **2 · Sign-ups** | One toggle | A roster. Reuses event registration wholesale. |
| **3 · Scheduled sessions** | One toggle | Dated instances with capacity, waitlist, reminders, check-in. |
| **4 · Advanced** *(collapsed)* | Open "Advanced" | Awards a badge on completion · prerequisites · quiz · assigned trainer |

### S32 · Training content (level 1)

Revive training as a plain content type: title, description, image, category, duration, objectives, prerequisites-as-text, materials provided. A catalogue and a detail page. `TrainingEnrollment` is a **neutralised stub** and `Formation` has no date — treat this as a fresh, careful model, not a resurrection.

### S33 · Enrolment and sessions (levels 2–3)

A `TrainingSession` = a scheduled instance. Reuse the event registration engine for signing up. Decide explicitly whether a session also **occupies a room and a trainer on the calendar** — it should, because it *is* a resource booking and the polymorphic model already supports it.

### S34 · Completion, badges and snapshots (level 4a)

Attendance or trainer confirmation completes an enrolment; completion may award a badge. **Credentials are immutable: snapshot on award** — freeze title, description, badge image, date and awarding trainer into the award record, so later edits to the live training never rewrite what somebody earned. This closes the loop with cert-gating: an earned badge is what unlocks a machine.

### S35 · Quizzes and prerequisites (level 4b)

Optional, hidden by default. Quiz model, pass/fail gating a badge, prerequisite chains. Build only once levels 1–3 are in real use.

---

## Phase E — Membership & billing

### S36+ · Memberships, then payments

Tiered dues, renewal, expiry, and access gated on active membership. Payments as **separate modules** per billable thing (memberships, event tickets, material purchases, machine time), designed against a provider but **never handling live payment credentials in this workflow** — the operator wires their own keys.

Also the gate for the event **price / paid-attendance** work deliberately left out of S20, and for material and consumable billing.

---

## Later, unchanged in priority

- **Activity feed as a shared contract** — every module publishes events with a severity; one admin feed; optional public kiosk (privacy-filtered, no incident data).
- **Control-box / IoT** — MQTT, device drivers, relay/interlock, power monitoring → real run-hours, fail-safe offline cache. Was blocked on the permission trio, which is done, so this is now unblocked.
- **Incident tracker** (staff-only, GDPR-sensitive), analytics/reports, public credentials page, LDAP login.
- **Open-source readiness** — Docker one-command deploy, community translation, GDPR export/delete, REST API + webhooks, accessibility pass, backup/restore.
- **Known small debts** — delete the dead `public/js/calendar.js` (967 lines, referenced nowhere); finish the half-wired mobile nav; add a machine-delete route that cancels its bookings (none exists, and there is no cascade).

---

## How these sessions are sized

Each `S##` is **one self-contained, deployable session**: build, deploy to the live container, verify, commit. If a session cannot be verified end-to-end it is too big and should be split. Every session ends with the app running.
