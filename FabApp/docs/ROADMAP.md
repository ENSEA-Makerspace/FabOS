# FabOS roadmap — from fablab tool to modular platform

**Written:** 2026-07-24 · **Last updated:** 2026-07-28 · **Status of the app:** S1–S23 shipped and live, then **capabilities and modules were collapsed into site features** (2026-07-28). **S25's health panel and S29's stylesheet extraction are in.** Next: the S29 visual pass (yours), then S24 — see *What to do next*.

---

## What to do next — advice as of 2026-07-28

**Before anything else, two things that are not sessions.**

1. **Push the branch.** Everything from S21 onward — S21, S22, S23, the collapse to site features, and S25's health panel — exists only on one Mac. `git push origin fix/creation-upload-duration-and-image`. Everything since S20 is unpushed and CT 210's own checkout is still `main`, so there is currently no second copy of any of it.
2. **Click through one real booking.** The booking *success* path has never been verified by the agent — it needs a login, and the firewall stops anonymous POSTs before the controller. Every refusal branch is tested; the happy path is assumed. S22 put a new gate at the top of `ReservationService::book()`, so this is the moment to confirm it in a browser.

**S25's health panel is done; only the wizard and sample data are left**, and both need a decision from you before they are worth building (see the S25 entry). The panel already teaches most of what the wizard would have, so the urgency has dropped — S24 (menus assembling themselves) is now a reasonable next move, though it changes nothing an operator can see: its own verify step is "the rendered nav should be byte-identical".

**✅ S29's extraction is done (2026-07-28)** — the 653 duplicated lines are gone, and a new admin screen no longer adds a copy. **What is left is the visual pass, and that one is yours:** every admin page in both light and dark. The dark-theme rules were written from the documented variable names and have never been looked at.

**Delete `LOCAL_ADMIN_BYPASS` before any of this reaches real users.** It auto-authenticates any loopback request to `/admin` or `/staff` as the first admin, and POSTs really execute. It has been invaluable for verification and it is a live hole. `LocalAdminAuthenticator` plus its `security.yaml` entry.

**✅ Settled 2026-07-28 — capabilities and modules are now one thing: site features.** S23's two-layer model was collapsed the same day it shipped. The catalogue was one-to-one, so the registry/derivation/deviation/Advanced-panel machinery bought a second vocabulary for the same choices, and every new feature would have had to be written down in two places that could disagree. `src/Feature/` now holds a single `SiteFeature` carrying the operator-facing name *and* the route-gating key, plus the metadata that used to be three separate constants (`LAYERS`, `CALENDAR_LAYERS`, `MODULE_BY_RESERVABLE`). Storage, keys and gates are unchanged — no migration. `feature_enabled()` in templates; `/admin/features` is the screen.

*Consequences for the plan:* **S26 (dependency warnings) now attaches to features, not capabilities** — a feature declares recommended companions and the screen warns. **S27's "per-portal capabilities" is per-portal features**, which `SITE_MODULE`'s existing portal scoping already supports. Anywhere below that says "capability", read "site feature".


---

## The goal in one sentence

**A deployment should be able to be only what it needs to be** — only equipment booking, only an event platform, only a training system, only a lending library — without the operator having to delete features, and without a newcomer having to understand the parts they aren't using.

Everything below serves that, plus one standing quality bar: **it has to be friendly to someone who has never seen the app before.**

---

## The central shift in this phase: features you can name

The admin used to see a flat list of fourteen "modules" and had to work out which ones added up to the thing they actually wanted. That is backwards — and the word *module* was doing several unrelated jobs at once.

> **Settled 2026-07-28.** S23 first introduced *capabilities* as a layer **over** modules, with a registry, a derivation, a deviation model and an *Advanced* disclosure joining the two. The catalogue turned out to be one-to-one, so the same day it shipped the two were **collapsed into one concept: the site feature.** A `SiteFeature` carries the operator-facing name *and* the key that gates routes. There is no second layer and no derivation. Read every "capability" below as "site feature".

| Layer | What lives here | Does the admin see it? |
|---|---|---|
| **Kernel** | auth, users & roles, profile, settings, portals, mail transport, the booking + calendar engine | No — this is just "the app" |
| **Site features** | *what this deployment does* — see the catalogue below | **Yes — these are the toggles** |
| **Surfaces** | whether a given page or menu entry is shown | Derived from the features |

Each feature declares what kind of thing it is, and that one declaration is read by the route gate, the calendar, the booking chokepoint and the admin screen alike:

| Group | What a feature of this kind owns | Examples |
|---|---|---|
| **resource** | a bookable kind — and whether bookings of it are accepted at all | `machines`, `places`, `person_booking` |
| **activity** | a feature domain, with its own pages and data | `events`, `formations`, `badges`, `projects`, `leaderboard`, `lab_pages`, `materials`, `loans` |
| **directory** | **a page and a menu entry, nothing else** | `staff`, `trainers` |

**Add-ons** are features with a `parent`: shown nested, and forced off by the service whenever the parent is off. `maintenance` under *Réserver de l'équipement* is the one that exists today.

**Worked example.** Turning on *Organiser des événements* leaves equipment alone — an event venue gets registration, tickets, check-in and the kiosk, and never sees a piece of equipment. Neither touches the other.

### The rule that still matters

**"Installed" and "visible" are different questions.** The clearest case is `staff`: the people, their roles and `ROLE_STAFF` authorisation are **kernel** and must never be switchable — the staff desk (pass issuing, ticket scanning) depends on them. What the `staff` feature controls is *whether a public directory page and menu entry exist*. Same for `trainers`. Booking someone's time is a third, separate thing (`person_booking`, which also has a per-person `bookable` flag).

> This distinction caused one real bug: the route gate matched `app_staff*` by prefix, so turning off the staff *directory* would also have 404'd the staff *desk*. Fixed by matching exactly — and **the underlying conflation was removed in S22**, where the three groups above were first named.

---

## The feature catalogue

The list an admin actually sees. Names describe **what you can do**, never what kind of organisation you are — calling one of these "fablab" would reintroduce the assumption that S31 exists to remove.

> **Vocabulary rule, settled 2026-07-24: the user-facing word is "equipment", not "machine".** It covers a laser cutter, a sewing machine, a microscope and a projector without implying a workshop, and it survives the de-fablab sweep. **Internally the module key, entity and routes stay `machines`/`Machine`** — renaming a `SITE_MODULE` key would need a migration, and renaming the entity would touch the reservation model, for no functional gain. So: *equipment* in every label, help text and catalog; `machines` in the code. S31 owns the sweep.

### Resource capabilities — the bookable kinds

| Capability | Internal modules | Optional add-ons |
|---|---|---|
| **Book equipment** | `machines` *(internal key)* | Maintenance backlog · Materials & stock · Physical access control |
| **Book rooms & spaces** | `places` | Kiosk / door display |
| **Book people (appointments)** | `person_booking` *(since S22)* | — |

This is the family the polymorphic reservation model was built for; they are siblings by construction, not by convention. Each one decides whether bookings of its kind are accepted at all — enforced at `ReservationService::book()`, not merely by hiding pages.

⚠️ **Being a resource capability and drawing a column on the calendar are two different things.** The calendar page exists if at least one **the `calendarLayer` flag** module is on (`machines`, `places`) and stands down otherwise. *Book people* is a full resource capability that is deliberately **not** on that grid — people are booked from their own pages — so listing it there would bring the calendar back as an empty grid for an appointments-only deployment.

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
Disabling a capability 404s the routes of the modules it owned (`FeatureAccessSubscriber`). Hiding a menu entry hides a link and nothing more. Never conflate these in the UI, and never imply that a hidden link is a closed door.

**5. Defaults over options — the "secondary click" rule.**
Apple ships a mouse whose right-click is off until you go looking. Adopt that posture: the default path should be short and obvious, and power features live behind a clearly-marked *Advanced* disclosure. A first-time admin should create something useful on one screen without meeting a concept they don't need yet. **This applies to the capability screen itself** — capabilities up front, the module truth underneath for whoever wants it.

**6. Push admin into the page, not into the panel.**
The admin panel has grown large enough to be intimidating. Where an action belongs to something already on screen, offer it **on that page, visible only to those who may do it** — an "Edit" affordance on the event page beats hunting through `/admin/events`. The panel keeps genuinely global configuration.

**7. A disabled capability goes quiet everywhere — pages *and* background jobs.**
Route gating is the visible half. The invisible half is timers, scanners and queued work: a feature that is off must stop *emailing people*, not just stop rendering. Four of the five reminder scanners were missing this check and would have kept mailing about disabled features. Every new background job inherits this rule.

**8. Disabling never destroys.**
Turning a capability off hides and 404s; it does not delete rows. Turning it back on restores what was there. This must stay true, or "try it and see" becomes dangerous and nobody will explore the capability screen at all.

**9. Empty means "as before".**
Every new setting must have an unset state reproducing today's behaviour exactly. This has held for booking quotas, access passes, reminders and portal scoping. Keep it.

**10. Don't add a field before its reader.**
A stored setting nothing consults is worse than a missing one — it lies to the operator. (`rappelReservation` was writable from two screens and read by nothing for months.)

---

## Phase A — Modularity

*This is the precondition for the whole goal. Do it in order.*

### S21 · Equipment becomes a resource module — ✅ shipped 2026-07-28

**Why.** Equipment was kernel. That single fact made an events-only or training-only deployment impossible: the equipment pages and an equipment-shaped calendar were unremovable. This session was deliberately first and deliberately narrow — no new concepts, just parity with how places already work.

**What landed.**
- `machines` is a module: `SiteFeatureRegistry`, admin label *Équipement (réservation)*, route gate, nav gating. The key stays `machines`; the operator-facing word is *equipment*.
- The route gate matches `app_machine…` as a prefix — every equipment route is named that way and nothing else is (`app_maintenance` diverges at the fourth letter) — plus `app_kiosk_machine` exactly.
- The calendar's equipment layer is conditional, exactly as the place layer was.
- **The calendar's visibility is derived.** `ModuleService::RESOURCE_MODULES` names the layers; `hasResourceLayer()` / Twig `has_resource_layer()` is the one question the route gate, header and footer all ask. With no layer on, `app_calendar*` 404s. Adding a third layer means one entry in that constant.
- `BookingReminderScanner` filters **per booking** rather than being switched off wholesale — it is the one scanner serving every layer at once, so equipment off must still leave room reminders going.
- Collateral swept: homepage featured-equipment block, the "book a slot" step card, search results, `/mes-reservations` calls to action, the calendar page's own equipment button, footer and header.

**Two shells found by the four-way boot, and fixed.**
- The **"Le Lab" nav group** rendered an empty dropdown under a heading that 404'd once all its children were gated off. Pre-existing, but only reachable now that equipment can leave that group. Its heading now follows whichever child page still exists, and the group drops entirely when none does.
- **`ModuleService::all()` was adopting rows for retired keys.** The `emails` row left behind when mail became kernel was still drawing a switch on the admin screen that controlled nothing. Unknown keys are now ignored.

**Deliberately left alone.** The RFID box endpoints (`/api/rfid/machines/…`) stay ungated — they are hardware, and *Physical access control* is its own capability later; gating them here would lock people out of a door as a side effect of a catalogue decision. The homepage stats tiles and the `how_it_works` cards are ungated for **every** module, not just this one; that is S23/S24's to fix uniformly rather than one module at a time. API routes have never been module-gated (`api_leaderboard`, `api_badges` likewise) — unchanged.

**Verified** on the live container by booting four ways — everything on, equipment-only, events-only, training-only — over HTTP: no 500s, every gated route **404s** rather than merely hiding, no empty menu shells, and the home page links to `/machines` and `/calendrier` exactly when it should. `/calendar/machine/{id}.ics` answers 403 without the feed token when enabled and 404 when not, which is the right precedence.

**Deployed** code-first with no migration, as planned.

---

### S22 · Untangle what is conflated — ✅ shipped 2026-07-28

**Why.** Rule two of the model. Until this was done, capabilities would have inherited today's conflation and the wrong things would have become switchable.

**The answer, and it is the thing to remember: `SiteFeatureRegistry`.** The word "module" was answering three questions at once, so the three are now named in code and rendered as groups on the admin screen:

| Layer | What a module of this kind owns | Modules |
|---|---|---|
| **resource** | a bookable kind — and whether bookings of it are accepted at all | `machines`, `places`, `person_booking` |
| **activity** | a feature domain, with its own pages and data | `events`, `formations`, `badges`, `projects`, `leaderboard`, `lab_pages`, `materials`, `loans`, `maintenance` |
| **directory** | **a page and a menu entry, and nothing else** | `staff`, `trainers` |

**What landed.**
- **People and roles are kernel and are not switchable.** `ROLE_STAFF` / `ROLE_TRAINER` authorisation, role membership and the staff desk (pass issuing, ticket scanning) depend on no module. The directory group on the admin screen says so in words, because the operator is the person most likely to assume otherwise.
- `staff` and `trainers` are **directory surfaces**, matched **exactly** (`app_staff`, `app_trainers`) rather than by prefix.
- **`person_booking`** is a resource module of its own, independent of either directory. It gates every `app_person*` route, the "book" button in the directories, the availability link in the profile and the nav.
- **Enforcement moved to the chokepoint.** `ReservationService::book()` refuses a booking whose resource layer is off (`RESOURCE_KIND_UNAVAILABLE`, 404). Route gating alone was not enough: `/api/reservations` speaks the polymorphic payload directly and would have kept accepting bookings for a kind the deployment does not offer. the feature's `reservable` is the one map, read by both the chokepoint and the reminder scanner.
- **The gallery left the leaderboard's namespace** — `/creations*`, `app_creation*`, with permanent redirects from the three old public GET paths. Redirects are named under the gallery's own gate, so with it off they 404 rather than redirecting onto a page that then 404s. POST endpoints moved without redirects: they are form targets, never a URL anyone holds, and a 301 is not reliably re-POSTed.
- The `app_leaderboard_creation…`-before-`app_leaderboard` **ordering dependency is gone**.
- The leaderboard's `stats` array — a user count, a machine count and a published-creation count — **was never rendered by the template**. Deleted along with the two repository injections feeding it, so a page that ranks people no longer queries the equipment and project tables on every request.
- `has_resource_layer()` → **`has_calendar_layer()`**. With a resource layer that deliberately is not on the grid, the old name had become a lie about which question it answers.

**`emails` was already settled** before this session (`fe36c67`): mail is kernel with a settings-level pause switch. S21 finished the job by making `ModuleService::all()` ignore its leftover row.

**Deliberately not done.** `person_booking` is **not** in `CALENDAR_LAYERS`. People are booked from their own pages and no column is drawn for them, so listing it there would bring `/calendrier` back as an empty grid for a deployment that books only people — the failure S21 removed. Add it the day people appear on the grid.

**Verified** across five boots on the live container — everything on, directories off, person-booking off, gallery-only, leaderboard-only. With both directories off the staff desk still answers and people can still be booked; gallery and leaderboard each work with the other disabled; old gallery URLs 301 while the gallery is on and 404 when it is off; and the booking chokepoint refuses each kind exactly when its module is off, proven with a past-dated probe that cannot write a row.

---

### S23 · Introduce capabilities — ✅ shipped 2026-07-28, ⚠️ **superseded the same day**

> **Read this first.** The two-layer model below was **collapsed into site features** hours after it shipped (`b294326`). `src/Capability/` no longer exists; `SiteFeature` carries both the operator-facing name and the route-gating key. The record is kept because the *reasoning* still applies — especially why module state stayed authoritative — and because it explains why there are inert `capability_*` rows in `SITE_SETTING`. Everything below is history.

**Why.** The heart of the phase — what turns fourteen implementation checkboxes into a question a newcomer can answer.

**What landed.** `src/Capability/` — a `Capability` value object, a `CapabilityRegistry` holding the catalogue, and a `CapabilityService` owning state and derivation. `/admin/capabilities` is the screen (`/admin/modules` 301s to it); the sidebar entry is now *Fonctionnalités*.

#### Two departures from the plan, both deliberate

**1. Module state stays authoritative in `SITE_MODULE`.** The plan had capability state authoritative with module rows kept *only* as deviations. Rejected for two reasons:

- Every existing install already has an explicit row for **every** module — the old admin form wrote them all on save — so on the day this shipped, every module would have read as a deviation.
- Deriving module state on read leaves an Advanced-panel untick nowhere to live. That *is* the "I unticked it and it came back" bug the plan was guarding against.

So capabilities are the operator's **intent**, persisted alongside (as `SITE_SETTING` rows, hence no migration); toggling one **writes** module rows. A deviation is a *diff* between intent and reality — informative, not authoritative. There is exactly one source of truth for "is this module on". What the operator sees is identical either way: capabilities, an Advanced panel showing what they imply with the differences marked, and a reset action.

**2. Capability saves apply as a delta, not a wholesale overwrite.** Only the modules whose *implication* moved are rewritten. That is what makes "a deviation survives a capability change" true, while a capability that actually owns the module still wins — the operator's newer, explicit choice should.

#### Other decisions worth knowing

- **One switch per module concept.** The catalogue offered *Materials & stock* both as an add-on of *Book equipment* and as a capability of its own. Union semantics make that work, but the screen would show two switches mirroring each other. It is one capability; *Book equipment* says in words that it also feeds "what this machine accepts". (The machine detail page already degrades on `materialsEnabled`, so the separation was correct in the code.)
- **Team directories are one capability over two modules**, as the catalogue had it. Publishing only one of the two lists is real but rare — and it is exactly what the Advanced override is for, which gives the deviation machinery a genuine use case rather than a hypothetical.
- **Add-ons are capabilities with a `parent`.** Stored, derived and applied identically; nested and dimmed in the UI, and still submitted while dimmed, so a parent switched off and back on restores its add-ons as the operator left them. Keeping them one concept is what stops progressive disclosure leaking into the model. Only one exists today: *Suivi de maintenance* under *Book equipment*, default off.
- **`recommends` was not built.** Its only reader is S26's warning UI. A stored field nothing consults lies to the operator (principle 10) — S26 adds it together with the warnings.
- **No `reservable` field either.** The `resource` group already says a capability books something and the feature's `reservable` already maps modules to kinds; a third copy of that fact is how the three drift apart.
- the registry surfaces any module no capability claims — it would otherwise be unreachable from this screen. Shown in the Advanced panel rather than thrown, so a half-wired new module cannot take the admin screen down.
- **Empty still means "as before".** Until the screen is saved once, capability state is read *back* from the current module state: it opens describing the install rather than proposing to change it, and an install that never visits it behaves exactly as it did.

**The catalogue is currently 1:1 between capability and module, bar team directories.** The union is therefore real code with no overlapping case to exercise yet; the first genuine overlaps arrive with the LMS (S32+), where *Train people* and *Credentials & badges* both touch badge award. Worth knowing before assuming that criterion was tested against overlap — it was tested against the real registry, which has none.

**Verified** on the live container: each capability alone produces a coherent app (equipment, equipment+maintenance, events, training, training+credentials, directories, appointments, gallery); turning any one capability off leaves every module its neighbours still need running; and a deviation survives an unrelated change, yields to its owning capability, and clears on reset. The form was exercised over HTTP — all three actions plus a rejected CSRF token.

**Out of scope, as planned.** Per-capability settings sprawl. A capability is on or off; its internals stay in their own admin screens.

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

### S25 · First-run setup — 🟡 **health panel shipped 2026-07-28, wizard + sample data outstanding**

**✅ Done — the setup health panel** (`/admin/setup`, sidebar *État de l'installation*). `SetupHealth` lists what is not configured **and what each gap costs**, because the whole point is that these gaps are invisible: mail that is never sent and never errors, a message with no link rather than a broken one, a ticket with no QR. Severity is by consequence, not rarity — **blocking** (people will try something that does not work) · **degraded** (works, quietly does less) · **info** (a deliberate choice worth confirming). Mail *paused* is a separate check from mail *never configured*. The panel is **read-only and links out**: every fix belongs on the screen that owns the setting, and a second place to edit the same thing is how they drift. Unresolved items sort above resolved ones, worst first. Verified healthy / all-features-off / events-only on the live container.

**⬜ Still to do.**
- **The first-run wizard.** Organisation name, public URL, address, timezone/locale, then site features. Two things to decide before building it: (a) what counts as "unconfigured" — there is no such flag today, and the S23 `capabilities_configured` key is now inert, so pick a signal deliberately rather than reusing that; (b) a global redirect-to-wizard interceptor on a *live* install is a real hazard, so gate it narrowly (admin routes only, never the public site) or make it a dismissible banner on the dashboard instead. The health panel already covers most of what the wizard would have taught, which lowers the urgency.
- **Sample data, idempotent, with a one-click wipe.** Deliberately not attempted by the agent: it writes production rows, and the wipe is the kind of irreversible action that should be a human's click. Needs the operator to drive it.

**Original scope, for reference.**

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

### S29 · One admin layout — 🟡 **stylesheet extracted 2026-07-28, visual pass outstanding**

**✅ Done.** `public/css/admin.css` owns the admin chrome. **49 rules were duplicated across the 55 admin templates — 653 lines** — and that duplication is the mechanism behind every bug listed below: a fix landed on whichever page someone was editing and nowhere else.

- Rules moved **byte-for-byte**, and the stylesheet is linked **before** each page's inline block, so light mode renders exactly as before and a page that genuinely needs to override something still can. What remains inline is only what is page-specific.
- **Corrections are at the end of the file, not folded into the move**, so they read as deliberate: radios/checkboxes opt out of `.form-field input { width: 100% }` · `.btn-action` (12 templates), `.btn-small` (14) and `.form-field.check` were used everywhere and defined nowhere · content after `form_end()` gets the gutter `.admin-edit-panel` never had · a dark-theme block, overriding rather than variabilising so light mode stays byte-identical.
- **Class audit re-run:** 6 undefined classes across all 55 pages, down from the set above. The remaining five are single-page and cosmetic, and are named in a comment in `admin.css` so the next audit knows they were judged, not missed.
- Checked: all **62 admin paths still answer**, and every page links the stylesheet.

**⬜ Still to do — and it needs you, not the agent.**
- **The visual pass over every admin page in both themes.** This is S29's actual acceptance criterion and the one session the plan says genuinely needs eyes on screens. The dark-theme block in particular is written from the documented variable names and has never been *looked at*.
- **The three skeletons still exist** (`.admin-edit-*`, `.admin-page`/`.admin-layout`, `.admin-header`/`.admin-main-content`) and are all served from the one file now. Collapsing them into one is the remaining half — but there is finally one place to do it, and doing it before the visual pass would mean debugging two changes at once.

**Original scope, for reference.**

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

A `TrainingSession` = a scheduled instance. Reuse the event registration engine for signing up.

#### One class, three calendars — settled 2026-07-24

The requirement: glance at a class and see **where** and **with whom**; look at the room's calendar and see the class sitting in it; look at the trainer's calendar and see it too.

**The answer is: duplicate the rows, but not the truth.** The session is the single source of truth for when/where/who; it **projects** an occupancy reservation onto each resource it consumes.

*Why not merge the model.* `RESERVATION` addresses exactly one target, `(reservableType, reservableId)`. Making that many-to-many would reach into overlap detection, both calendar builders, the quota engine, access passes and the booking mail — essentially everything built across S8–S20 — to serve one use case. The blast radius is not worth it.

*Why not "just display it".* If the room is not genuinely reserved, somebody books it out from under the class. A real reservation gets **collision detection for free**, which is the entire point of putting it on the calendar in the first place.

*So: one owner, N projections.* Creating a session with a room and a trainer writes two reservations — `place:{room}` and `user:{trainer}` — both stamped as **derived from that session**.

Rules that keep projections from rotting, each one preventing a specific failure:

- **Not individually cancellable.** Cancelling the room booking directly would leave a class with nowhere to happen. The cancel path must refuse and point at the session instead.
- **Re-synced on edit, cancelled with the session.** Moving the session moves its projections; calling it off releases them.
- **Skip quotas and passes, keep overlap.** A projection is not personal consumption, so it must not eat the trainer's weekly booking allowance — but it absolutely must still collide with anything else in that room. `ReservationService::book()` is the chokepoint, so this is a sibling `occupy()` path that runs the collision check and skips the quota layer, not a bypass of the chokepoint itself.
- **Label them.** `reservableLabel` already snapshots a name; the projection's `motif` should say which class, so the room's calendar reads "Formation: Laser 101" rather than an anonymous block.

*There is no third calendar.* The training view reads sessions directly — it needs no reservation at all. So this is one calendar with resource layers, plus a training listing, not three calendars to reconcile.

*Build it generic.* An event occupying the main hall is the same shape. Implement **resource occupancy** once, keyed by an owning record, and let both `TrainingSession` and `Event` use it — otherwise this gets written twice and drifts.

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
