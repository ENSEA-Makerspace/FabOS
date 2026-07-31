# FabOS roadmap — from fablab tool to modular platform

**Written:** 2026-07-24 · **Last updated:** 2026-07-31 · **Status of the app:** S1–S23 shipped and live, then **capabilities and modules were collapsed into site features** (2026-07-28). **Phase A is complete (S21–S26).** S25's health panel and S29's stylesheet extraction are in too. **S37, S27, S28 shipped 2026-07-30; S29 and S30 2026-07-31.** The live site runs `prod`, portals are reachable and brandable, and admin dark mode has been looked at. ⚠️ **Next: Phase H (S38–S44) — hardening, and it runs before Phase D.** S38 first: the public API is publishing badge UIDs today. See *What to do next*.

---

## What to do next — advice as of 2026-07-31

**Before anything else, two things that are not sessions.**

1. **Keep pushing.** The branch was pushed on 2026-07-28 and is current through S25's health panel; anything after that is again Mac-only until you push. ⚠️ CT 210's own checkout is still `main` and every deploy there is hand-placed, so the branch on GitHub is the only second copy.
2. **Click through one real booking.** The booking *success* path has never been verified by the agent — it needs a login, and the firewall stops anonymous POSTs before the controller. Every refusal branch is tested; the happy path is assumed. S22 put a new gate at the top of `ReservationService::book()`, so this is the moment to confirm it in a browser.

**✅ S25 is finished (2026-07-31).** The health panel and the wizard both shipped; **sample data was dropped at the operator's request** and is not pending work — see the S25 entry.

⚠️⚠️ **Phase H was added 2026-07-31 and it goes before Phase D.** Most of it is a codebase audit from 2026-07-10 that was parked before anything was implemented; it was re-checked against the live site and **the critical findings are still true**. The first one is not theoretical: `https://fabos.dstei.fr/api/leaderboard` currently returns real names paired with their **physical badge UID**, unauthenticated, from the internet — and the RFID device endpoints have no authentication at all, because the token check returns "allowed" when the token is unset. **Start at S38.**

**✅ S29 is done bar the skeletons (2026-07-31)** — the 653 duplicated lines went in the extraction, and the **visual pass has now been done in both themes**, which the plan had assumed only you could do. It found 108 hardcoded light backgrounds across 40 templates and a status palette that failed dark everywhere; see the S29 entry. What remains is collapsing the three skeletons into one, which is refactoring rather than a bug hunt.

**✅ Fixed 2026-07-30 — the live site runs `prod`.** It had been on `APP_ENV=dev`, so every public 404 returned Symfony's development exception page with the routing internals in an HTML comment, and the profiler was reachable by anyone. `APP_ENV=prod` now sits in `/opt/fabos/FabApp/.env.local` (overriding `.env`, which was left untouched; `.env.local.bak-20260730` is the backup). Verified: a public 404 carries no debug markers, `/_profiler` 404s, every public page still answers.

**⚠️ `LOCAL_ADMIN_BYPASS` went inert with it, and that changes how anything gets verified.** The bypass required debug mode as one of its three conditions, so `prod` switched it off on its own — loopback requests to `/admin` now 302 to `/login` like anyone else's. Admin screens stopped being inspectable from inside the container, which matters because most of what is left on this roadmap is admin-shaped.

**✅ The replacement is deployed and working (2026-07-30).** `LocalAdminAuthenticator` is gone, replaced by `ConsoleRenderAuthenticator` + **`php bin/console app:render <path>`**, which runs a real request through the real kernel from a shell and cannot be armed by a request at all — a method call on the service instance arms it, with `PHP_SAPI === 'cli'` as a structural backstop. Confirmed on the box: `app:render /admin/features` → 200, loopback HTTP → 302. This is now *the* way to inspect an admin screen, and S27/S28/S29 were all built with it. `LOCAL_ADMIN_BYPASS=1` can come out of `.env.local`, where it now means nothing.

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

### S24 · Menus assemble themselves — ✅ shipped 2026-07-28

**What landed.** `src/Nav/NavBuilder.php` returns the header as an ordered list of links and groups, already filtered; `_header` and `_footer` render that list. Fifteen hand-written `feature_enabled()` checks and a nested-ternary landing-page chain are gone.

The three rules it now states **once** rather than fifteen times:

1. **A group with no visible children is not rendered at all** — not as an empty dropdown, and not as a heading linking to a 404.
2. **A group's own link follows its children.** Where the natural landing page is gone, the heading points at the first child still there.
3. **Visibility is presentation, never permission.** Hiding an entry hides a link; the route gate and the firewall decide what exists. Nothing in the nav may be the only thing between a user and a page.

⚠️ **The calendar group keeps its own landing rule** rather than inheriting rule 2: it goes to `/calendrier` whenever anything is drawn on the grid, even when the first child listed (equipment) is what is off. That was the one place the old chain was *not* first-visible-child, and a naive rewrite would have lost it silently.

**Verified as the plan asks.** The rendered nav was snapshotted **before** the change across four feature combinations — everything on, events-only, equipment-only, training-only — and the header and footer link lists come back **byte-identical** after. The refactor changes no output, which was the whole claim.

**Still out of scope, deliberately.** Admin-editable menus, custom labels, reordering, external links. The derived version may well be enough permanently.

**Original scope, for reference.**

**Why.** Nav is hardcoded in `_header.html.twig` as two curated dropdowns with ~15 scattered `module_enabled()` checks. Every new module means hand-editing the header, and an all-disabled group still renders its wrapper.

**Scope.**
- Grow module entries to carry **nav metadata**: group, order, route, i18n label key, required role.
- Header and footer render from the registry: an entry shows if its module is enabled and the viewer's role allows; **a group with no visible children is not rendered at all.**
- Keep today's grouping and labels. Items stay in the submenus they are in now.

**Out of scope — deliberately.** Admin-editable menus, custom labels, reordering, external links. Explicitly deferred by the operator; the derived version may be enough permanently.

**Verify.** With everything enabled, snapshot the rendered nav before and after — it should be **byte-identical**, proving the refactor changed no output. Then each capability off in turn: no empty dropdowns, no dead links.

---

### S25 · First-run setup — ✅ **complete** (health panel + wizard; sample data dropped)

**✅ Done — the setup health panel** (`/admin/setup`, sidebar *État de l'installation*). `SetupHealth` lists what is not configured **and what each gap costs**, because the whole point is that these gaps are invisible: mail that is never sent and never errors, a message with no link rather than a broken one, a ticket with no QR. Severity is by consequence, not rarity — **blocking** (people will try something that does not work) · **degraded** (works, quietly does less) · **info** (a deliberate choice worth confirming). Mail *paused* is a separate check from mail *never configured*. The panel is **read-only and links out**: every fix belongs on the screen that owns the setting, and a second place to edit the same thing is how they drift. Unresolved items sort above resolved ones, worst first. Verified healthy / all-features-off / events-only on the live container.

**✅ The wizard shipped 2026-07-31**, at `/admin/wizard`, sidebar *Configuration initiale*. Both of the questions this entry said to settle first are settled, and both the conservative way.

**"Unconfigured" is an explicit flag, never an inference.** `setup_completed_at` is written once — by finishing or by skipping — and nothing else moves it. Deciding from whether settings happen to be empty is wrong in both directions: an operator deliberately running without a public URL would be nagged forever, and one who filled a single field would be declared finished.

⚠️ **A missing flag alone is not enough to call an install fresh, and this is the part that protects you.** Every install predating this code has no flag, so the flag by itself would show a first-run wizard to a lab that has run for a year. `FirstRun::isFresh()` also requires the install to *look* empty, counting operator-created content (machines, events, training, spaces, lab pages) rather than users — SSO or an import can create accounts on a site nobody has configured. It fails safe toward "already set up": if the tables cannot be read, it assumes there is content.

**Nothing redirects.** This entry warned that a global redirect-to-wizard interceptor is a real hazard on a live install; it is, because every route that forwards is a route that can strand someone. The wizard is reached from a dashboard card (shown only when fresh) and the sidebar — so the worst a bug in it can do is render a page badly.

It asks **only for settings that already have readers**: organisation and venue (S31), public URL, address, locale. Features get a *link*, not a copy of their screen — two places to switch the same thing on is how they drift apart. Skipping is a real answer and also sets the flag.

**Verified on the live container:** reports *already set up*, the dashboard card does not render, and setting then clearing the flag flips `completedAt` while `isFresh()` stays false throughout. ⚠️ The genuinely-empty case is **not** exercised — proving it would mean emptying five tables on a production database.

**❌ Sample data: dropped 2026-07-31, at the operator's request.** It was the last item here and it is not coming back unless asked for. Worth recording *why* it is a fair thing to drop rather than a gap: the health panel already teaches what a populated install would have demonstrated, and the feature is unusually expensive for what it gives — it writes production rows, and a believable demo set has to be maintained in step with every schema change forever. **If it is ever revived, the wipe is the hard half, not the seeding.**

**Original scope, for reference.**

**Why.** Now nearly free, because the wizard just asks for capabilities — the same concept the admin manages forever after, not a separate preset system.

**Scope.**
- A first-run wizard while the install is unconfigured: organisation name, public URL, address, timezone/locale, then **capabilities**.
- A **setup health panel** listing what is not yet configured and why it matters ("no sender account → no mail goes out", "no public URL → tickets carry no QR"). Several existing features degrade silently *by design*; this is where that becomes discoverable.
- ~~Optional **clearly-labelled sample data**, idempotent, with a one-click wipe.~~ *(dropped 2026-07-31)*

**Verify.** Each capability combination produces a coherent app. Wipe returns the install to clean.

---

### S26 · Feature dependencies, stated honestly — ✅ shipped 2026-07-28

**What landed.** `SiteFeature::$recommends` (the field S23 deliberately deferred until it had a reader) plus `FeatureAdvice`, which turns it into warnings on the feature screen: a feature that is **on**, leaning on a companion that is **off**. A recommendation from a feature that is itself off raises nothing — nobody is relying on it.

**Warn, never block.** Every combination here is a legitimate way to run a place, so the card says what is being given up and leaves the choice alone.

**⚠️ One relationship contradicts what this plan assumed, and the plan was wrong.** The worked example below read *"Machine workshop uses Training & badges for certification gating — without it, anyone can book any machine."* That is **not** what happens. `MachineQualificationService` has **no feature check at all** — it reads badge records directly — so switching badges off does not re-open equipment to anyone. The real consequence is harder to diagnose: **the gate keeps refusing while the pages that would explain the refusal disappear**, so a member denied a slot has no way to see what they are missing. The shipped warning says that.

The other three, each verified in the source:

| Feature on | Companion off | What is actually lost |
|---|---|---|
| `machines` | `materials` | Every equipment page loses its "accepted materials" section |
| `person_booking` | `staff` | The "réserver" button exists **only** on the directory pages, so booking still works but nothing links to it |
| `badges` | `formations` | Nothing awards credentials automatically; each is issued by hand from the admin |

**Verified** on the live container: each declared relationship produces its warning and only when it should; no warnings with everything on; none with everything off; nothing hard-fails.

**Original scope, for reference.**

**Why.** Capabilities lean on each other. Cert-gating needs badges. Training awards badges. Events want mail.

**Scope.** Capabilities declare *recommended* companions; the capability screen warns clearly when a choice will lame something ("Machine workshop uses Training & badges for certification gating — without it, anyone can book any machine"). **Warn, don't block** — an ungated community workshop is a legitimate choice.

**Verify.** Each declared relationship produces its warning; nothing hard-fails.

---

## Phase B — Portals

*Multi-tenant, one install. The operator wants this working before revisiting event polish.*

### S27 · Portal admin CRUD + branding — ✅ shipped 2026-07-30

**What landed.** `/admin/portals` (list + create + delete) and `/admin/portals/{id}` (identity, features, overrides), sidebar *Portails*. **No migration** — `PORTAL` has existed since S12 and both config tables have carried a `portalId` just as long; the whole mechanism was simply unreachable because nothing could create a portal.

**Per-portal features are tri-state, and that is the load-bearing decision.** A portal says **on**, **off**, or **nothing at all**, and nothing-at-all is the default. Plain checkboxes would have written an explicit row for every feature the first time anyone pressed Save — silently freezing that portal against every later change to the site-wide switches, while looking identical on screen. `null` means inherit, and choosing inherit *deletes* the row rather than storing a value.

**The default portal has no override editor, by construction.** It is not the important portal, it **is** the global scope: `portalId = 0` means "the default portal's value", which is what every other portal falls back to. Its page says so and links to the ordinary settings screens. `isDefault` is not editable anywhere — moving the flag would re-point every existing global row at a different front door without moving the rows, and that needs a migration, not a checkbox.

⚠️ **Only settings read *during a request* can be overridden per portal, and this is structural.** A portal is resolved from the request's hostname. Anything read outside a request resolves at the global scope whatever a portal's row says — so **mail sender identity and `public_base_url` are deliberately absent from the catalogue**: they are read by the queue worker, which handles no request and has no hostname, so a per-portal value would be stored, shown in the admin, and never once used. The screen says why rather than offering a setting the app would ignore. **Check this before adding a field to `PortalOverrides::FIELDS`.**

Two smaller traps, both encoded in `PortalRepository`:

- **A blank hostname stores as `NULL`, never `''`.** The column is UNIQUE, and MySQL treats NULLs as all distinct but empty strings as equal — so a second portal saved without a hostname would be refused as a duplicate of the first, over a field nobody touched.
- **Deleting a portal deletes its scoped config rows.** No FK by design, so no cascade — the same rule reservations live under. `id` is AUTO_INCREMENT, so orphaned rows would eventually be handed to a new portal, which would look like a haunting.

The accent colour is **validated on read as well as on save**, because it is interpolated into a `<style>` block where HTML escaping does nothing — inside CSS a `}` ends the rule. The logo setting refuses path separators for the same class of reason.

**Verified on the live container** against a throwaway portal, since deleted: 15 service-level checks (slug and hostname normalisation, both clash refusals, the NULL-hostname trap, all three feature states, inherit removing its row, blanking removing an override, the default portal refusing deletion, delete taking its 3 scoped rows with it) — and end to end, a request on the portal's hostname **404s `/machines` while the main site serves it 200**, drops it from the nav, swaps the logo and emits the accent colour, with the main site's markup unchanged.

**⬜ Left for later, deliberately.** Add-ons are not per-portal settable (they follow their parent, and the screen only offers root features). Logo is a filename in `public/images/`, not an upload — an upload path belongs with the lab-page photo convention. And a portal that overrides nothing still renders the site's own hero, which is where **S28** picks up.

---

### S28 · Per-portal home page — ✅ redirect shipped 2026-07-30, blocks still to do

**Why.** The driving case: a tenant who only wants events should have the events page as their front door.

**Scope.**
- One setting: home is **either** the block homepage (default) **or** one enabled capability's landing page, from an allow-list of route names. **Never a free-text path** — that is an open-redirect and a 500 generator.
- **302 redirect**, not an internal forward: no controller duplication, and the URL bar stays honest.
- Portal-scope the existing `HomepageSectionVisibility` blocks, which are *already* ordered and role-gated — they just aren't per-portal yet.

**✅ What landed.** `SiteFeature::$landingRoute` — each feature's canonical **public** page — plus `PortalHome`, and a `portal_home_route` field in `PortalOverrides`. A portal opens on one of its own features; blank means the ordinary block homepage. **No migration.**

- **A route name chosen from a list, never a path.** Free text would be an open redirect, and a 500 on the front page the first time somebody typed one wrong. The list comes from the enabled features, so **`app_home` is not in it** and a redirect loop cannot be configured. `person_booking` has no landing route on purpose: its page needs a login.
- ⚠️ **The list is rebuilt on every request, and that is the load-bearing part.** A portal pointing at events whose `events` feature is later switched off would otherwise 302 its own front page onto a 404 — the gate doing its job would make the site unreachable at the root. Re-checking means the worst case is the ordinary homepage coming back. **Verified live:** with events off for that portal, `/events` 404s and `/` answers 200.
- **302, and it must stay 302.** This redirect is changeable from a form. A 301 would be cached by every browser that ever saw it and would keep sending people to the old page long after the setting changed, with nothing left server-side to explain why.

**Prompted by a real portal created through the UI mid-session: overrides are now validated on save**, all-or-nothing, before anything is written. The accent colour was already validated on read — it lands in a `<style>` block where HTML escaping buys nothing — but accepting on save and refusing on read is the worst combination available: stored, shown back as though it applied, silently doing nothing. That is precisely what `#1D4ED8.` (trailing dot) did. Refusals now carry a sentence naming the format. Verified against a trailing-dot colour, a CSS injection (`#fff} body{display:none`), a `../../.env` logo, an unknown route and a bad locale — each refused, nothing written.

**⬜ Still to do — and it needs a migration, so read §7 first.** Portal-scoping the `HomepageSectionVisibility` blocks. ⚠️ **That is an ORM entity**, so adding a `portalId` column is the dangerous shape: entity mapping has no fail-safe degradation, and the code must ship *after* the migration or every page touching the table 500s. It also serves the weaker half of the case — the driving one is answered by the redirect above. Worth doing only if a portal genuinely needs its own *hero content* rather than its own front door.

---

## Phase C — Usability & consistency

### S29 · One admin layout — ✅ **complete (extraction, visual pass, chrome, panel padding)**

**✅ Done.** `public/css/admin.css` owns the admin chrome. **49 rules were duplicated across the 55 admin templates — 653 lines** — and that duplication is the mechanism behind every bug listed below: a fix landed on whichever page someone was editing and nowhere else.

- Rules moved **byte-for-byte**, and the stylesheet is linked **before** each page's inline block, so light mode renders exactly as before and a page that genuinely needs to override something still can. What remains inline is only what is page-specific.
- **Corrections are at the end of the file, not folded into the move**, so they read as deliberate: radios/checkboxes opt out of `.form-field input { width: 100% }` · `.btn-action` (12 templates), `.btn-small` (14) and `.form-field.check` were used everywhere and defined nowhere · content after `form_end()` gets the gutter `.admin-edit-panel` never had · a dark-theme block, overriding rather than variabilising so light mode stays byte-identical.
- **Class audit re-run:** 6 undefined classes across all 55 pages, down from the set above. The remaining five are single-page and cosmetic, and are named in a comment in `admin.css` so the next audit knows they were judged, not missed.
- Checked: all **62 admin paths still answer**, and every page links the stylesheet.

**✅ The visual pass is done (2026-07-31), and the agent could do it after all.** The plan said this one needed eyes on screens. `app:render` plus the fact that the stylesheets are *public* turned out to be enough: render each admin page to HTML, serve it locally against the real CSS, drive a browser over it in both themes, and measure contrast in the DOM rather than judging it by eye. Re-checked after fixing — **7 pages × 2 themes, all clean**.

What it found, none of which was visible in the markup:

- **108 hardcoded light backgrounds across 40 of the 58 admin templates.** The shared chrome was already safe — `html[data-theme="dark"] .admin-panel` outranks a page's inline `.admin-panel` on specificity, whatever the source order — but each page's *own* classes were never covered. `/admin/emails` rendered its counters at **1.06:1**. Fixed in `admin.css`, grouped by what the surface *is* (panel / recessed fill / form control / meaningful tint), so a new page picks a line instead of inventing another hex.
- **Every status colour failed in dark**: 5.0–6.7:1 on white, 2.2–3.0:1 on the dark panel. The colours carrying the meaning were the unreadable ones, on the screens whose job is to flag problems. Three pages had each invented the same four hues inline; they now share `.admin-status-*`, dark values measured at 7.1–10.7:1.
- **Two failures in *light* mode too**, found while checking dark: the solid status marks put white text on `#16a34a` (3.3:1) and `#d97706` (3.2:1).

⚠️ **Two traps worth keeping.** (1) The status classes were first called `.status-ok` / `.status-info` — names **style.css already uses** for the public `/status` page. It loads first, so the new colour won while their near-white background stayed, and the dark rule then put pale green on near-white. In a 113 KB global stylesheet, assume every generic utility name is taken; this is invisible in the markup and in either file alone. (2) **`main.js` applies the OS theme at runtime**, so a "light" test render is only light if you strip it and pin `data-theme` — the first audit pass reported light-mode results that were actually dark.

**✅ The skeletons are converged too (2026-07-31).** The plan said three; measuring found **six** — the two documented extras `.admin-rfid-*` / `.admin-user-*` (3 pages) plus `.admin-content-grid`, its own two-column shell on 6 more, which had never been named anywhere. Banner, shell and rhythm are now defined once in `admin.css` and every existing class name resolves to it, rather than rewriting 55 templates.

**Verified by measuring, not by eye**: across one page of every family, header background, header padding, sidebar column and gap all agree, and the **content column is 938px on every page** — the number that actually matters. Two families keep different margin/padding because their width and gutter come from an outer `.admin-main`; identical content width is the proof that this is structure rather than drift.

⚠️ **One deletion was wrong, and only the measurement caught it.** Six pages nest `.admin-main-content` *inside* `.admin-content-grid`, where it is the content column and not the shell. Each carried an inline `display: flex` saying so; removing those as "duplicate chrome" gave them a second, empty 260px sidebar. Now one descendant rule states the structure instead of six copies. **Treat an inline override as deliberate until measured otherwise.**

**✅ And the panel padding is unified too (2026-07-31) — S29 is done.** This was logged as a markup job needing a `.admin-panel-body`; measuring 21 panels across 14 pages showed the premise was wrong. `.admin-panel` was not two semantics — it was **three values with no rule behind them** (0, 24px, 32px). Pages whose panel opens with `.admin-panel-header` had 32px *and* 0px, and the header brings its own 20px/24px regardless, so the 32px was padding wrapped around already-padded content. Drift, not design.

The panel is now a frame everywhere and its children take the gutter — the identical idiom `.admin-edit-panel` already used, needing no wrapper element. `.admin-panel-header` and the table wrappers are exempt: they pad themselves, or want to bleed to the edge. It was checked first that every other child had **zero** horizontal padding of its own, so nothing doubles up.

Fallout, both found by measuring: nine pages carried a hand-written `<div style="padding: 0 24px">` compensating for an unpadded panel — redundant now, removed; and two bordered cards (`.check`, `.reader-help`) had their *internal* 16px overridden to 24px by the gutter, restored at page level where a card's own metrics belong.

**Verified before and after:** all 21 panels report 0px, every non-exempt child reports the 24px gutter, tables still bleed full width, both themes correct.

⚠️ **The reusable lesson from all three S29 passes: read the templates to find candidates, but decide from measurement.** Reading said three skeletons (there were six), said `.admin-panel` had two semantics (it had three arbitrary values), and made six deliberate `display: flex` overrides look like duplicate chrome. Every one of those was caught by printing computed styles, and none of them by looking at the source.

**Original scope, for reference.**

**Why.** Three incompatible admin skeletons exist, each with copy-pasted inline CSS across ~53 templates. Direct cause of several visible bugs: panels with no padding, buttons styled nowhere, radios inflated to full width, 37 pages rendering the sidebar as a bare link list.

**Scope.** One shared admin base template owning chrome, panel padding, form controls, buttons and flash styling. Migrate every admin page to it and **delete the per-page copies**. Re-run the class audit (cross-reference every `class="…"` against defined selectors).

**Verify.** Visual pass over every admin page in **both themes** — the one session that genuinely needs eyes on screens, not status codes.

---

### S30 · Admin actions where the content is — ✅ shipped 2026-07-31

**What landed.** One macro, `site/_admin_inline.html.twig`, and one affordance, on six detail pages: events (edit + registrations), equipment, spaces, training (edit + content), lab pages, badges. Global configuration stays in the panel; this is only ever *this record, here*. No migration.

**The button is a shortcut, never the permission.** Every target sits behind `/admin`, gated by `access_control` — so what makes it work is the firewall on each request, not the chip's visibility. **Both halves were verified rather than assumed:** signed in, the six pages render 8 chips; anonymous, they render **zero**; and every real href scraped out of the rendered pages returns **302** when fetched without a session.

⚠️ **`role` must mirror the route's actual access rule, and this app has no role hierarchy — `ROLE_ADMIN` does not imply `ROLE_STAFF`.** The registrations chip was gated `ROLE_STAFF` on the reasoning that it is a staff-desk job; its route lives under `/admin`, so the effect was to hide it from the only people who can use it. Not a hole — the firewall was never in question — but the wrong kind of wrong, and it surfaced only because the chip count came back 1 instead of 2. Check the path prefix and the matching `access_control` line.

⚠️ **The chip carries its own backdrop.** It sits beside a title, and a title can be overlaid on a photo hero (events, equipment). Transparent looked right on a white panel and was invisible on the event page — visible only in a screenshot, not in any markup check.

**⬜ Not done.** List pages have no "add" chip yet (the panel's own new-buttons already cover it), and there is no inline check-in — the ticket scanner is its own `/staff` screen and moving it inline needs the staff-vs-admin split thought through, given the missing role hierarchy above.

---

### S31 · Neutral vocabulary + organisation identity — ✅ shipped 2026-07-31

**What landed.** Two settings on `/admin/settings` — `org_name` (the school, association or company) and `venue_label` (what the place is called) — kept **separate on purpose**, because "the ENSEA FabLab" is two different nouns and a deployment can need to change one without the other. No migration.

**158 occurrences across five catalogs** became `%venue%` (133) and `%org%` (25), plus page titles and kiosk headers via `venue_label()` / `org_name()` in Twig.

**`VocabularyTranslator` decorates the translator, so no call site passes anything.** Doing it per-`|trans` would have meant editing every call in the app and, worse, would have failed *silently* the first time someone forgot — printing a raw `%venue%` onto a public page. Decoration makes it structural: the parameters are always present for templates, controllers, mail and the console alike, and Symfony ignores parameters a string has no placeholder for, so the other ~800 keys are untouched.

⚠️ **Resolution is lazy and fail-safe, and it must stay that way.** This service sits in front of *all* text, so it is reachable during cache warmup, from the console and from the mail worker — none of which are guaranteed a database. Reading settings in the constructor would make a config store a hard dependency of rendering any string at all. On failure it returns the old hardcoded words.

**Defaults are this install's existing wording, not neutral words.** The point is to make the vocabulary editable, not to rename somebody's site out from under them — an operator who never opens the screen sees no change.

**Verified:** with defaults, "FabLab" appears exactly where it did and **no page leaks a raw placeholder**; setting the pair to "Makerspace Lyon" / "atelier" changes the wording across the public site; clearing them restores it.

⚠️ **Concatenating into a `??` fallback needs parentheses.** `description ?? 'text' ~ venue_label()` can bind as `(description ?? 'text') ~ venue_label()`, appending boilerplate to a record that *has* a description. Twig 3.15 deprecates the ambiguity; here it was a bug in waiting.

**Known remaining:** five `@ensea.fr` email placeholders. Those are a domain, not a name — inventing a setting to hold one example string is worse than leaving it.

---

## Phase H — Hardening, correctness and coverage

*Added 2026-07-31. **This phase runs before Phase D.** The letter is out of alphabetical order on purpose: A–E were already assigned and several of those sessions have shipped, so renumbering them would invalidate every commit message and cross-reference that cites them. Read the order from this sentence, not from the letter.*

**Why now.** Most of this is not new work — it is a codebase audit from 2026-07-10 that was parked before anything was implemented, plus the verification debts this project has been honest about accumulating. It was re-checked against the live site on 2026-07-31 and **the critical findings are still true today**, so the parking was the only thing keeping them open.

**Sizing note.** These are deliberately small next to a Phase A session. That is the point: each one should be startable and finishable without a decision from anybody, because the reason they went unfixed for three weeks was never difficulty.

### Before anything else — three chores, not sessions

1. **Push.** Twenty commits sit only on this Mac and CT 210's filesystem. The branch on GitHub is the only second copy.
2. **Clear two probe rows** from `MISSING_PAGE` (`/probe-raw`, `/probe-service`), left by an S37 verification harness — the *Vider le journal* button on `/admin/missing-pages`.
3. **Re-save the `events` portal.** Its accent colour is stored as `#1D4ED8.` with a trailing dot, so no colour is emitted. Since S28 the form refuses that instead of accepting it silently.

---

### S38 · Stop publishing badge UIDs and names — ⚠️ do this first

**Why.** `GET https://fabos.dstei.fr/api/leaderboard` returns, unauthenticated and from the public internet, a list of real people paired with their **physical badge UID**:

```
{"rank":1,"username":"yanis","displayName":"Yanis Test","rfid":"8C52B359"}
```

That UID is the credential the door and machine readers trust. Publishing it next to a name is a badge-cloning kit. Two neighbours are the same class: `/api/access-rfid-logs` exposes badge UIDs plus who used which machine when, and `/api/reservations` exposes real names with what they booked and when.

**Scope.** Drop `rfid` from the leaderboard payload outright — nothing needs it client-side. Decide per endpoint whether the fix is *narrowing the serialiser* or *requiring authentication*; prefer narrowing, because an endpoint that stops returning a field cannot leak it again by a permissions mistake later.

⚠️ **Watch — find the consumers before changing the shape.** These are public JSON endpoints on a live site; a kiosk screen, a script or somebody's dashboard may be reading them. Removing a field is a breaking change for whoever that is. Grep the repo, then ask the operator, before deciding between narrowing and gating.

⚠️ **A field-name grep is not enough to check this.** The entity property is `identifiantRfid` and the JSON key is `rfid`; grepping the former returns zero hits and reads like "already fixed". **Verify by fetching the endpoint and reading the payload.**

**Verify.** Anonymous fetch of each endpoint contains no badge UID and no personal data beyond what a public leaderboard needs; the signed-in paths still return what their pages require.

---

### S39 · RFID device auth that fails closed — ⚠️ sequencing matters more than the code

**Why.** `RfidMachineController::rejectUnauthorizedDevice()` reads `FABOS_RFID_API_TOKEN` and, when it is empty, **returns `null` — which means "allowed"**:

```php
if ($expectedToken === '') {
    return null;
}
```

The variable is unset in both `.env` and `.env.local` on CT 210, so **the RFID device endpoints currently have no authentication at all**. They are not read-only: `POST /api/rfid/machines/{token}/authorization` decides whether a badge may use a machine, and the work-session routes start and stop machine time.

**Scope.** Invert the default so an unset token refuses instead of allows, provision a real token, and write down how a reader gets one.

⚠️ **This is the fail-open/fail-closed rule from §3 of the handover, applied to a place that got it backwards.** Config stores fail *open* so a config problem cannot take the lab offline; a credential check must fail *closed*, because the failure mode of guessing is that anybody is a device.

⚠️ **Deploy order, and it is the migration hazard in a different costume: provision the readers FIRST, flip the code SECOND.** Landing fail-closed before every reader carries the token locks every door and machine in the building at once. If the readers cannot yet be provisioned, ship the token check as *warn-and-log* first and flip it after — but do not leave it warning forever, which is where this finding came from.

**Verify.** With the token set: correct header → 200, wrong or missing → 401. **Then the operator badges in on a real reader** — this is not verifiable from a shell.

---

### S40 · The two remaining ways in

**Why.** `security.yaml` has no `login_throttling`, so `/login` accepts unlimited guesses at whatever rate an attacker can manage. And `LOCAL_ADMIN_BYPASS=1` is still sitting in `.env.local` — inert today only because the site runs `prod` and the class it belonged to is deleted, but it is a live-looking switch that means nothing, which is exactly the kind of thing someone re-enables.

**Scope.** Add `login_throttling` to the main firewall. Delete the `LOCAL_ADMIN_BYPASS` line from `.env.local`.

**Verify.** Repeated failed logins start being refused; a correct login still works first time. `app:render` is unaffected — it never used that flag.

---

### S41 · The queries that read whole tables

**Why.** Several hot paths load a full table into PHP and aggregate by hand instead of asking SQL: `/formations` (`SiteController:1860`), `/search` and `/api/search` (the same logic written twice, `SiteController:1466-1515` + `ApiController:337-389`), the leaderboard's all-time tab (`LogUtilisationRepository::computePresenceMinutesByUser`), and admin usage-logs, which has no pagination at all. `LOG_UTILISATION` — the fastest-growing table in the app — **has no index on `dateDebut`** despite being filtered and sorted on it everywhere.

**Why now rather than later.** It currently holds 2 rows. Every one of these is cheap to fix today and a production incident to fix at 500 000.

⚠️ **The index is a migration, so it is an expand: migration first, code second** (§7). Adding an index is safe to run ahead of the code that benefits from it.

**Scope.** Push the aggregation into SQL, paginate usage-logs, add the index, and collapse the duplicated search logic into one place.

**Verify.** Same output before and after — snapshot the rendered lists and diff them, exactly as S24 did for the nav. Not "it looks right".

---

### S42 · A test suite that exists

**Why.** `tests/` contains one file: `bootstrap.php`. There is **no automated coverage of anything**, on a codebase that now carries a booking engine with three independent permission layers, a polymorphic reservation model, portal scoping with tri-state inheritance, and fail-safe repositories whose whole contract is what they do when the database is unavailable. Every regression this project has caught, it caught by hand.

**Scope.** Start where tests are cheapest and most valuable — pure logic with no database:

- quota ordering in `BookingPolicyService` (coarsest-constraint-first is a *deliberate* order and nothing enforces it)
- `ReservableRef` / `ReservableResolver`
- `PortalOverrides::save()` validation and the all-or-nothing rule
- `SiteFeatureService::stateForScope()` tri-state, and add-ons forced off by parent
- `FirstRun::isFresh()` — including the "flag absent but install has content" case, which is the one path that could not be exercised against production
- `MissingPageSubscriber`'s filters
- `VocabularyTranslator` fallback when settings are unavailable

**Watch.** Do not chase a coverage number. The list above is chosen because each item is a rule someone could plausibly "simplify" away, and the test is what says no.

**Verify.** `vendor/bin/phpunit` green, and it runs without a database.

---

### S43 · Delete what is dead, and clear the deprecations

**Why.** Dead code is read by the next person as if it matters. `src/Entity/TlseUser.php` and `src/Entity/UserBadge.php` are both orphaned — `UtilisateurBadge` is the real one — and both are still present. Two deprecation warnings appear on every cache rebuild: `UtilisateurBadge::$utilisateur` declares `nullable` on a join column that is part of the identifier (a no-op that becomes an error in Doctrine 4), and every DDL migration warns about committing an already-committed transaction (MariaDB commits DDL implicitly; the fix is `transactional: false`).

⚠️ **Watch — `resources/database/` is stale but not unused.** It is still read by `app:fabos:seed` and `ImportGuidedSectionsCommand`, and it has drifted from `migrations/sql/`. Reconcile or retire those commands deliberately; do not delete the directory because it *looks* obsolete.

**Verify.** App boots, `cache:clear` is silent, and whatever the seed commands are supposed to do, they still do or are gone on purpose.

---

### S44 · Pay off the verification debts

**Why.** These are the things this project has been honest about not knowing, and they have outlived several sessions.

- ⚠️ **The booking success path has still never been verified end to end.** Every refusal branch is tested; the happy path is assumed. It needs a login, so it needs the operator — **this is the oldest open item in the whole plan.**
- **50 of the 58 admin pages have never been looked at** in either theme. S29 audited 8. The harness exists and is written down in `PROJECT_STATE.md` §9.
- **Public pages have never been contrast-audited at all** — S29 only ever pointed the tool at `/admin`.

**Scope.** Run the existing harness over the remaining admin pages and over the public site; have the operator click one real booking.

**Verify.** It is the verification. Record what was checked, so the next session inherits a smaller list rather than the same one.

---

### Also outstanding, tracked in their own entries

Deliberately not duplicated here — each is written up where it belongs, with its reasoning:

- **S27** — add-ons are not per-portal settable; the portal logo is a filename rather than an upload.
- **S28** — portal-scoped homepage blocks (an ORM entity, so migration-first).
- **S30** — no "add" chips on list pages; no inline check-in (needs the staff/admin split settled, given there is no role hierarchy).
- **S31** — five `@ensea.fr` email placeholders, left because a domain is not a name.
- **From the 2026-07-10 audit, smaller items:** CSRF on `MachineFavoriteController` favourite add/remove; `.env` committed to git (placeholders today, a bad habit to keep); the "who voted" panel not wired to the mini-cards or the ranking page, and the open privacy question of whether individual scores should be public at all.

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

### S37 · 404s that help, and a record of what people are looking for — ✅ shipped 2026-07-30

*Logged 2026-07-28 at the operator's request, after hitting a disabled page. Migration `Version20260804100000` applied by the operator the same day.*

**✅ The cause, first: the live install now runs `prod`.** It had been on `APP_ENV=dev`, so "it throws an error" was literally Symfony's development exception page, in English, with `No route found for "GET http://fabos.dstei.fr/…"` sitting in an HTML comment — and the profiler answering to anyone. That was the whole of the reported bug; everything below is what makes the page actually good. See *What to do next* for the change and for the `LOCAL_ADMIN_BYPASS` consequence it dragged along.

**✅ A real 404 page.** The templates already existed and had never once been *seen*, because dev mode meant they never rendered. Two things were wrong with them and both mattered:

- **They were hardcoded French, and hardcoded to features.** The way out of the 404 was "Voir les machines" / "Voir les formations" — so on an install with equipment and training switched off, **every escape route from the 404 was itself a 404.** They now come from `NavBuilder::safeDestinations()`, which reads the footer: the suggestions inherit the menu's own rule that nothing switched off is ever offered. Wrapped in a `try/catch` down to the home link alone, because an error page that throws while explaining an error has nowhere left to go.
- **The wording said "something broke".** A disabled feature 404s *by design* — that is the gating model, not a fault — so the page now reads **"this page isn't part of this site"**, translated across all five catalogs (`error.*`). It deliberately stops there: *which* feature is off goes to the log for the admin, not onto a page anyone can probe.

**✅ The misses are logged** — `MISSING_PAGE`, raw DBAL, fail-safe, aggregated one row per distinct path with a counter, plus first/last seen and the last referrer. Screen at `/admin/missing-pages` (sidebar *Pages introuvables*), which sorts most-asked-for first and tags every row with **why**: a switched-off feature, a broken internal link, or a wrong URL out in the world. That distinction is the entire point — the first is not a bug, it is demand, and it is answered on the feature screen.

**Most of the work was deciding what *not* to record**, because a raw 404 log is bots. Three filters, each killing a whole class of noise:

1. **No route matched, or the feature gate refused.** A 404 from a controller that *did* match its route — reservation 4211 does not exist — is a missing *row*, not a missing page, and it would bury everything else.
2. **A referrer, or a known reason.** Probes for `/wp-admin` arrive without one; a person following a broken link brings the page they came from. The deliberate exception is a switched-off feature, where an old bookmark is exactly how somebody arrives and nothing links there any more — so `FeatureAccessSubscriber` now writes the feature key onto the request before throwing, and the reason survives into the log.
3. **Pages, not assets.** A dead stylesheet is a broken asset; mixing those in makes a list of missing *pages* unreadable.

**No scheduled prune, on purpose.** Aggregation already bounds the table by the number of *distinct* wrong URLs rather than by traffic, so size is not the problem relevance is: the screen has a "forget what has been quiet for 90 days" button and a clear-all, both POSTs behind a CSRF token.

**⚠️ One bug, found only because the table was there to check: the subscriber never ran.** It was registered at priority **−256**, deliberately below Symfony's `ErrorListener` (−128) so as not to interfere with the response being built. But `ExceptionEvent` extends `RequestEvent`, and `RequestEvent::setResponse()` calls **`stopPropagation()`** — so attaching the error page ends the event, and everything below −128 is skipped. `debug:event-dispatcher` listed the listener as present and correct the whole time; the only symptom was an empty table. Now −100. **The general rule: a `kernel.exception` listener that merely observes still has to run before the error page exists.**

**Verified on the live container**, migration applied: public 404 renders the real page with working links and no debug markers · `/_profiler` 404s · every public page still 200s · a miss with an internal referrer is recorded and its counter increments on a second hit · an external referrer is tagged as such · **a bot probe with no referrer is dropped** · a dead asset is dropped · a 404 from a controller whose route *did* match (machine 99999999) is dropped, because that is a missing row, not a missing page. Earlier, before the migration, the screen also rendered its empty state with the table absent — the fail-safe doing its job.

*(Two rows, `/probe-raw` and `/probe-service`, are left over from the throwaway command used to prove the store worked. Clear them with the screen's own **Vider le journal** button, which is also the only untested control on the page.)*

---

## Later, unchanged in priority

- **Activity feed as a shared contract** — every module publishes events with a severity; one admin feed; optional public kiosk (privacy-filtered, no incident data).
- **Control-box / IoT** — MQTT, device drivers, relay/interlock, power monitoring → real run-hours, fail-safe offline cache. Was blocked on the permission trio, which is done, so this is now unblocked.
- **Incident tracker** (staff-only, GDPR-sensitive), analytics/reports, public credentials page, LDAP login.
- **Open-source readiness** — Docker one-command deploy, community translation, GDPR export/delete, REST API + webhooks, accessibility pass, backup/restore.
- **Known small debts** — finish the half-wired mobile nav; add a machine-delete route that cancels its bookings (none exists, and there is no cascade).

---

## How these sessions are sized

Each `S##` is **one self-contained, deployable session**: build, deploy to the live container, verify, commit. If a session cannot be verified end-to-end it is too big and should be split. Every session ends with the app running.
