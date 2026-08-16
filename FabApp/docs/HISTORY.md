# FabOS roadmap — from fablab tool to modular platform

**Written:** 2026-07-24 · **Last updated:** 2026-07-31 · **Status of the app:** S1–S23 shipped and live, then **capabilities and modules were collapsed into site features** (2026-07-28). **Phase A is complete (S21–S26).** S25's health panel and S29's stylesheet extraction are in too. **S37, S27, S28 shipped 2026-07-30; S29, S30, S45, S46, S50 and S54 2026-07-31; S55 2026-08-01; S47, S48, S49, S51, S52 partly and S53 barely started 2026-08-01.** ✅ **The decision that blocked the rest of Phase U is settled (2026-08-01): AssetMapper is on, Stimulus runs, Turbo stays off.** `importmap('app')` is in both shells and 8 standalone pages; `window.Turbo` is undefined in prod and navigation is unchanged. See S51. ⚠️ **S48 found live public exposure of machine device tokens — fixed on the page side, but `FABOS_RFID_API_TOKEN` is still unset and the device check still fails open; see S48.** The live site runs `prod`, portals are reachable and brandable, and admin dark mode has been looked at. ⚠️ **Next: Phase H (S38–S44) — hardening; S38 first, the public API is publishing badge UIDs today.** **Phase U (S45–S57) is under way: S45 (design tokens), S46 (one public layout — all 37 header-and-footer pages now extend `site/base_public.html.twig`) and S54 (dead affordances, −748 lines) all shipped 2026-07-31.** S45 and S46 were the part that had to land before the LMS; S54 spun out **S56** (password reset) and **S57** (account deletion), two features the UI advertised with no backend behind them. See *What to do next*.

---

## What to do next — advice as of 2026-07-31

**Before anything else, two things that are not sessions.**

1. **Keep pushing.** The branch was pushed on 2026-07-28 and is current through S25's health panel; anything after that is again Mac-only until you push. ⚠️ CT 210's own checkout is still `main` and every deploy there is hand-placed, so the branch on GitHub is the only second copy.
2. **Click through one real booking.** The booking *success* path has never been verified by the agent — it needs a login, and the firewall stops anonymous POSTs before the controller. Every refusal branch is tested; the happy path is assumed. S22 put a new gate at the top of `ReservationService::book()`, so this is the moment to confirm it in a browser.

**✅ S25 is finished (2026-07-31).** The health panel and the wizard both shipped; **sample data was dropped at the operator's request** and is not pending work — see the S25 entry.

⚠️⚠️ **Phase H was added 2026-07-31 and it goes before Phase D.** Most of it is a codebase audit from 2026-07-10 that was parked before anything was implemented; it was re-checked against the live site and **the critical findings are still true**. The first one is not theoretical: `https://fabos.dstei.fr/api/leaderboard` currently returns real names paired with their **physical badge UID**, unauthenticated, from the internet — and the RFID device endpoints have no authentication at all, because the token check returns "allowed" when the token is unset. **Start at S38.**

**✅ Phase U's two front-loaded sessions are done (2026-07-31).** **S45** put one token layer in `style.css` with `/admin/design` as its reference screen, and **S46** gave the public side a layout: `site/base_public.html.twig`, extended by **all 37 pages that carry the header and footer**. Both went before Phase D on purpose — the LMS adds around ten new screens, and building them against the old chrome would have meant building them twice. **The rest of Phase U (S47–S53, S55) can follow the LMS quite happily.** **S54 also shipped 2026-07-31** — it was independent of the layout work and is the reason S56/S57 now exist.

⚠️ **Adding a public page now means extending `base_public.html.twig`**, not copying another page's `<head>`. If you find yourself writing `<!DOCTYPE html>` in `templates/site/`, you are re-opening the problem S46 closed — the exceptions are the eight kiosk/ticket/scan templates, which carry no header or footer at all.

*A finding from that phase's audit worth knowing even if you never run it:* the admin sidebar had **29 entries and gated none of them by feature** — **closed by S50 on 2026-07-31**. *(The same audit's "19 bientôt disponible buttons" finding was **fixed by S54 on 2026-07-31** — and it was an undercount: 40 dead controls across 16 templates, because a third of them never used the phrase. Disabled controls are exempt from contrast and markup audits alike, so grep the attribute.)*

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

#### What shipped 2026-08-01 — the API side

**The scope was more than double what this section listed.** Fetching every endpoint anonymously and reading the payloads — the check this section demands, and the only one that works — found **8 leaking endpoints, not 3**:

| endpoint | leaked | fix |
|---|---|---|
| `/api/leaderboard` | `rfid` + real name | **narrowed** — field dropped, stays public |
| `/api/access-rfid-logs` | `badgeUid` + name + machine + time | gated `ROLE_ADMIN` |
| `/api/machines/{id}/historique` | `badgeUid` + names, enumerable | gated `ROLE_ADMIN` |
| `/api/progressions` | name + training **score** | gated `ROLE_ADMIN` |
| `/api/reservations` | name + what + when + `motif` | gated `ROLE_ADMIN` |
| `/api/reservations/{id}` | same, enumerable by id | gated `ROLE_ADMIN` |
| `/api/calendar` | name + `motif`, every active booking | **narrowed** to occupancy |
| `/api/machines/{id}/reservations` | name + `motif` | **narrowed** to occupancy |

⚠️ **Three of the eight were in no list anywhere** — `machines/{id}/historique`, `progressions` and `calendar`. They were found by the sweep, not by reading the finding.

⚠️ **The grep trap is worse than recorded.** The property is `identifiantRfid`, the leaderboard key is `rfid`, and the RFID-log key is **`badgeUid`**. No single grep covers all three; only fetching the payloads does.

**`/api-docs` decided narrow-vs-gate more cleanly than guesswork could.** `/api/leaderboard` is *documented public* — so the contract was right and only the field was wrong, which is the textbook narrowing case. `/api/reservations` was *already documented* under "Réservations connectées" and simply never enforced, so gating it **restored** the documented contract instead of breaking one. The other three were undocumented, so nothing public could depend on them.

⚠️ **The shared serialisers could not be narrowed in place.** `reservationToArray()`, `rfidLogToArray()` and `progressionToArray()` are used by the `ROLE_ADMIN` endpoint `/api/users/{id}`, where the name and the motif are exactly what the caller is entitled to. Stripping them would have silently emptied the admin views. Hence a **second** serialiser, `reservationToOccupancyArray()` — and deliberately a separate function rather than an `$includeIdentity = false` flag, because a flag puts the safe outcome one forgotten argument away.

**Verified:** all 18 GET endpoints fetched anonymously; every one either returns no identity field at all or redirects to login. `/api/leaderboard` still ranks, `/api/calendar` and `/api/machines/{id}/reservations` still return their slots.

#### What shipped 2026-08-01 — the page side, as an operator setting

**`booking_identity_roles`** in `SITE_SETTING` (no migration — the table predates this), ticked per role in Site settings from the **`ROLE` table the operator already edits**. This install offers `ROLE_ADMIN`, `ROLE_STAFF`, `ROLE_TRAINER`, `ROLE_USER`; the default is **staff + admin**, deliberately the restrictive end, so an install that never opens the screen does not inherit the leak.

`BookingIdentityPolicy` owns the decision so the two calendar templates cannot drift, and so the next surface that shows bookings asks rather than invents.

⚠️ **Each configured role is tested separately and OR'd** — this app has no role hierarchy, so testing only the "highest" one would hide names from admins on an install that ticked staff alone.

⚠️ **`user_id` is no longer sent to the browser at all.** The calendars badge your own booking as "Ma réservation", which needed the viewer's id client-side; shipping every booking's id would let anyone group one person's slots and follow their week — most of what a name gives you. A server-computed `mine` flag replaced it, so **hiding identities never hides your own booking from you**.

⚠️ **`method_exists(null, 'getId')` is a TypeError in PHP 8** — it took all three calendars to 500 on first deploy, for the anonymous visitor the change exists to serve. Fixed by typing against the entity. The lesson is the older one: `curl` the page, do not trust that it compiled.

**Verified:** all three calendars fetched anonymously emit `user: null`, `motif: null`, no `user_id`, and still carry every slot's start and end; `app:render /admin/settings` shows the four role checkboxes with staff + admin pre-ticked.

#### S38c — the surfaces the setting did not reach (2026-08-01)

Closing the calendars was not the end of it. A sweep of **every public page** for real badge UIDs and real names — rather than of the pages just touched — found two more:

- 🔴 **`/machines/{id}/historique` was published in full to anonymous visitors**: badge UIDs in clear (`8C52B359`, `39F1C387`, …), the name attached to each scan, each work session and each booking. The JSON endpoint of the same name had been gated an hour earlier; **the page rendering the same rows was missed.** Names now follow `BookingIdentityPolicy`; the badge UID column is `ROLE_ADMIN` only, because a UID is a credential and no display need justifies it.
- 🔴 **`/api-docs` hardcoded a real badge UID as its example.** `8C52B359` is an actual row in the database — the public documentation was handing out a working credential labelled "example". Replaced with `A1B2C3D4` and a comment telling the next author never to paste a live one back in.

⚠️ **The lesson is the sweep, not the fix.** Both were found by grepping every public page for known-real UIDs and names, not by reasoning about which pages "should" have them. This is the second time the API was closed while a page kept publishing the same rows.

**Left deliberately public, because names are the point:** `/equipe` (a team directory), `/creations` (a gallery of members' own published work), `/leaderboard` (documented public, and a leaderboard without names is a list of numbers).

**Decided the same day — the history pages are scoped to the reader, not merely masked.** Masking the columns still told an anonymous visitor *how many* people used a machine and exactly when, which is most of the information. So the **rows** are filtered:

| viewer | rows |
|---|---|
| anonymous | none, with an invitation to sign in |
| signed-in member | their own scans, sessions and bookings, nobody else's |
| staff / admin | everything |

⚠️ **The entitlement reuses `BookingIdentityPolicy` rather than a hardcoded `ROLE_STAFF`.** An operator who ticks *formateur* for the calendars means it here too, and two rules answering the same question drift apart.

⚠️ **The badge UID column stays `ROLE_ADMIN` even in "own" scope.** A member's own UID is not a leak, but a credential printed on a page has no display need behind it.

⚠️ **The scope has to be *said*, not just applied.** The stat cards count the visible rows, so a member shown "3 passages" with no explanation reads it as the machine's whole history — a wrong number stated confidently, which is worse than the missing rows. Hence `.fab-scope-notice`, a real component in `components.css` (⚠️ `.notice` looked reusable but is defined only *inline* in `admin-emails.html.twig`, plus an orphaned dark-theme rule in `style.css` — using it here would have styled nothing). Cache-buster bumped on `components.css`.

**Verified with `app:render --as`**, all three scopes on the same page: anonymous 0/0/0; Sofia 36/0/6 with her own name and **zero** occurrences of Yanis, Cedric or Alvaro; admin 129/2/14 with UIDs.

**Decided: `/kiosk/*` stays public.** It publishes who is in the building right now, unauthenticated — accepted for now, with **restricting the kiosk routes as a group** left as a later job rather than a per-page patch.

**Superseded — the original text of this finding:**

🔴 **Not fixed, and it needs a decision: the calendar *page* leaks the same data by another route.** `/calendrier`, `/calendar` and `/machines/{id}/calendrier` all return **200 anonymously** and server-render every booking's real name and free-text `motif` into the page's inline JS (`calendrier.html.twig:204`). Closing the API while the page publishes the same rows is theatre. It was left open on purpose: what the calendar should show is a **product decision, not a security one** — some labs show who booked so you can go and ask them, some do not — and it changes what signed-in members see, not just anonymous visitors. **S38 is not done until that is answered.**

---

### S38b · The timezone audit — what the two conventions actually are (2026-08-01)

The blocked step from the reverted `date_default_timezone_set()` attempt. **The conventions are not arbitrary; they are two coherent rules, and only one of them is broken.**

**Convention A — machine timestamps. Stored UTC wall-clock, displayed WITHOUT conversion. 🔴 This is the bug: shown 2 h early.**
Written by `new \DateTimeImmutable()` with PHP's default zone (UTC on the box), or by MariaDB's `CURRENT_TIMESTAMP` default — **MariaDB is on UTC too** (`NOW()` == `UTC_TIMESTAMP()`, checked), so the 21 columns carrying that default agree with the PHP ones.
Fields: `AccessRfidLog.createdAt`, `LogUtilisation.dateDebut/dateFin` (`WorkSessionService`), `Progression.dateDebut/dateEnd`, every `createdAt`/`created`/`updated`, `derniereConnexion`, `lastSeenAt`, `lastAuthorizationTime`, `queuedAt`.

**Convention B — human-entered wall-clock. Stored in the lab's wall-clock, displayed WITHOUT conversion. ✅ Correct today.**
The rule is documented in `StaffController` around line 126: parse the `datetime-local` string in the lab zone so that, formatted back to a naive string for storage, it stays the time the human typed.
Fields: `Reservation.dateDebut/dateFin` (`SiteController` ~1572, `ApiController` ~720, `PersonBookingController`), access-pass validity, `OpeningHours.openTime/closeTime`, and `Event.dateDebut/dateFin` — the last via `EventAdminType`, which sets no `model_timezone`/`view_timezone`, so the submitted wall-clock round-trips verbatim.

⚠️ **The fix is therefore the opposite of "pin `Europe/Paris` everywhere".** Pinning a Convention B field **double-shifts it**: opening hours render `08:00` correctly today and a pin would make them `10:00`. The spawned session that was mass-pinning 142 call sites was stopped for exactly this reason — it would have broken the fields that were already right while fixing the ones that were wrong.

⚠️ **And it is why no global default can work.** A global default moves the read *and* the hydration (they cancel, so nothing appears to change) *and* the Symfony form model timezone — so newly created events would start being stored under a different convention from the existing ones. A silent split, on top of a fix that fixes nothing.

**What the real fix looks like:** convert on display **for Convention A only**, via the operator's configured zone. The 142 unpinned calls split roughly by field — `createdAt` (22), `dateEnd`, `checkedInAt`, `derniereConnexion`, `lastAuthorizationTime`, `updated` and friends are A; the 48 `dateDebut` and 20 `dateFin` are **split by entity**, `Reservation`/`Event` being B and `LogUtilisation`/`Progression` being A. ⚠️ **Classify by entity, never by field name** — `dateDebut` alone belongs to both conventions.

#### What shipped 2026-08-01 — the unification

**The zone is an operator setting.** `timezone` in `SITE_SETTING` (no migration), asked as part of wizard step 3 ("Où est-ce, dans quel fuseau, et dans quelle langue ?") and editable in Site settings — 418 zones, validated against `DateTimeZone::listIdentifiers()`, rejected with a flash rather than silently ignored. The settings screen shows *"Il est 23:31 chez vous d'après ce réglage"* so the operator can check it against their own watch instead of trusting a zone name.

**`|lab_date()` — a named filter rather than a timezone argument.** `LabTimeExtension` adds one filter and one function (`lab_timezone()`). The name is the point: **the call site now states which convention it belongs to**, instead of leaving the next reader to trace the value back to the code that wrote it. `|lab_date()` = machine timestamp, convert. `|date()` = human-entered wall-clock, already lab time.

**The pass:** 48 machine-timestamp calls across 23 templates converted to `|lab_date()`, plus the 11 hardcoded `|date(…, 'Europe/Paris')` pins — **zero `Europe/Paris` left in `templates/`**. PHP: 15 hardcoded zones replaced by the setting across `AccessPass`, `AccessPassRepository`, `PersonAvailabilityService`, `NextFreeSlotService`, `ReservationService`, `IcalFeedService` and four controllers (a `labZone(SiteSettingService)` helper).

⚠️ **Date-only fields were deliberately left on `|date()`** even when they are convention A — `dueDate`, `expectedReturnDate`, `dateObtention`. A due date is a calendar day, not an instant; converting it would flip the day around midnight on any zone west of UTC.

⚠️ **The iCal feed had a trap that only appeared once the zone became configurable.** It emitted wall-clock under `TZID` with a hand-written `VTIMEZONE` carrying CET/CEST offsets and the EU daylight-saving rules. Swapping the TZID alone would have advertised, say, `America/New_York` against Paris's offsets. It now emits **UTC** — no VTIMEZONE, unambiguous for every client, and immune to a country changing its DST rules.

⚠️ **`clock_controller.js` no longer hardcodes the zone either** — it reads `data-clock-timezone-value`, rendered from the setting. Empty falls back to the browser's own zone: on a wall display, the machine's local time beats the server's UTC.

**Verified by measurement, before and after.** The witness was `/kiosk/entries`, which shows a machine timestamp next to a clock: entry times read **08:41 before and 10:41 after**, while the clock, opening hours (`08:00`) and booking slots (`10:00`) did not move — convention B untouched, which was the whole risk of the pass. All 18 public pages 200; `app:render` confirms the field on both admin screens.

**One 500 in the middle of it:** `resolveLeaderboardPeriod()` gained a parameter its caller did not pass, so `/leaderboard` threw `ArgumentCountError` until the call site caught up. Found by sweeping every page rather than the ones that were touched — the pass changed private helper signatures, and their callers are not in the diff you are looking at.

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

## Phase U — Design system and UI upgrade

*Added 2026-07-31. **Sequencing: S45 and S46 landed 2026-07-31; the rest of Phase U can follow Phase D.** The LMS adds roughly ten new screens, and building them against today's chrome means building them twice. The letter is out of alphabetical order for the same reason as Phase H — read the order from this sentence.*

🔴 **A second block, S58–S66, was added 2026-08-01 from a read of 73 Fabman screenshots — see *Phase U (continued) — read against Fabman* at the end of this phase. It is PROPOSED ONLY and nothing in it is authorised to be built.** It also carries the **feature-gap table** (packages, billing, notes, change log, holidays, overrides…), each row with a recommendation, none of them decided. ⚠️ **Its argument for going before Phase D is the same one S45/S46 won**: S58 and S59 fix the page shapes, and the LMS is about to add ten more pages to whichever pile exists at the time.

**The goal in one sentence.** The app should be simple, modern and pleasant enough that a member books a machine without being taught how, and an operator recognises their own place in it.

### What is already true, so nobody redoes it

S29 gave the **admin** one chrome, one panel model and a checked dark theme. S30 put contextual "edit this" actions on the public detail pages. S31 made the vocabulary the operator's. Portals already carry a **per-portal accent colour** (`portal_primary_color`) and a logo. None of that needs redoing — this phase builds on it.

### What was measured on 2026-07-31, because the numbers set the agenda

| | |
|---|---|
| CSS shipped | **460 KB across 14 stylesheets** — `style.css` alone is 113 KB |
| …of which calendar | **135 KB in four overlapping files**: `calendar.css`, `calendar-fix.css`, `calendar-modern.css`, `calendar-leaderboard.css` |
| Inline `<style>` blocks | **87 templates, 5 782 lines** |
| Public templates extending a shared layout | **0 of 45** — every public page is a standalone HTML document |
| Admin sidebar | **29 entries, 3 generic groups, zero feature gating** |
| Longest public templates | `formation-suivi` 1 199 · `profil` 714 · `machine-historique` 663 · `calendrier` 589 (395 of them inline JS) |

Two of those are the real story. **The public site — the part members actually use — never got the treatment the admin got in S29**: no shared layout, so every page re-declares its own head, its own stylesheet set and its own spacing. And **the admin sidebar ignores the feature model entirely**, so an events-only deployment still sees Machines, Espaces, Matériaux, Prêts and Maintenance in its own navigation, which is precisely the promise Phase A made and this screen breaks.

---

### How this phase works — read before starting any of it

#### The stack is already there and unused

`symfony/ux-turbo`, `symfony/stimulus-bundle` and `symfony/asset-mapper` are **all in `composer.json` today**, and:

- **Turbo is used in exactly 0 templates.**
- The only Stimulus controllers are the two scaffolding defaults, `hello_controller.js` and `csrf_protection_controller.js`.
- Meanwhile `calendrier.html.twig` hand-writes **395 lines of inline JS**.

So the modern interaction layer this phase needs is installed, paid for and idle. **Turbo Frames give inline editing and partial page updates** (S49, S51) with no client-side framework and no duplicated rendering — the server keeps rendering Twig, which is the architecture this app deliberately has. **Stimulus is for the sprinkles**: a date picker, a disclosure toggle, a segmented control.

⚠️ **Do not add a front-end framework.** "Twig server-rendered, no SPA" is a stated architectural decision, not an accident, and it is what makes the feature-gating model work — a route that 404s takes its UI with it. A React island would need the gate reimplemented client-side, which is how "visibility is not permission" gets violated by accident.

⚠️ **Do not write more inline JS either.** Every line added to a `<style>` or `<script>` block in a template is a line the next audit cannot find. The 395 in the calendar were moved into that page's `javascripts` block by S46 and are S47's to rewrite, not a pattern to follow.

#### Anti-goals

- **Not a rebrand.** S31 made the words the operator's and S27 made the accent colour theirs. A redesign that hardcodes new ones takes that back.
- **Not a rewrite.** Every session here should be revertible on its own and leave the app running, exactly like every session before it.
- **Not "modern" at the cost of legible.** A vibrant palette that fails AA is not modern, it is broken — see the measured table in S45.
- **Not a components-for-their-own-sake exercise.** A component earns its place by replacing at least two hand-rolled copies.

#### The verification recipe, once, for all of these

Every session in this phase is visual, and this project already learned how to check visual work without guessing (`PROJECT_STATE.md` §9):

1. `php bin/console app:render <path> --save=…` for each affected page.
2. Serve the saved HTML locally against the **public** stylesheets; drive it with the browser.
3. **Measure, don't eyeball** — computed contrast and geometry in the DOM. S29's whole yield came from printing computed styles; none of it from looking.
4. ⚠️ **Strip `main.js` and the inline theme bootstrap, and pin `data-theme`** — the signed-in user's stored theme is baked into every rendered page, so an unmodified "light" render is dark on a dark machine. A whole audit pass was silently run against one theme twice before this was noticed.
5. ⚠️ **Bump the `?v=` cache-buster** on any stylesheet touched, or the change reaches nobody who has visited before.

#### Order and dependencies

```
S45 tokens ✅ ┬─> S46 public layout ✅ ┬─> S47 booking flow
             │                        ├─> S48 disclosure
             │                        └─> S52 mobile
             ├─> S51 components ──────┴─> S49 role surfaces
             └─> S53 retire stylesheets   (last — it is the clean-up the rest earns)

independent, do any time: S50 admin nav ✅ · S54 dead buttons ✅ · S55 single-feature look ✅
spun out of S54:          S56 password reset · S57 account deletion
```

**S45 and S46 went before Phase D and are done (2026-07-31).** The LMS adds around ten screens; built against the old chrome they would have been built twice. Everything else in Phase U can wait for the LMS to land.

**S54 was the cheapest win in the phase and is done (2026-07-31)** — it came out at −748 lines for +25, and it turned up two features that were advertised but had no backend at all (**S56** password reset, **S57** account deletion). ⚠️ **The "19 buttons" estimate was low by half**: grep for the `disabled` attribute, not for "bientôt disponible" — a third of the dead controls never apologised.

#### A type scale for the "big bold numbers"

The principle is "large, bold numbers for key information", which needs actual sizes or every page invents its own:

| Token | Size / weight | Used for |
|---|---|---|
| `--fs-display` | 48–64px / 800 | the one number a page is about — minutes used, rank, places left |
| `--fs-stat` | 32px / 700 | stat tiles in a row |
| `--fs-h1` | 30px / 800 | page title *(already the admin's size — keep)* |
| `--fs-h2` | 22px / 700 | panel titles *(already in use)* |
| `--fs-body` | 15–16px / 400 | prose |
| `--fs-meta` | 13px / 500 | labels, timestamps, help text |
| `--fs-micro` | 12px / 600 | chips, tags, table headers |

⚠️ **One display number per screen.** If everything is 56px, nothing is. The dashboard's six stat tiles are `--fs-stat`, not six displays.

---

### S45 · Design tokens: one palette, one type scale, one rhythm — ✅ shipped 2026-07-31

**Why.** Everything else in this phase is "make page X nicer", and without a token layer each of those sessions ends by adding a fifteenth stylesheet. This is the session that makes the others cheap.

**Scope.** One token sheet — colour, type scale (including **display sizes for big bold numbers**), spacing, radius, elevation, motion durations — expressed as CSS custom properties layered *over* the existing `--theme-*` variables rather than replacing them. A vibrant accent to move away from the current muted palette.

⚠️ **The accent must stay the operator's, not ours.** `portal_primary_color` already exists per portal and S31 made the words configurable; hardcoding a fashionable brand colour would walk that back. Ship a good *default* and keep the override path.

⚠️ **Contrast is a gate, not a preference.** Brand pink `#9E1B56` is **1.97:1** as text on the dark panel — that is why the admin needed lifted tints. Every token pair goes through the S29 harness (`PROJECT_STATE.md` §9) in both themes before adoption. A vibrant palette that fails AA is not a modern palette.

**Verify.** A swatch page rendering every token pair in both themes with its measured ratio, and one page migrated as proof the tokens are usable.

**✅ What shipped.** The token layer extends the existing `:root` vocabulary rather than starting a parallel one — spacing, radius, shadow and transition were already there, so only the gaps were filled: semantic pairs promoted out of `admin.css` (the public side could not reach them), `--font-size-display: 56px` (the ramp stopped at 40px, a heading rather than a number you can read across a room), weight tokens, and the accent's text form.

⚠️ **The structural fix: `--color-primary-text`.** The dark block overrode fifteen variables and *not* `--color-primary`, which is why every use of the brand colour as text failed on dark and why S29 had to hand-patch classes one at a time. The dark value is **derived, not fixed** — `color-mix(in srgb, var(--color-primary) 50%, white)` — so a portal that sets its own accent gets a legible dark variant for free. 50% was chosen by measurement: it keeps every candidate accent between 5.75:1 and 7.08:1 on `--theme-surface`.

**`/admin/design` is the reference the rest of Phase U checks against.** It reads computed values off `:root` and measures contrast in the browser, so it cannot claim a ratio the stylesheet does not deliver, and it reflects a portal's own accent.

⚠️ **That page shipped wrong twice before it shipped right**, caught by reading its own output rather than trusting it: `color-mix()` computes to `color(srgb 0.81 0.55 0.67)`, three numbers on a 0–1 scale. Reading them as bytes reported the accent at 1.39:1; assuming `srgb` contributed a digit returned NaN. **If a colour token ever reads as absurd, suspect the parser before the palette.**

**Verified** in both themes, all eight text roles ≥ 4.5:1 — dark 5.77→13.65, light 5.02→17.40. Cache-buster bumped on both stylesheets across 81 templates.

#### A validated starting palette, so S45 does not begin by guessing

⚠️ **The finding that shapes the whole token layer: no single accent hex can be text in both themes.** Every candidate was measured against white and against the dark panel `#2b2335`:

| candidate | on white | on dark panel |
|---|---|---|
| `#9E1B56` *(today's brand)* | 7.65 ✅ | **1.97 ❌** |
| `#D6246E` vivid rose | 4.84 ✅ | **3.11 ❌** |
| `#6D28D9` violet | 7.10 ✅ | **2.12 ❌** |
| `#0D9488` teal | **3.74 ❌** | **4.02 ❌** |

Anything saturated enough to be vibrant and dark enough to read on white is too dark to read on `#2b2335`; lighten it until it works there and it fails on white. **So every colour role is a pair — a light-theme value and a dark-theme value — never one hex.** This is exactly the shape `.admin-status-*` already has from S29; the token layer generalises it rather than inventing something new.

**A starting set, all measured, all passing 4.5:1 in their own theme:**

| Role | Light | on white | Dark | on panel | on elevated |
|---|---|---|---|---|---|
| primary (rose — keeps the brand family, more saturated) | `#C2185B` | 5.87 | `#FF8FB8` | 7.08 | 6.30 |
| primary (violet — if a cleaner break is wanted) | `#6D28D9` | 7.10 | `#C4B5FD` | 8.16 | 7.25 |
| accent (cyan — for secondary actions and data viz) | `#0E7490` | 5.36 | `#67E8F9` | 10.39 | 9.24 |
| danger | `#b91c1c` | 6.47 | `#fca5a5` | 7.93 | 7.06 |
| warn | `#b45309` | 5.02 | `#fcd34d` | 10.44 | 9.29 |
| ok | `#15803d` | 5.02 | `#86efac` | 10.72 | 9.54 |
| info | `#1d4ed8` | 6.70 | `#93c5fd` | 8.35 | 7.43 |

The four semantic rows are **already shipped and in use** — S29 measured and deployed them as `.admin-status-*`. Only the primary and accent rows are new, which makes S45 much smaller than it looks.

⚠️ **`#C2185B` doubles as a button fill with white text at 5.87:1**, so one value covers "primary text" and "primary button" in light mode. Do not assume that holds for whatever replaces it — check `white on X` as well as `X on white`.

⚠️ **This is a *default*, not a brand decision.** `portal_primary_color` already lets each portal override the accent, and S28 validates it as a hex on read *and* on save. Whatever S45 picks must flow through that same setting, or the palette work quietly un-does S27.

---

### S46 · One public layout — ✅ **shipped 2026-07-31**

**Why.** **0 of 45 public templates extended `base.html.twig`** (14 lines, doing nothing). Every public page was a standalone document re-declaring its head and stylesheet set. Same disease as the admin's six skeletons, on the side that matters more, and the reason the public pages drifted apart.

**What shipped.** `site/base_public.html.twig`, extended by **all 37 pages that carry the header and footer**. It owns the doctype, the head, the favicon, `style.css` and its cache-bust, and the header/footer includes. Children fill `title`, `head`, `stylesheets`, `page_styles`, `body`, `javascripts`, and set `body_class` / `html_attrs` as **variables** rather than blocks.

The other 8 public templates (kiosk ×4, `event-ticket`, `staff-scan`, `recherche`, `creation-new`) include **neither header nor footer** — a different medium, deliberately out of scope, not pages that were missed.

**How it was verified.** All 37 rendered with `app:render --save` before and after, then compared. Each batch came back identical once five differences were normalised away, and every one of the five is deliberate:

| Difference | Why it is expected |
|---|---|
| head whitespace collapsing | the base's `{%-` controls |
| `<html lang="fr">` → the request locale | **the i18n bug the base fixes as a side effect** — 20 public pages hardcoded `fr` on a site shipping five languages |
| `style.css` gaining `?v=20260731-tokens` | six pages were still asking for the un-busted file |
| CSRF tokens | per-request |
| `noindex` meta moving up inside `<head>` | the two mail-reached pages, now in the `head` block |

**Anything else in that diff is a regression.** Normalise the five, then demand equality — do not read diffs by eye. Two findings came out of exactly that and would not have survived eyeballing:

⚠️ **`profil` server-renders `data-theme-preference` on `<html>`**, and the first pass dropped it silently — the base's `<html>` tag is fixed. Hence **`html_attrs`**, printed raw because it is markup, so **values are escaped at the call site** (`~ user.theme|e('html_attr') ~`). It is a one-page hook, not an attribute bag.

⚠️ **Four pages carried a `BANDEAU D'ALERTE` comment banner above the header include.** On `index.html.twig` there was a real second alert bar behind it (fixed in 9613eb2 — the homepage was emitting two); on `login`, `profil` and `register` only the dead label. Both are the same fossil from when the alert bar was per-page.

⚠️ **The base loads no `main.js`, and `body_class` is a `set` variable.** Both from measuring: main.js is on 29 public pages and absent from the rest, so a default would add a script where none existed; and a `body_class` *block* would leave `class=""` on forty pages that never had one. ⚠️ Two pages interpolate `is_granted` into their body class — that needs a Twig **concatenation**, not a quoted copy of the attribute, or the `{{ }}` renders literally.

⚠️ **A separate base from `base.html.twig` on purpose** — that one is extended by 23 admin templates, and widening it would have meant changing the admin's shell inside a public-side session. **S53 merges them.**

⚠️ **The calendar's 395 lines of inline JS were moved, not rewritten** — they are S47's, and they now sit in that page's `javascripts` block.

**Verify.** ✅ 37 of 37 identical under the five normalisations above. Rendered HTML being unchanged is also what carries "in both themes": the theme is CSS- and `data-theme`-driven, and the one page that server-renders a theme attribute keeps it.

---

### S47 · The booking flow, in as few steps as it can honestly take — 🟡 **partly shipped 2026-08-01**

**Why.** Booking is what the app is *for*, and it is the one flow the plan admits has never been verified end to end. The calendar is a 589-line page with 395 lines of inline JS.

**Scope.** Pick resource → see availability → confirm. Smart defaults throughout: the nearest bookable slot pre-selected, the member's usual duration remembered, and **slots that cannot be booked never offered** rather than offered and refused. Inline explanation at the point of choice — "you need the laser badge for this", "you already hold your two bookings this week" — instead of after submit.

⚠️ **Client-side hints are a convenience; they are never the rule.** The three permission layers (certification, quotas, access passes) meet server-side in `ReservationService::book()` and stay there. Mirroring a quota in JS is a UX improvement and a security no-op.

⚠️ **Quota refusals are 409, not 403** — "this conflicts with what you already hold", not "you are not allowed". The wording must offer the fix (cancel something), because that is what makes it true.

⚠️ **Refusal order is deliberate: min-notice and horizon before slot alignment.** Get it wrong and a member fixes the alignment of a slot they were never allowed to book, then gets refused again.

**Verify.** Count clicks for one real booking before and after. Every refusal branch still reachable and still explained. **This is also the session to finally verify the happy path** (Phase H S44).

#### What shipped 2026-08-01 — and what did not

⚠️ **The click-count baseline below is partly stale, so read it with this.** Both calendars *already* meet the "book a specific slot" target: clicking a slot opens the panel with the date and both times filled from the click (`setBookingSelection` → `syncBookingHiddenFields`), the start/end are hidden inputs driven by two selects rather than four empty fields, and `motif` is already labelled optional on both. Re-measure before "fixing" that row again.

**Shipped: the answer, up front.** `machine-detail` now computes the next slot *this person could actually book* and puts it on the primary action — "Réserver — libre le 03/08 à 08:00" — instead of a bare *Réserver* that opens a grid to be read by eye. New `src/Reservation/NextFreeSlotService.php`.

- **It validates candidates *through* `BookingPolicyService`**, it does not reimplement min-notice, horizon, alignment or quotas. A second copy would drift, and the copy is what members would see. It walks the slot grid and asks the real checker about each, taking the first not refused.
- **It suggests, it never permits.** `ReservationService::book()` still runs every layer; a slot suggested and then taken is still refused there.
- **Anonymous visitors get opening-hours-and-overlap only** — quotas are per person, so with no user the honest claim is "the resource is free then", not "you may book it".
- **No slot is offered on a machine the member is not certified for**, which would be a promise `book()` refuses.

⚠️ **The timezone warning below is real and it fired immediately.** The service pins `Europe/Paris` and produced 08:00; the button advertised **06:00**. Twig's `|date` renders in the app default, which is **UTC**, so a correct value was converted on the way out. **Pin the zone on the read as well as the write.** Caught only by diffing the rendered button against `/api/opening-hours` — the render looked entirely plausible on its own.

**Also shipped 2026-08-01:** `place-detail`'s three empty inputs now open on the next bookable slot, from the same service. ⚠️ Guarded with `|default(null)` — `twig.yaml` sets `strict_variables: true` and `renderPlaceBookingError()` re-renders that template without the variable, so an unguarded reference would have 500'd **every failed booking submit**, which is exactly the path where the member has already made one mistake.

⚠️ **Availability on the `/machines` card face was deliberately NOT built.** There are 11 machines and the list is *not* server-paginated, so a per-card next-free-slot is 11+ extra queries growing linearly with the lab. It needs one batched query for all upcoming reservations grouped by resource, and a session that can measure the result — Phase H's S41 is about exactly this class of query. Doing it naively would trade a UI win for the performance problem the plan already knows about.

**Not done, still S47's:** "book next free" as a true one-click confirm (it links to the calendar, so it is 2 interactions, not 1); availability on the card face in `/machines` (see above); cancel-with-undo; the smart-defaults inventory (`place-detail`'s three empty inputs and the six admin form defaults); making `motif` optional *per site*. **And the happy path is still unverified end to end** — that needs a booking to actually be created, which means real rows in production, so it stays with the operator (Phase H S44).

#### The click-count baseline S47 has to beat

Traced through the templates on 2026-07-31. These are the numbers to measure against afterwards — "it feels faster" is not a result.

**Booking a machine, as a member, from the home page — 3 page loads, 5 interactions, 5 fields:**

1. click **Machines** in the nav → `/machines`
2. click a machine card → `/machines/{id}` (`machine-detail`)
3. click **Réserver** → `/machines/{id}/calendrier` (`machine-calendar`)
4. click a slot → modal opens
5. fill `booking-start` + `booking-start-time` + `booking-end` + `booking-end-time` + `booking-motif`, then submit → `api_reservation_create`

The shared `/calendrier` grid is the same shape with a resource picker (`booking-machine`, `resourceKey`) instead of step 2–3.

**What is wrong with it, specifically:**

- **Three page loads before the member sees a single free slot.** Availability — the only thing they came for — is four clicks deep.
- **The modal asks for a start date, a start time, an end date and an end time as four separate inputs**, all empty (see the defaults inventory above), when the member has just clicked the exact slot they want. The click already contains the answer.
- **`motif` is required-looking on every booking.** For a member booking a printer for an hour, "reason" is a tax on the common case.
- The date/time inputs re-ask for the day the member is already looking at.

**Targets for S47** — pick from these, do not do all of them:

| Flow | Now | Target |
|---|---|---|
| Book the next free slot on a known machine | 5 interactions | **1** — a "book next free: 14:00–15:00" primary action on `machine-detail`, confirmed inline |
| Book a specific slot | 5 + 5 fields | **2** — click the slot, confirm; times come from the slot, `motif` optional and remembered |
| See whether a machine is free at all | 2 page loads | **0** — availability on the card face in `/machines` (see the catalogue) |
| Cancel a booking | list → confirm dialog | **1 + undo** — cancel immediately, offer undo for a few seconds |

⚠️ **Fewer clicks must not mean fewer refusals.** All three permission layers still run server-side on every one of these; a one-click path that skips `ReservationService::book()` is not a shortcut, it is a hole. What one-click removes is *typing*, not *checking*.

⚠️ **`motif` may be load-bearing for some operators** — a shared workshop may genuinely want to know why a machine is held for three hours. Make it optional per site rather than deleting it, and default it to empty rather than to a placeholder that becomes noise in the admin list.

#### The smart-defaults inventory

Every date and time input in the app was checked on 2026-07-31. **Not one of them opens with a useful value.** The three on `place-detail` look pre-filled — `value="{{ submitted.date ?? '' }}"` — but that only echoes back what the member already typed after a failed submit; on first load all three are empty. `EventAdminType`, `LoanAdminType` and `MaintenanceTaskAdminType` set no `'data' =>` default either.

So booking a space today means typing a date and two times from nothing, on a page that already knows the opening hours, the existing bookings and the minimum notice.

| Input | Today | Should open at | The app already knows this from |
|---|---|---|---|
| `place-detail` date / start / end | empty ×3 | next open slot that satisfies min-notice, duration = this member's usual | `OpeningHoursProvider`, `ReservationRepository`, `BookingPolicyService` |
| `person-booking` (2) | empty | the bookable person's next published availability | `UserAvailabilityRepository` |
| `my-availability` (2) | empty | next week, mirroring last week's pattern | the owner's existing rows |
| `staff-access-passes` (2) | empty | today → +7 days, the common case | — |
| `EventAdminType` start/end | empty | next round hour, end = start + last event's median duration | `EventRepository` |
| `LoanAdminType` due date | empty | today + the site's default loan period | `loans` settings |
| `MaintenanceTaskAdminType` due | empty | today + the task's recurrence interval | the task being cloned |
| `admin-reservations`, `admin-usage-logs` filters | 2 of 4 / 2 of 2 filled | ✅ leave alone — these are the pattern to copy |

⚠️ **A default is a suggestion, never a constraint.** The point is that the common case needs no typing, not that the uncommon one becomes impossible. Every one of these stays editable, and the *server* still validates — a pre-filled slot that has since been taken must still be refused by `ReservationService::book()`.

⚠️ **Defaults must respect the quota rules, or they teach the wrong thing.** Offering "now + 10 minutes" on a site with a two-hour minimum notice produces a form that is wrong the instant it loads. Compute the default *through* `BookingPolicyService`, not around it.

⚠️ **Timezone.** The box runs UTC and the booking flow uses `Europe/Paris`. A default computed server-side and rendered into a `datetime-local` will drift by the offset unless the zone is pinned on the write *and* the read — this has bitten before (an access pass tested as valid after expiry).

---

### S48 · Progressive disclosure on the pages that are walls of text — 🟡 **partly shipped 2026-08-01**

**Why.** `formation-suivi` is 1 199 lines, `profil` 714, `machine-historique` 663, `machines` 503. These are not complicated ideas; they are everything shown at once.

**Scope.** Essential-first on each, with the rest behind disclosure — summary card, then "show more". Per page: **profil** → identity + next booking + badges held, history collapsed. ⚠️ Its "tabs" are anchor links (`#info`, `#badges`) and every section renders at once — this is a real restructure, not a CSS change. **machine-detail** → can I book it, when, and what do I need; specs and accepted materials collapsed. **machine-historique** → paginated, filtered by default to the recent window. **formation-suivi** → current step foregrounded, completed steps collapsed. **machines** → card grid with availability as the primary signal.

⚠️ **Never collapse safety information.** Certification requirements, hazards and the reason a booking was refused stay visible by default. Progressive disclosure is for volume, not for consequence.

#### What shipped 2026-08-01

**Native `<details>`, no JavaScript.** ⚠️ **Stimulus cannot be used for this yet: `importmap()` is called in exactly zero templates, so AssetMapper never loads and neither Turbo nor Stimulus runs on any page.** The bundles are installed and inert. Wiring them up is **S51's** job and is not a thing to slip into a disclosure session — Turbo eager-loads and would start intercepting every navigation. Native disclosure also degrades to "always open" and keeps content in the DOM for in-page search and print.

- **`machine-detail`** — technical bookkeeping behind disclosure; certification requirements untouched and open.
- **`profil`** — reservation history collapsed, and **open by default when there are five or fewer**, because collapsing three rows hides nothing and costs a click.

**Still S48's:** `machine-historique` (pagination + a default recent window), `formation-suivi` (current step foregrounded, completed collapsed), `machines` (availability as the primary card signal), and `profil`'s deeper restructure.

---

#### 🔴 Found during S48: machine device tokens were public — **half-fixed, needs the operator**

**`machine.machineToken` was rendered to anonymous visitors** on `machine-detail`, `machine-historique` (twice) and the homepage machine cards. It reads like a label — `printer-01` — and is not one: it is the path segment that addresses the machine on the device API.

```
POST  /api/rfid/machines/{machineToken}/authorization
POST  /api/rfid/machines/{machineToken}/work-sessions
PATCH /api/rfid/machines/{machineToken}/work-sessions/current
```

**And `RfidMachineController::rejectUnauthorizedDevice()` fails open**: when `FABOS_RFID_API_TOKEN` is empty it returns `null` — allowed. **That variable is not set on the live box**, so those endpoints accept anyone, and the public pages were publishing the identifier needed to address them. This is the same finding Phase H recorded as "the RFID device endpoints have no authentication at all", now with the other half of it: the machine identifier was public too.

✅ **Fixed 2026-08-01:** the token is gone from all three public templates (admin screens still show it, which is where it belongs). Verified: zero hits on `/machines/1`, `/machines/1/historique`, `/` and `/machines`.

❌ **Not fixed, and not mine to fix:**
1. **Set `FABOS_RFID_API_TOKEN`** in `/opt/fabos/FabApp/.env.local` and on the devices — it is secret material, so the operator generates it.
2. **Make the check fail closed** (S38/S39). An unset token must refuse, not allow. Until then the endpoints are open to anyone who can guess a machine token, and `printer-01` is a guess.

⚠️ The endpoints were **not** exercised against production to demonstrate this — the reasoning is from the code and the env, deliberately.

---

### S49 · Where each role changes data — 🟡 **partly shipped 2026-08-01**

**Why.** S30 answered "edit this thing" for admins on six detail pages. The unanswered question is the shape of the whole app: a member checking availability and a staff member moving somebody's booking are doing different jobs on the same screen, and today both are sent to the admin panel.

**Scope.** Decide, per surface, who edits what and where — then build the affordances:

| Surface | Member | Staff | Admin |
|---|---|---|---|
| Calendar / a slot | book, cancel **their own** | move, reassign, cancel **anyone's**, inline on the grid | same as staff |
| Machine page | see availability + what they need | mark out of service, add a maintenance note inline | edit the record |
| Event page | register, cancel their place | check-in, manage the roster inline | edit, cancel the event |
| A person | edit their own profile | issue an access pass, see held badges | roles, authorisation |
| Configuration | — | — | admin panel only |

The pattern: **members act on their own records in place; staff act on other people's records in place; admins configure the site in the panel.** Global configuration stays in the panel — that boundary is what keeps the public pages readable.

⚠️ **There is no role hierarchy in this app: `ROLE_ADMIN` does not imply `ROLE_STAFF`.** S30 gated a chip on `ROLE_STAFF` whose route was under `/admin` and it vanished for the only people who could use it. Every affordance's role must mirror its target route's own `access_control` line.

⚠️ **Visibility is never permission.** Every one of these is a shortcut; the firewall decides on each request. Verify both halves — affordance absent for the wrong role *and* the endpoint refusing.

#### What shipped 2026-08-01 — the mechanism, not yet all the surfaces

**Affordances no longer name a role; they ask the route's own rule.** New `can_reach('route', params)` Twig function (`src/Twig/RouteAccessExtension.php`) consults **`security.access_map` — the same `access_control` map the firewall itself uses**. There is no second copy to drift: edit `security.yaml` and every affordance follows. Multiple roles on one line mean "any of these", matching the firewall's affirmative decision.

That is the direct answer to the no-role-hierarchy warning above. Hand-writing a role is guesswork about a rule that lives somewhere else, and S30 already lost a chip to it.

- **`_admin_inline.html.twig`** — callers stop passing `role:`. An explicit `role:` still works, but now as an *extra* restriction on top of the route's rule rather than a replacement.
- **`_admin_sidebar.html.twig`** — entries render only if reachable, on top of S50's feature gate. **This fixed a real bug**: `staff-access-passes` is the one non-admin template that includes the admin sidebar, so a staff member who is not an admin reached it legitimately and was shown **thirty `/admin` entries, every one of which bounces them to `/login`**.

**Verified both directions on the real stack**, by rendering the same pages as two accounts: an admin gets the chips (2 on `/formations/1`, 1 on `/machines/1`, plus 21 inline-edit links) and a plain member gets **zero**. The admin sidebar is unchanged at 30 entries across 6 groups.

⚠️ **This install has no staff-but-not-admin account**, so the exact case the sidebar bug affected could not be reproduced end to end — the fix is verified by the mechanism and by the member/admin split, not by that persona. Worth creating one before trusting the staff surfaces.

⚠️ **`can_reach` answers "may you open this URL", not "may you do this to this record."** Per-row questions — is this *your* booking, is this event yours to cancel — belong to controllers and voters, and must not migrate into it.

**Not done, still S49's:** the surface table above is a design brief, and only its *access plumbing* exists now. Nobody has yet built inline staff editing on the calendar grid, mark-out-of-service on the machine page, roster check-in on the event page, or member edit-in-place on the profile (the last is also what S54 removed a dead button for).

---

### S50 · Admin navigation that follows the feature model — ✅ **shipped 2026-07-31**

**Why.** The sidebar has **29 entries in three generic groups** (Contenu, Réglages, Stats & Live) and **gates none of them by feature**. An events-only portal — the exact deployment Phase A exists to make possible — still sees Machines, Espaces, Matériaux, Prêts and Maintenance in its admin. The public nav solved this in S24; the admin never did.

**Scope.** Gate every entry by its feature, and regroup along the registry's own vocabulary (**resource / activity / directory**, plus kernel) instead of "Contenu". Collapse the rarely-touched settings behind a single entry.

⚠️ **Reuse `NavBuilder`'s rules rather than inventing a second nav system**: a group with no visible children is never rendered, a group's own link follows its children, and visibility is presentation and never permission.

⚠️ **Kernel screens are never gated** — users, roles, auth, settings, portals, mail. An admin who switched everything off must still be able to switch something back on.

**Verify.** Toggle a feature off and the entry disappears from the admin as well as the public nav; with everything off, the sidebar still offers the kernel screens.

#### What actually shipped

30 entries (not 29 — `/admin/design` arrived with S45), each carrying its gate, regrouped exactly as the table below says. **The gating alone would have been a bug**: all four sidebar variants printed `<span class="admin-nav-group-label">` unconditionally, so hiding every child of a group would have left a bare heading over nothing. Emptiness is now checked per group, in all four.

⚠️ **The warning below about `Accès exceptionnels` was stale and is now resolved.** `security.yaml` grants `^/staff` to **`[ROLE_STAFF, ROLE_ADMIN]`**, so an admin can open it; the entry stays in the sidebar. (`NavBuilder` shows it in the *public* header only for staff who are not admins — the two are consistent, not contradictory.)

⚠️ **Verifying this needed more than the live site: all 14 features are switched on there, so nothing exercised a gate.** Rendering `/admin` proved the six groups appear, and no more. The gating itself was tested by running the template's own `|filter` expressions through Twig with `feature_enabled` and `has_calendar_layer` stubbed, over four deployment shapes — everything on, events-only, everything off, spaces-only. Results: events-only drops the whole *Réserver* group with no stray heading; everything-off still lists the kernel; **spaces-only keeps *Réservations*, which is the polymorphic rule the table demands and the one a naive `machines` gate would have broken.** *(An install where a feature is genuinely off would have caught this for free — this one had nothing to offer, so the check had to be built.)*

#### The concrete regrouping S50 should land

All 29 entries as they stand, with the feature that should gate each. **Fourteen of them belong to a switchable feature and none is gated today** — which is what an events-only operator currently sees in their sidebar.

| Proposed group | Entry | Gate by feature |
|---|---|---|
| *(always, no group)* | Tableau de bord · État de l'installation | kernel — never gated |
| **Réserver** *(resource)* | Machines · Espaces · Réservations · Quotas de réservation · Accès exceptionnels | `machines` · `places` · any resource on · any resource on · any resource on |
| **Activités** *(activity)* | Événements · Formations · Badges · Créations · Matériaux · Objets prêtables · Prêts · Maintenance · Pages du Lab | `events` · `formations` · `badges` · `projects` · `materials` · `loans` · `loans` · `maintenance` · `lab_pages` |
| **Annuaires** *(directory)* | *(none today — the staff/trainer directories have no admin screen)* | `staff` · `trainers` |
| **Les gens** *(kernel)* | Utilisateurs · Institutions | never gated |
| **Le lieu** | Horaires · Interface accueil · Portails | kernel · kernel · kernel |
| **Suivi** | Utilisations · Logs RFID · Lecteurs RFID · Pages introuvables | `machines` · `machines` · `machines` · kernel |
| **Configuration** *(collapsed by default)* | Fonctionnalités · Réglages du site · E-mails · Configuration initiale | never gated |

**What this buys.** An events-only portal's sidebar becomes: Tableau de bord · **Activités** (Événements) · Les gens · Le lieu · Configuration — five groups instead of twenty-nine entries, and not one of them mentions a machine. That is the Phase A promise finally reaching the screen the operator spends their time on.

⚠️ **Three traps in this table specifically.**

- **`Réservations`, `Quotas` and `Accès exceptionnels` are gated on "any resource feature", not on `machines`.** Bookings are polymorphic since S8–S10; gating the booking screens on equipment would hide them from a spaces-only or appointments-only deployment that books perfectly well.
- **`Accès exceptionnels` is a `/staff` route living in an admin sidebar.** With no role hierarchy, an admin who is not staff cannot open it — check the `access_control` line before deciding whether it belongs here at all (S49's rule).
- **`Prêts` and `Objets prêtables` share one feature** (`loans`); two entries, one switch. Group them or the operator will look for two toggles.

⚠️ **`Institutions` and `Utilisateurs` are kernel and must never be gated** — an operator who switched everything off still has to be able to manage people and switch something back on.

---

### S51 · Feedback, motion and the components to build it from — 🟡 **partly shipped 2026-08-01**

**Why.** Actions today mostly succeed by reloading the page. Nothing acknowledges a click.

**Scope.** A small component set, used everywhere rather than reinvented per page:

- **Cards** as the default container for a thing (machine, event, booking, member).
- **Toasts** replacing full-page flash reloads for anything that does not change the page.
- **Skeleton loaders** for the calendar and lists, instead of a blank gap.
- **Inline edit-in-place** for staff (see S49) — click the value, not a link to a form.
- **Segmented controls and toggles** replacing single-choice selects.
- **Date/time pickers** replacing the paired text inputs in the booking and event forms.
- **Sheets/drawers** for detail-on-demand instead of a page navigation.
- **A floating action button** on mobile for the one primary action of each screen (book, register, add).
- **Stat tiles** with large bold numbers for the dashboard, leaderboard and profile.

⚠️ **Respect `prefers-reduced-motion`** — every animation needs a no-motion path.

⚠️ **Optimistic feedback must not claim success the server has not given.** The booking chokepoint can still refuse; a green tick that gets retracted is worse than a half-second wait.

#### What shipped 2026-08-01

`public/css/components.css`, loaded once from the public layout, built only from S45 tokens. **Two components, because two are used:** toasts (the server-rendered flashes on `/` and `/profil`) and a mobile FAB (the machine page's primary action).

⚠️ **Cards, skeleton loaders, segmented controls and stat tiles were written and then deleted before commit.** Nothing referenced them, and shipping CSS nothing uses is the dead weight this phase keeps removing. They go in when a surface adopts them — not before.

#### The front-end decision — settled 2026-08-01: **AssetMapper on, Stimulus only, Turbo off**

The decision was recorded as a binary ("AssetMapper on or off"). It is not one — there are **three** levers, and only one is site-wide:

- **AssetMapper** — where the JS lives and how it is built. Local.
- **Stimulus** — behaviour attached to markup by `data-controller`. Local, opt-in per element.
- **Turbo Drive** — intercepts every link and form. **Site-wide, and it switches itself on**: `assets/controllers.json` shipped `turbo-core` as `enabled: true, fetch: eager`, so the first call to `importmap()` would have started Turbo without a line of Turbo being written. That is the trap the original framing hid.

**What shipped.** `importmap('app')` in **10 templates** — both shells (`base.html.twig` 23 pages, `site/base_public.html.twig` 37) and the 8 standalone pages that carry inline JS. `turbo-core` set to `enabled: false`.

⚠️ **The "one admin shell" assumption was wrong.** `base.html.twig` covers only 23 templates (14 admin + 9 static); **53 admin/kiosk templates are standalone documents with their own `<head>`**, and *none* of the admin pages carrying real inline JS extend it. Scoping this to "the admin shell" would have reached 23 pages with nothing to gain and missed every page that needed it.

**Three scaffolding files deleted, and they were not inert:**
- `assets/styles/app.css` was `body { background-color: skyblue }`, imported by `app.js`. Calling `importmap()` with it in place would have turned **60 pages sky blue**.
- `csrf_protection_controller.js` is not a Stimulus controller at all — it is a global `submit` listener that rewrites `_csrf_token` fields for Turbo's stateless CSRF manager. Left in place it would have been live on every page.
- ⚠️ **A tarball cannot delete.** All three still existed on CT 210 after extraction and needed an explicit `rm`; skipping it would have shipped exactly what deleting them locally prevented.

**First adoption — `clock_controller.js`**, replacing the 3 hand-rolled kiosk clocks (`kiosk-entries` and `kiosk-stats` were identical to the character; `kiosk-events` was the two-element variant). −26 lines of inline JS, and it clears its `setInterval` on `disconnect()`, which none of the three did.

⚠️ **Found while replacing them: the kiosk clock rendered UTC.** `{{ 'now'|date('H:i') }}` with no zone, so the wall display showed **two hours early** for the first 10 seconds after every reload — and permanently if JS failed. The controller now pins `Europe/Paris` rather than trusting the Pi's own timezone. **This is systemic: 142 `|date()` filters across `templates/` pin no zone, against 9 that do.** Logged as its own task; only the two clocks rewritten here were fixed.

⚠️ **What Stimulus cannot take over: anything that must run before first paint.** `_header_auth` and `_admin_sidebar` carry byte-identical theme-bootstrap blocks — a textbook duplication, and exactly the wrong thing to convert. A Stimulus module is deferred, so it would run *after* first paint and trade a duplication for a flash of the wrong theme. They stay inline.

**Verified in prod, measured not eyeballed:** the importmap tag and 6 modulepreloads are emitted; every module returns 200; `window.Turbo` is `undefined`; `main.js` still works alongside it and the theme still applies; no console errors. The decisive check: the clock element was overwritten with a sentinel string and **Stimulus rewrote it 10 s later** — proof the controller is connected and ticking, not merely downloaded.

**Left deliberately:** `@hotwired/turbo` stays declared in `importmap.php`. Nothing imports it, so it is never preloaded or fetched — it costs one unused file in `public/assets/` and is the ready-made path the day Turbo is turned on.

🟡 **What the rest of S51 costs now.** Drawers and segmented controls need no JS at all (`<dialog>`, radios). Date pickers are native or a small controller. **Inline edit-in-place and toasts-without-reload were the two items that wanted Turbo Frames/Streams** — without them each is roughly 30–40 lines of fetch-and-swap, written once. That is the price of keeping navigation untouched, and it was paid knowingly.

Two rules encoded rather than assumed:
- **Success toasts fade after 6s; errors and warnings never do.** An error that removes itself is one the reader can miss, and it exists precisely because something did not happen.
- **`prefers-reduced-motion` gets no motion, not less** — and content stays visible, so the success toast that would have faded simply stays.

---

### S52 · Mobile and touch — 🟡 **partly shipped 2026-08-01**

**Why.** The half-wired mobile nav is a long-standing debt, and the calendar — the primary surface — has never been designed for a phone, which is where somebody standing next to a machine will actually book it.

**Scope.** Finish the mobile nav; make the calendar usable one-handed; audit touch target sizes; check every form on a narrow viewport.

**Verify.** The S29 harness at mobile and tablet presets, both themes.

#### What shipped 2026-08-01

**Measured first.** The burger was ~40×32 and nav links ~35px tall — both under the 44px minimum, on the surface someone uses while standing next to a machine.

- 44×44 minimum for the burger, nav and dropdown links, profile/detail/formation tabs, buttons, and inline admin actions.
- ⚠️ **Gated on `(pointer: coarse)`, not on width.** A narrow desktop window is still a mouse and a large tablet is still a finger; a width query would have changed desktop layouts for no reason.
- The mobile toggle flipped a class and **announced nothing** — a screen reader got the same thing open or shut. It now carries `aria-controls` and a kept-in-sync `aria-expanded`, and **Escape** closes it and returns focus to the button instead of stranding focus in a menu that has gone.
- Nav dropdowns already had `:focus-within` beside `:hover`, so touch and keyboard opening were fine — **checked before changing anything.**

**Still S52's:** the calendar one-handed (the actual headline), and every form on a narrow viewport. ⚠️ **Neither was verified at the mobile/tablet presets** — that needs the S29 browser harness, which this session did not run.

---

### S53 · Retire the fourteen stylesheets — 🟡 **barely started 2026-08-01**

**Why.** **460 KB of CSS, 135 KB of it four overlapping calendar files** (`calendar.css`, `calendar-fix.css`, `calendar-modern.css`, `calendar-leaderboard.css`), plus 5 782 lines of inline `<style>` across 87 templates. A file called `-fix` next to the file it fixes is a description of the problem.

**Scope.** Fold everything into the token system and the component set from S45/S51; delete what the migration makes dead; move what remains page-specific into the components it belongs to.

⚠️ **Last, not first.** This is the clean-up that the rest of the phase earns; done before them it is just churn.

⚠️ **The cache-buster is load-bearing.** Every stylesheet is referenced with a `?v=` string; changing a file without bumping it ships nothing to anyone who has visited before. S29 had to bump all 58 admin templates for exactly this reason.

**Verify.** Total CSS materially smaller, every page still renders identically, and the class audit finds nothing newly undefined.

#### What shipped 2026-08-01 — the provably-dead subset only

- **`calendar.css` deleted: 28 KB referenced by zero templates**, and imported by nothing (checked `templates/`, `src/`, `public/`, `assets/`, `config/`, and `@import` across the other sheets). The three calendar sheets that *are* loaded are untouched.
- **`index.html.twig` and `search.html.twig` each carried the same `<style>` block twice, byte for byte** — once in `page_styles`, again inside `javascripts`, left over from S46. Second copy removed after asserting the two were identical.

⚠️ **Deleting a file from the repo does not remove it from the box** — the deploy tarball can only add. It needs an explicit `rm` on CT 210 in the same step, which is what happened here.

**S53 remains almost entirely undone**, and deliberately: ~460 KB across thirteen sheets and ~5 700 lines of inline `<style>` in 87 templates. Folding the three overlapping calendar files together is the real work and needs the pages *driven in a browser*, not grepped — which is exactly why this session stopped at what could be proven dead by measurement.

---

### S54 · Remove the buttons that do nothing — ✅ **shipped 2026-07-31**

**Why.** The app currently advertises features it does not have. **19 "bientôt disponible" affordances across six templates** — five on `formation-detail`, ten on `admin-badges`, one each on `profil`, `register`, `forgot-password` and `admin-reservations` — plus a `disabled` *Edit* button sitting next to the profile's personal-details card. Nothing in this phase does more damage to how finished the app feels: a greyed-out button is a promise the software breaks every time somebody hovers it.

**Scope.** For each one, exactly one of three outcomes, decided per case and written down: **build it** (if it is a session's worth of work, it becomes a session), **remove it** (if nobody has missed it), or **replace it with a sentence** saying what to do instead. No fourth option, and specifically not "leave it, it explains a roadmap" — the roadmap is not on the page.

⚠️ **A disabled control is exempt from contrast rules, which is why these hide from audits.** WCAG 1.4.3 does not apply to inactive components, so S29's contrast harness passed straight over the *Gestion bientôt disponible* buttons on `/admin/badges`. Dead affordances have to be found by grep, not by measurement.

**Verify.** `grep -r "bientôt disponible" templates/` returns nothing, and every previously-disabled control either works or is gone.

#### What actually shipped

⚠️ **The count was wrong: it was 19 by one grep, but 40 static `disabled` attributes across 16 templates once you look for the attribute rather than the phrase.** The phrase-grep misses every dead control that never said "bientôt" — the ten filter and bulk-select widgets on `/admin/badges`, the three reservation-history filter tabs on `/profil`, the two "coming soon" tabs on `formation-suivi`. **Grep for `disabled`, not for the apology.**

Net: **−748 lines, +25.** Every decision, by page:

| Page | Dead affordance | Outcome |
|---|---|---|
| `admin-badges` | 2 bulk buttons, a **duplicate dead filter bar sitting directly under a working one**, select-all + per-row checkboxes, and a **69-line mock "create badge" modal that nothing ever opened** (`openAddBadgeModal` had no callers) | **removed** — 302 lines. `app_admin_badge_new` / `app_admin_badge_edit` and the `q` filter already worked; the page was a mockup layered on top of a working page |
| `admin-reservations` | Export button, "valider/annuler la sélection", checkbox column | **removed** — the JS behind them was three `alert()` stubs, and no CSV/export pattern exists anywhere in the admin |
| `profil` | `disabled` Edit, 3 reservation filter tabs, a one-option `disabled` `<select>`, 2FA, active sessions, delete account | **removed**; the `<select>` became the static label it always was. Edit-in-place is **S49's**, already scoped there |
| `formation-detail` | Enrol, add-to-calendar, per-session enrol, 3 share buttons | **removed** — "Voir ma progression" was already there and works, so it became the primary action |
| `formation-suivi` | Discussions + Notes tabs | **removed** |
| `formation-suivi` | `no_question` ×2, `practical_to_configure` | **replaced** — these were never "coming soon", they were **real states rendered as disabled buttons**. Now the note/badge spans their siblings already used |
| `login`, `register` | CAS buttons, `studentId` field | **removed** — no CAS config and no `studentId` exist anywhere in `src/`; pure mockup residue |
| `forgot-password` | Submit button, under a subtitle promising a reset email | **replaced with a sentence** → **S56** |

Also deleted because nothing referenced them any more: ~92 lines of `login-register.css` (`.btn-cas`, the divider group, `.field-disabled`), 24 lines of dead `.quick-action-card.disabled` rules, and **19 orphaned translation keys × 5 locales**. Locale key parity re-verified at 627 keys each, and every `|trans` key in every template resolves.

⚠️ **Removing a cell from a CSS-grid row means fixing `grid-template-columns` too** — both list tables declared a `40px`/`42px` checkbox column that outlived its checkboxes, which would have left every row indented by a phantom column.

⚠️ **Two of the CSS rules deleted were selector *lists* mixing dead and live selectors** — `html[data-theme="dark"] .field-note, … .divider-text` and a `.login-divider::before, … , .checkmark` group. Deleting the whole rule would have broken `.checkmark` in dark mode; blindly deleting the two dead lines from the first would have left a **bare `html[data-theme="dark"] {}` selector recolouring the entire page**. Prune selectors, don't delete rules.

**Found in passing, fixed:** `kiosk-events` asked for `kiosk.no_events`, which does not exist — a wall-mounted kiosk with no upcoming events was rendering the raw key string. `kiosk.events_empty` already existed with exactly the right text; the template referenced the wrong key.

#### Landed in the same commit, but not S54: *Mes réservations* and *Mes disponibilités* left the main nav

**Operator decision, 2026-07-31.** Both were top-level `header()` entries in `NavBuilder`. They are personal pages — one person's own bookings, one person's own availability — sitting in a menu whose every other entry is a thing the *site* has: the machines, the rooms, the training catalogue, the events. They are now reached from `/profil` only, where the reservations section had **already linked to both** long before this change; the destination needed no work.

Also removed from `footer()`, which the class documents as "a flat list of the same entries" — leaving it there would have made that sentence false.

⚠️ **This overrides a deliberate earlier judgement, and the comment explaining it is gone with it.** The old code gave *Mes disponibilités* a nav link specifically for people with `isBookable()`, reasoning they "manage their slots and answer requests often enough to deserve" one. If a bookable person later complains the page got harder to reach, that is this change, and the fix is a link in the account area — not putting a personal page back in the site menu.

### S56 · Password reset, end to end

**Why.** S54 found `/forgot-password` shipping a form whose submit button was `disabled`, under a subtitle promising "nous allons vous envoyer un lien de réinitialisation par email". The controller has a **`GET`-only route and no handler at all** — there is no reset feature, and never was. The page now says to ask an administrator, which is true but is a person doing a computer's job.

**Scope.** A reset-token entity (hash, expiry, single use), a POST handler that always answers the same whether or not the address exists, the mail itself, and the set-new-password form. The mail worker and `getPublicBaseUrl()` from S31 are already in place, so the link in the inbox will resolve.

⚠️ **This is a new ORM entity, so the migration ships and runs *before* the code** — the expand rule.

⚠️ **Do not reuse the `q`-style GET flow for the token.** A reset token in a query string lands in server logs, browser history and any `Referer` header the next page sends.

**Verify.** A reset mail arrives, its link works exactly once, a second use is refused, and an expired token is refused. Requesting a reset for an address that does not exist returns the same page and the same timing as one that does.

---

### S57 · Account deletion someone can actually reach

**Why.** S54 removed a `disabled` *Supprimer mon compte* button from the profile's danger zone — the affordance was dead, so it had to go, but the underlying right is real and the app now offers no route to it at all.

**Scope.** Decide erase-vs-anonymise **per table first, written down** — reservations, usage logs, RFID access logs and badge awards each have a different answer, and some are records the lab needs to keep. Then the confirmation flow, and the admin's side of it.

⚠️ **This is the one place where "delete" must not mean `ON DELETE CASCADE`.** S10 already proved that lesson on booking policies: a cascade here would silently delete lab history that has nothing to do with the person's identity.

---

### S55 · What a single-feature deployment actually looks like — ✅ **shipped 2026-08-01**

**Why.** Phase A made an events-only or lending-only install *possible*; this session makes one look like it was built that way rather than like a fablab with most of the lights off. S24 sorted the nav, S28 gave a portal its own front door, S50 fixes the admin sidebar — the front page itself was never checked.

**Of the six homepage blocks, two are feature-gated and four are not:**

| Block | Gated on | Verdict |
|---|---|---|
| `featured_machines` | `machines` ✅ | correct |
| `upcoming_events` | `events` ✅ | correct |
| `mini_leaderboard` | **nothing** | ⚠️ **bug** — `leaderboard` *is* a registry feature, so switching it off hides the nav entry and 404s `/leaderboard`, while the homepage keeps rendering a leaderboard block that links there |
| `fablab_stats` | **nothing** | machine and usage statistics on a site that may have neither — and the key name is itself a leftover of the vocabulary S31 removed |
| `how_it_works` | **nothing** | describes the equipment/badge/training journey to people who may have none of it |
| `opening_hours` | **nothing** | fine — a venue has hours whatever it does |

⚠️ **`mini_leaderboard` is the same bug S37 fixed on the error page**: an affordance pointing at a route the gate 404s. It is worth fixing on its own merits before the rest of this session.

**Scope.**
- Gate every homepage block on its feature, and rename `fablab_stats` now that the vocabulary is configurable.
- **Empty states worth landing on.** An events-only site with no events currently shows `events.none_upcoming` and nothing else; it should say what this place is and — for an admin — offer to create one. Same for a fresh lending library, a training catalogue with no courses.
- **Check the whole public surface against three personas**, not one: events-only, equipment-only, lending-only. Each should read as a coherent product.

⚠️ **This is a presentation session and must not become an authorisation one.** Blocks disappearing is the same rule as the nav: hiding a block hides a block. The route gate and the firewall decide what exists.

**Verify.** Switch every feature off but one, three times over, and walk the public site as an anonymous visitor: no block, link or empty state mentions a feature that is off, and nothing offered leads to a 404.

#### What actually shipped

**Seven blocks, not six** — `latest_rfid_logs` is a seventh, staff/admin-only, and it is machine data, so it gates on `machines` too.

**The gate went into `HomepageVisibilityService::getVisibilityMap()`, not into the call sites.** That map is what the controller loads data from, what the template renders from, *and* what `HomepagePersonalizationService` builds the section order out of. Those three could disagree and did: the controller checked `isEnabled('machines')` before loading featured machines while the template rendered the section from visibility alone. One rule, one place, three consumers.

| Block | Gate |
|---|---|
| `opening_hours`, `how_it_works` | kernel — a venue has hours whatever it does |
| `upcoming_events` | `events` |
| `fablab_stats`, `featured_machines`, `latest_rfid_logs` | `machines` |
| `mini_leaderboard` | `leaderboard` — **the live bug** |

**Also fixed:** `how_it_works` step 2 linked to `/formations` unconditionally, which 404s with `formations` off — now gated exactly as step 3's calendar link already was (keep the text, drop the link). And an events-only site with no events rendered *nothing at all* here; it now says so, and offers an admin the one action that fixes it.

⚠️ **`fablab_stats` keeps its key on purpose.** It is persisted in `HOMEPAGE_SECTION_VISIBILITY.sectionKey`, so renaming it is a data migration that changes nothing anyone sees. Only the **label** was the S31 vocabulary leftover, and only the label changed.

⚠️ **Which exposed a real bug: `buildRow()` preferred the stored `label` over the code default, so the rename appeared to work and did nothing.** The label is code-owned — the admin screen renders it as a caption, never an input, and `AdminController` writes it back from `DEFAULT_SECTIONS` on every save — so the stored value was a snapshot shadowing the code. Fixed; code now wins.

⚠️ **Verifying gates on this install is not possible: all 14 features are on.** What the live site *did* prove, for free: the new events empty state renders (there are no upcoming events), and `featured_machines` is absent because the operator has switched it off for all four audiences — not because anything broke. The check that mattered was mechanical: **every `feature` key in `DEFAULT_SECTIONS` validated against the registry's 14 real keys**, since a typo there hides a block forever and silently. *(Do that check whenever a table maps to feature keys — S50 needed the same one.)*

---

### The per-page catalogue

Read against the templates on 2026-07-31 — page sizes are real, and so are the specific problems. This is the working list for S46–S48 and S51; it is not a separate session.

| Page | Today | Change |
|---|---|---|
| **`index`** (354) | Hero + blocks, homepage visibility already role-gated and orderable | Lead with **one big number and one action** for the signed-in member — their next booking, or "book something". A logged-in member should not land on marketing. |
| **`machines`** (503) | **Already a card grid** (`machines-grid` / `machine-card`), with category filters and search | The cards exist; **availability is not the primary signal on them**. Put "free now / free at 14:00 / needs a badge you don't have" on the card face, and make the filter row a segmented control rather than a select. |
| **`machine-detail`** (327) | Has the right states already — reserve, login-to-reserve, training-required, unavailable, go-to-quiz, favourite | Keep the states, raise the hierarchy: the **answer** ("you can book this, next free at 14:00") above the specification. Collapse specs and accepted materials. Certification requirements stay visible (S48's rule). |
| **`machine-historique`** (663) | Full history, unpaginated | Paginate; default to the recent window; summary counts as stat tiles at the top. |
| **`calendrier`** (589, **395 inline JS**) | The booking surface | S46 moved the JS into the page's `javascripts` block unchanged; S47 owns rewriting the interaction. |
| **`events`** (**73**) | Thin list, `events.none_upcoming` | **The most underbuilt page in the app** — an events-only deployment's front door is 73 lines. Card grid with date, capacity remaining, registration state; empty state that offers the admin a "create one" path. |
| **`event-detail`** (246) | Hero image, registration panel | Registration as a one-click primary action with immediate feedback; guest flow visible without an account. |
| **`profil`** (714) | ⚠️ **Anchor links styled as tabs** (`profile-nav` → `#info`, `#badges`, `#reservations`) — every section is rendered at once, so it is one long page pretending to be four. Plus a **`disabled` Edit button**. | Real tabs or real sections with disclosure; inline edit-in-place replacing the dead button (S49, S54). |
| **`mes-reservations`** (187) | Filter tabs (current/upcoming/past) + tables | Cards on mobile, table on desktop; cancel as a one-click action with undo rather than a confirm dialog. |
| **`formation-suivi`** (1 199) | Longest template in the app | Current step foregrounded, completed steps collapsed. Progress as a single bold number. |
| **`formation-detail`** (301) | **Five "bientôt disponible" controls** | S54 decides each. |
| **`leaderboard`** | Ranked list | The one page where big bold numbers are obviously right. |
| **`badges`** | Grid | Held vs available as the primary split; "what unlocks this" on the card. |
| **Kiosk pages** | Standalone screens | Genuinely different medium — large type, no interaction, high contrast at distance. Do not fold them into the member design system; give them their own tokens from S45. |

### Component inventory (what S51 builds, used everywhere after)

**Layout** — card, stat tile (large bold number + label + delta), section with disclosure, sheet/drawer, empty state (illustration + one action), skeleton loader.
**Input** — date/time picker, segmented control, toggle, slider for durations, combobox with search, inline edit-in-place.
**Feedback** — toast, inline validation, optimistic state with rollback, progress ring.
**Navigation** — sticky page header with the primary action, breadcrumb, mobile bottom bar, floating action button (one per screen, the screen's single primary verb).
**Status** — the `admin-status-*` set from S29, extended to the public side with the same measured contrast discipline.

⚠️ **Build these against the tokens from S45, not against `style.css`.** A component that hardcodes a hex is a component that will need the S29 dark-mode treatment all over again.

---

## Phase U (continued) — read against Fabman

🔴 **S58–S66 are PROPOSED and awaiting the operator's approval. Nothing in them is authorised to be built.** The operator asked for it to be written up, not started. Approve session by session; each one below is independently revertible and independently skippable.

✅ **S67, S68 and S69 are APPROVED in principle (2026-08-01) — packages, the booking lock and no-show release, and archive-not-delete — but none is scheduled and none is authorised to start.** They exist because the feature-gap table below got four answers; see *Decisions taken 2026-08-01*. **Billing and credits are decided NO.**

🔴 **A second comparison, against [UTA-FabLab/fabapp](https://github.com/UTA-FabLab/fabapp), added S70–S73 — also proposed, also unapproved.** It is at the end of this phase and it is the more important of the two reads: **it found a model gap, not a UI gap.** We can book a machine and we cannot queue for one, and a 3D printer is queued, not booked. See *read against UTA FabApp*.

*Added 2026-08-01, from **73 screenshots of Fabman** (`Stage/Drive/Images/Fabman UI/`) — a commercial fablab-management SaaS covering roughly the same ground: equipment, members, bookings, training, RFID door/machine control. It is a fair comparison for the shapes and an unfair one for the scope: Fabman is a paid product with billing at its centre, and this app is not. **Read the two apart: adopt the shapes, decide the features.***

### The one structural difference, from which most of the others follow

**Fabman ships two applications, not one site with an admin.**

| | staff app | member portal |
|---|---|---|
| URL | `fabman.io/manage/{space}/…` | `fabman.io/members/{space}/…` |
| Nav | **6 tabs** — Overview · Members · Activity Log · Bookings · Billing · Analytics | **3 tabs** — Equipment · My bookings · Billing |
| Configuration | one **Configure ▾** menu, 7 entries | none |
| Anonymous visitors | — | — (both need a login) |

Against FabOS today: a public header of 1 link + 4 groups (~16 destinations), an **admin sidebar of 30 entries in 6 groups**, and **123 templates in `templates/site/`, 60 of them `admin-*`**. The member's own things — bookings, availability, progress — are scattered across the public site, and S54 has just removed two of them from the main nav on the correct reasoning that a personal page does not belong in a menu of things the *site* has. **That removal created the gap S65 fills.**

⚠️ **Fabman's member portal is not equivalent to our public site and must not be copy-pasted onto it.** Ours is *also* the venue's public face — anonymous visitors, opening hours, the events an events-only deployment exists to advertise, lab pages. Fabman has no such audience. The lesson to take is the **member area**, not the deletion of the public site.

### The consistency contract — six shapes, and nothing else

The reason Fabman reads as one product with a fraction of our chrome is that **every screen is one of six shapes.** This table is the deliverable of S58–S60 and the thing every later session is checked against. **A seventh shape is a bug.**

| Shape | Where it is used in Fabman | Our equivalent today |
|---|---|---|
| **1 · List** | members, equipment, packages, courses, bookings, invoices, activity log | 20-odd hand-rolled tables and grids |
| **2 · Detail** | a member, a machine, a course, an invoice, a package | **7 `*-detail` templates, no two alike** |
| **3 · Object settings** | package, space, equipment — left sub-nav inside the object, counts on the entries | none — settings are top-level admin screens |
| **4 · Form card** | add equipment, add member, book, create invoice | ~25 forms, each its own layout |
| **5 · Wizard step** | set up courses, connect a bridge, add packages — always with *"Skip this step for now"* | `admin-wizard` / `admin-setup` only |
| **6 · Calendar** | overview day-timeline, 7-day equipment grid | `calendrier` (589 lines) + 3 overlapping stylesheets |

⚠️ **This contract only holds if it is enforced when the next feature lands.** Phase D adds around ten LMS screens. If they are built before S58/S59, they become ten more hand-rolled pages and this section is dead on arrival. **That is the argument for taking S58 and S59 before Phase D, the same argument S45/S46 won.**

### The details worth stealing, itemised

Small, cheap, and the actual source of the "feels finished" difference:

- **Every settings control is a sentence with inputs in it.** *"Allow bookings from `[2]` hours up to `[7]` days in advance."* · *"It must be cancelled `[0]` `[days]` in advance."* · *"Don't charge anything if the session is less than `[0]` seconds long."* Not a label above a box.
- **A consequence line under the control, computed from what is typed.** *"Booking this package today would cost €9 today and €9 on every 30th of the month."* · *"The next invoice will get the number 1001."* · *"Creating an invoice today would set its due date to 07/30/2026."* The form tells you what it is about to do.
- **Optional fields do not exist until asked for.** *"Add a note"* · *"Add metadata"* · *"Add a comment"* · *"Set end date"* are links; the field appears on click and gains a *"Hide"* affordance. A form opens at its common case.
- **A checkbox reveals its own sub-fields, indented underneath it.** Unchecked, they are not there at all.
- **Repetition collapses.** Opening hours show seven weekday rows; tick *"Different per weekday?"* off and they become one **"Monday – Friday"** row plus Saturday and Sunday. Same data, one third of the screen.
- **Status is a small uppercase strip under the title**, not a badge floating in the corner: `LASER CUTTER – AVAILABLE`, `ACTIVE MEMBER – joined a minute ago`, `ACCOUNT OWNER`.
- **`Edit` is a quiet link in the card's top-right**, never a button competing with the page's real action.
- **A provenance footer on every record**: *"Edited a few seconds ago by CCk CcK (view change log)"*.
- **Empty states offer the widening action**: *"No activity found for this date range. **Show results for all dates**"* — never a bare "no results".
- **Result counts are sentences**: *"1 result: 1 member, €20.00 (excl. taxes)"*.
- **Applied filters become removable chips** (`Owners ✕`) sitting under the filter row.
- **Dates are relative where relative is clearer**: *"Tomorrow at 9:00 AM"*, *"due today"*.
- **`?` tooltips explain the rule, not the field.** The one on *"Ignore member booking restrictions"* enumerates every layer it bypasses, in a paragraph.
- **Duplication is a first-class action**: *"Create a copy of this equipment"*, *"Copy all settings from…"* on create.
- **Archive, not delete**, as the default destructive action — with a split-button caret hiding the real delete.

⚠️ **Not everything there is good, and two things are actively worse than ours.** Fabman's booking grid renders **all 24 hours** including the dead ones, so a member scrolls past midnight-to-8am to find the morning; and its equipment "list" is an unsorted flat table with no card face and no availability until you open the item. **Our `machines` card grid is already better than theirs.** Copy the shapes, not the page.

---

### The feature gap — what Fabman has that we do not

Comparing entity for entity. `src/Entity/` has 39 entities and **none** of them is a plan, a charge, a note or an audit row. **Each line below needs a yes/no from the operator before it can become a session** — several are deliberate absences, not oversights.

| Missing | What Fabman does with it | Recommendation |
|---|---|---|
| **Packages / membership plans** | A named bundle of *equipment permissions* (what, when — 24/7, opening hours, or custom) + *credits* + a price, assigned per member with a start date. | ✅ **DECIDED 2026-08-01 — build it, and badges stay.** See S67 and the decision note below. |
| **Billing** — charges, invoices, payments, taxes, dunning, Stripe/SEPA | An entire tab, plus per-member charges and an invoice PDF | ❌ **DECIDED 2026-08-01 — not now.** "Not our goal, we'll do that later." It is a phase, not a session, and S67 must be built so it does not block one. |
| **Credits** | Prepaid balance, scoped to equipment or category, with an expiry | ❌ **DECIDED — deferred with billing.** ⚠️ A package therefore has **no price field** for now; see S67. |
| **Notes + metadata on every object** | A staff-private rich-text note and a free JSON blob on member, equipment, course, package, booking, charge | 🟢 **Recommend building** — S63. Cheap, and the thing every operator improvises in a spreadsheet otherwise. |
| **Change log** | *"Edited … by … (view change log)"* on every record | 🟢 **Recommend building** — S63, same session. |
| **CSV export on every list** | One cloud icon in the table header | 🟢 **Recommend building** — S59. ⚠️ S54 deleted a fake Export button from `admin-reservations`; this is what earns it back. **Do not re-add the icon before the download works.** |
| **Manually log a machine session** (*"Track activity"*) | Staff record a usage session by hand: start, optional end, member, note | 🟢 **Recommend building** — S62. We have `LogUtilisation` and an RFID box that will miss sessions; today there is no way to correct that. |
| **Invite a member to create an account** | *"X has not yet been invited to create a user — Send invitation email"*, and *"Send password reset email"* from the admin side | 🟢 **Recommend building** — S62, and it partly **pre-empts S56**: admin-triggered reset is the same mail with a different entry point. |
| **Lock a member account** | One link on the member card | 🟢 Cheap, same session. |
| **Export all member data** | One link on the member card | 🟢 Same session — and it is **half of S57**, done for free. |
| **Holidays & exceptions** on opening hours | Dated closures with a title, and a choice: *"affects only opening hours"* vs *"affects everyone — only admins can use equipment"* | 🟢 **Recommend building** — S66. `OpeningHour` exists; this is a second table and a check in `OpeningHoursProvider`. A lab that never closes for Christmas does not exist. |
| **Take equipment out of service** | *"Disable equipment (maintenance, repair, …)"* on the detail page | 🟢 **Already scoped as S49's** unbuilt surface — S62 builds it. |
| **Booking granularity, lock window, no-show release** | *"Prevent members from cancelling … `[24]` hours before"* · *"Allow others to use booked equipment if the member hasn't shown up for `[30]` minutes"* | ✅ **DECIDED 2026-08-01 — copy both, as settings.** See S68. Granularity was not asked for and is not included. |
| **Equipment categories as a first-class managed list** | A tiny CRUD screen; categories then carry permissions and discounts wholesale | 🟢 We have `MachineCategory` already — it just has no admin screen. Fold into S59 as a list-pattern proof. |
| **Archive vs delete** | Archive is the button; delete hides behind a caret | ✅ **DECIDED 2026-08-01 — build it.** See S69. Still wants S58/S59 first, because it is a change to ~15 list and detail screens. |
| **Copy an object** | *"Create a copy of this equipment / package"* | 🟢 Cheap per entity; add opportunistically, not as a session. |
| **Two-factor authentication** | Member-facing, in the portal's security screen | 🔴 S54 deleted a dead 2FA control from `profil` for good reason. Leave deleted until someone asks. |

#### Decisions taken 2026-08-01 — four of them

**1 · Packages: yes. Badges stay. They are two questions, not one.** The operator's sentence is the specification: *"a user might be allowed 24/7 access but still need training on a machine before using it if the machine requires it."*

| | answers | scope | set by |
|---|---|---|---|
| **Package** | *what may I reach, and when* | a set of equipment or categories × a time window (24/7 · opening hours · custom) | the operator, assigned per member |
| **Badge** | *am I qualified for this machine* | one machine, independent of any package | the machine's own requirement |

**Both must pass, plus the existing quotas.** A 24/7 package on a laser cutter that requires the laser badge still refuses an untrained member — and refuses them for the *training* reason, not the access reason. Fabman draws the identical line and says so in its own override tooltip: *"the member still needs to be allowed to turn on the machine, eg., have training and a valid package for the time period."*

⚠️ **This makes four permission layers, and the honest question is whether it should stay at three.** Today: badges (certification) · booking policies (quotas) · access passes (`Accès exceptionnels`, the `/staff` pass desk). **An access pass is arguably just a package with an end date** — same shape, temporary. Decide in S67 whether it becomes one, because merging them later means migrating live rows.

**2 · Billing: no, not now.** Therefore **a package has no price and no billing cycle** — it is a permission grant, nothing else. ⚠️ **Do not add a nullable `price` "for later".** An unused money column is the thing that makes the eventual billing phase harder, not easier: it will have accumulated nulls, no currency, no tax rule and no history, and the first real invoice will have to migrate all of it.

**3 · Booking lock window and no-show release: copy both.** As settings, in the same sentence form S60 builds.

**4 · Archive rather than delete.** Archive is the visible action; real deletion moves behind a caret.

---

⚠️ **Three things we have that Fabman does not, and they must survive all of this**: the **feature registry** (their app is one fixed shape; ours becomes an events-only or lending-only install), **portals** with their own accent and front door, and the **public, anonymous-readable venue site**. Any pattern imported here is imported *through* those three, not over them.

---

### S58 · One detail page, seven times over

**Why.** There are **seven `*-detail` templates** — `machine`, `place`, `event`, `formation`, `badge`, `lab-page`, `admin-utilisateur` — and no two are laid out alike, plus `machine-historique` (663) and `profil` (714) which are detail pages that grew. Fabman renders a member, a machine, a course and an invoice through visibly the same card. This is the S29 "six skeletons" finding again, on the pages a member actually reads.

**Scope.** One include — working name `site/_detail_page.html.twig` — owning: breadcrumb (`Parent › Thing › Action`); an identity card carrying title, the small uppercase status strip, a quiet `Edit` link top-right, label-above-value micro-rows, verb links at the card foot, and the provenance footer; a right rail for related objects, each with its own action button; and related-list sections below with the action in the section header and a **sentence** empty state inside.

Migrate **three** pages as proof — recommend `machine-detail`, `admin-utilisateur-detail` and `event-detail`, because they are the three most different from each other today.

⚠️ **Status strips are `.admin-status-*` from S29, promoted to the public side by S45 — not new colours.** Every one goes through the contrast harness in both themes before it ships.
⚠️ **The card's verb links are affordances, so they go through `can_reach()` (S49), never a hand-written role.**
⚠️ **`strict_variables: true`** — every slot the include reads needs a `|default()`, or the first page that omits one 500s. S47 has already been bitten by exactly this on `place-detail`.
⚠️ **Titles and status words are operator vocabulary (S31) and translated ×5 locales.** Key parity is 627 and verified; a new key is five new rows, not one.

**Verify.** The three pages rendered with `app:render` before and after, diffed with the S46 normalisations; contrast measured in both themes; and a class audit finding nothing newly undefined.

---

### S59 · One list page, and the CSV that was promised

**Why.** Every list in the app was built on its own — filters sometimes above, sometimes in a bar, sometimes absent; empty states that say nothing; and S54 found a fake *Export* button whose JS was three `alert()` stubs. Fabman's lists are one component: search + two or three filters, always visible, never behind a disclosure.

**Scope.** One include — working name `site/_list_page.html.twig` — with: an always-visible search-and-filter row; **applied filters as removable chips**; a **result-count sentence** ("2 résultats · 1 membre"); a **CSV download** control; and an empty state that **offers the widening action** ("aucune activité sur cette période — *voir toutes les dates*"). Filter selects become type-to-search comboboxes with grouped options where the list is long.

Adopt on `admin-reservations`, `admin-utilisateurs`, `admin-usage-logs` and `mes-reservations`. Ship `MachineCategory`'s missing admin screen on the same include as the proof that it is reusable.

⚠️ **Build the CSV before showing the icon.** The whole point of S54 was that an affordance that does nothing is worse than an absent one. And a CSV of `admin-usage-logs` is **personal data leaving the building** — it must be behind the same `access_control` line as the page, and it must not include badge UIDs (S38's finding).
⚠️ **Comboboxes need JS, which does not load.** Native `<select>` with `<optgroup>` until the AssetMapper decision (see S51); the combobox is a follow-up, not a blocker.
⚠️ **`admin-usage-logs` and `admin-reservations` filters are the two that are already right** (S47's inventory) — copy their defaults into the component, do not overwrite them.

**Verify.** Same list, same rows, before and after. Every filter combination still reachable by URL. A CSV that opens in a spreadsheet with correct accents (BOM) and `Europe/Paris` timestamps.

---

### S60 · Settings that read as sentences, and forms that open small

**Why.** Our settings screens are label-over-input grids where Fabman's are prose with inputs embedded, and ours show every optional field at once where Fabman's reveal them on request. The difference is not decoration: *"Allow bookings from `[2]` hours up to `[7]` days in advance"* is self-documenting, and `Min. notice (h)` + `Horizon (j)` is not.

**Scope.** Three patterns, applied to `admin-booking-policies`, `admin-opening-hours`, `admin-settings` and the booking forms:

1. **Sentence controls** — the rule written out with the inputs inline.
2. **Reveal links** for optional fields — *"Ajouter une note"*, *"Définir une date de fin"* — with a *"Masquer"* once open. Native `<details>`, no JS (S48's constraint).
3. **Consequence lines** computed under the control — *"Une réservation demain à 08:00 serait refusée : le préavis minimum est de 2 h"*. This is the S47 refusal-order warning turned into something the operator can see **while setting the rule** instead of after a member hits it.

Also: collapse the seven weekday rows in `admin-opening-hours` behind a *"Horaires différents selon les jours ?"* toggle, defaulting to the collapsed "Lundi – Vendredi" form.

⚠️ **A consequence line is computed by the real service or it is a lie.** `BookingPolicyService` and `OpeningHoursProvider`, never a second copy of the arithmetic — the S47 `NextFreeSlotService` rule.
⚠️ **Sentence layouts break first at 320px and in the longest locale.** Check both; a sentence that wraps mid-input is worse than a label.
⚠️ **Timezone on the read as well as the write** — S47 shipped a button advertising 06:00 for an 08:00 slot because `|date` rendered in the app default (UTC). A consequence line is exactly the same hazard.

**Verify.** Every rule still saves and loads identically — the forms change shape, not schema. Screenshot both themes at mobile and desktop presets.

---

### S61 · Book in three clicks, with the closed hours drawn

**Why.** S47 got `machine-detail` to advertise the next free slot and got both calendars to fill the times from the clicked slot. What is left is the shape of the surface itself, and Fabman's member path is: equipment row shows `AVAILABLE` + a **Book** button → a 7-day column grid → click a slot → a confirm card reading *"Epilog 48 / Friday, 7/31/2026 / 3:00 PM – 4:00 PM (1 hour)"* with an `Edit` link and an optional comment → **Confirm**. **Zero fields typed.**

**Scope.** The confirm step as a page of its own that states the booking as a sentence and offers exactly two actions (Edit, Confirm); `Book` promoted onto the `machines` card face; and — the part that matters most — **closed and already-taken hours drawn on the grid** rather than merely refused on submit.

⚠️ **The 24-hour grid is the thing Fabman gets wrong.** Render the opening window plus a margin, with an affordance to expand — a member should not scroll past 3am.
⚠️ **Availability on the card face is the query problem S47 deliberately refused** — 11 machines, unpaginated, one query each. It needs the single batched "upcoming reservations grouped by resource" query, and that is Phase H's **S41**. **S61 is blocked on S41 for the card face**; the rest of it is not.
⚠️ **Fewer clicks, same refusals.** Every layer still runs in `ReservationService::book()`. A confirm page that posts to anything else is a hole, not a shortcut.
⚠️ **A slot suggested is not a slot held.** Between the grid and Confirm someone else can take it; the refusal must name that, not blame the member.

**Verify.** Re-count clicks against the S47 baseline table for all four flows. Every refusal branch still reachable and still explained.

---

### S62 · The staff surfaces, and the honest override

**Why.** S49 shipped the *mechanism* (`can_reach()`) and none of the *surfaces*: nobody has built inline staff editing, out-of-service, or roster check-in. Fabman shows what those look like — and shows the one thing we have no equivalent of at all: an **explicit, labelled override**. Their booking form has *"Ignore member booking restrictions"* whose tooltip enumerates every layer it bypasses and then says what it still cannot do ("the member still needs to be allowed to turn on the machine").

**Scope.** Build S49's table, plus the member-record actions Fabman has and we lack:

- **Book on behalf of a member**, and a `Staff only` reservation that holds a machine with no member attached.
- **"Ignore member booking restrictions"** — staff-only, logged, with the enumerating tooltip.
- **Take equipment out of service** from `machine-detail` — with a reason and an expected return, shown to members as a reason and not as a silence.
- **Track activity** — record a machine session by hand (start, optional end, member, note) for the sessions the RFID box misses. `LogUtilisation` already exists.
- On the member record: **send an invitation** to create an account, **send a password reset**, **lock the account**, **export all their data**.

⚠️ **An override is a security decision and must be audited, not just gated.** It writes who overrode what and why; without that it is a back door with a nice label. This is the strongest single argument for taking **S63 first**.
⚠️ **It must not be able to override the things that are physical safety.** Certification for a laser cutter is not a scheduling preference. Fabman draws exactly this line and says so in the tooltip — copy the line and the sentence.
⚠️ **`ROLE_ADMIN` does not imply `ROLE_STAFF`.** Every one of these goes through `can_reach()` against its own route's `access_control` (S49), and this install still has **no staff-but-not-admin account** to test with. **Create one before this session, not during it.**
⚠️ **"Send password reset" from the admin side is S56's mail with a different trigger.** Build S56 first or build the token here and let S56 add the public entry point — but do not build the token twice.
⚠️ **"Export all member data" is half of S57** and touches the same erase-vs-anonymise question. Read S57 before writing the export.

**Verify.** Both directions, as S49 did: the affordance absent for a member *and* the endpoint refusing. Plus: an override leaves an audit row, and a certification refusal survives the override.

---

### S63 · Notes, metadata, and a change log

**Why.** Fabman puts a staff-private note and a free-form JSON blob on every object, and a *"Edited … by … (view change log)"* footer on every record. We have neither, so every operator improvisation — why this machine is really out of service, which locker number this person has — lives in a spreadsheet or in someone's head. And S62's override is not safe to ship without the log.

**Scope.** A polymorphic note (author, timestamp, body, target) and an optional metadata blob, exposed on member, machine, place, event, reservation and loanable item; plus a change log written on the mutations that matter, and a provenance footer rendered by S58's detail card.

⚠️ **New ORM entities — the migration ships and runs *before* the code.** The expand rule; see `feedback_fabos_migration_hazard`.
⚠️ **Notes are staff-private and will contain things about people.** They are personal data: gated on the route's own rule, excluded from the public API, included in S57's export and deletion decision, and **never** rendered on a public detail page. The S38 lesson — a field that looks like a label is not necessarily one.
⚠️ **A change log grows without limit.** Decide retention now, not when the table is 40M rows: what is kept, for how long, and whether it is trimmed. Phase H's S41 is the session that will otherwise have to.
⚠️ **Do not log the values of secret fields.** `machineToken`, password hashes, anything from `.env` — a diff-based logger will happily record all three.

**Verify.** A note round-trips and is invisible to a member. A change to a machine appears in its log with the right author. The public API and every public template are re-grepped for both new fields, the way S48 re-grepped for `machineToken`.

---

### S64 · Finish the thing you just created

**Why.** In Fabman, saving a machine with *"requires training"* ticked does not return you to a list — it routes you to *"OK, so members need a training course for Epilog 48. Do you want to create one right away?"*, then to *"Connect a bridge"*, then to the finished detail page. Every step offers *"Skip this step for now — you can always return here later."* Our `admin-machine-new` saves and drops you on `admin-machines`, with the badge and the RFID reader left as things you have to know to go and do.

**Scope.** A `?new=1` step chain after creation: machine → the badge it requires (create or attach) → the RFID reader → the detail page. Same for event → registrations settings, and formation → sections. Plus the dismissible **onboarding checklist** on `/admin` that ticks its own items off (Fabman's *"Ready to dive in?"* panel), reusing `FirstRun` / `SetupHealth`, which already know most of the answers.

⚠️ **Every step is skippable and the object is already saved before the chain starts.** A wizard that can lose work is worse than no wizard. Fabman's steps are post-save navigation, not a multi-step form.
⚠️ **The chain has to respect the feature gates.** Offering "create the badge" on an install with `badges` off is offering a 404 — the exact class of bug S37 and S55 each fixed once.
⚠️ **`admin-wizard` and `admin-setup` already exist.** Read them first; this is an extension of that idea to per-object creation, and a third setup mechanism would be its own problem.
⚠️ **Dismissal must persist per operator**, or the panel comes back every login and becomes noise. `HomepageUserPreference` is the nearest existing pattern.

**Verify.** Create a machine requiring a badge on a fresh install and reach a fully configured machine without visiting a list. Do it again with `badges` off and reach the detail page with no dead step. Skip every step and land somewhere sensible.

---

### S65 · A member area, not a member page

**Why.** S54 removed *Mes réservations* and *Mes disponibilités* from the main nav — correctly, because a menu of what the *site* has is the wrong home for one person's own things — and left them reachable only from a link inside `/profil`. Fabman's answer to the same problem is a **portal**: three entries, one of which is "my bookings". `profil` is 714 lines with anchor links styled as tabs, which is not an area; it is a long page pretending to be four.

**Scope.** A small account area — recommend `/mon-compte` — with a handful of entries: *Mes réservations*, *Mes disponibilités* (only for people who are bookable), *Ma progression*, *Mon compte*. `profil` becomes its landing screen and sheds the sections that move out. The identity card, next booking and badges held stay foregrounded; history goes behind S48's disclosure.

⚠️ **This is not Fabman's member portal and must not swallow the public site.** Anonymous visitors, opening hours, events and lab pages stay exactly where they are. This is an area *inside* the public site for the signed-in member's own records.
⚠️ **The bookable-person case is the one S54 knowingly made worse** and wrote down: the old code gave *Mes disponibilités* a nav link specifically for `isBookable()` people, and that reasoning is now only in the roadmap. **This session is where it comes back** — as an entry in the account area, not in the site menu.
⚠️ **`profil` server-renders `data-theme-preference` on `<html>`** via S46's `html_attrs` hook. Whatever becomes the landing screen inherits that, or the signed-in theme silently stops applying.
⚠️ **Every route moved is a URL someone has bookmarked or mailed.** Redirect the old ones; do not just move them.

**Verify.** Every destination reachable in ≤2 clicks from anywhere signed in. A bookable person finds their availability page without being told where it is. Old URLs still resolve.

---

### S66 · Holidays, exceptions, and the closures nobody modelled

**Why.** `OpeningHour` describes a normal week and nothing else. Every real venue closes for holidays, and the current answer is to edit the weekly hours and remember to change them back. Fabman models it as dated exceptions with a title and — the part that is actually thoughtful — a **scope**: *"affects only opening hours: 24/7 packages and packages with custom times work as usual"* versus *"affects everyone! Only admins can use equipment during this time."*

**Scope.** A dated exception (title, from, until, all-day or timed, scope) read by `OpeningHoursProvider`, so that it flows for free into `BookingPolicyService`, the calendars, `NextFreeSlotService` and the homepage's opening-hours block. Past entries hidden behind *"Voir les entrées passées"*.

⚠️ **New ORM entity — migration first**, and the read path is the one that matters: a closure that the booking checker does not see is a closure that lets members book Christmas Day.
⚠️ **The two scopes are two different questions, and the honest one for us is different from Fabman's.** They distinguish by package; we have no packages (see the gap table). Our equivalent is almost certainly "closed to members, open to staff" — **decide the wording before the schema**, because it becomes a column.
⚠️ **`Europe/Paris`, pinned on the read and the write.** An all-day closure stored as UTC midnight closes the wrong day twice a year.
⚠️ **An exception must not silently cancel bookings that already exist inside it.** Decide: refuse the exception, or list the affected bookings and make the operator act. Do not cascade — the S10 lesson.

**Verify.** Set a closure; `/api/opening-hours`, the calendar grid, the next-free-slot button and the homepage block all agree. Booking inside it is refused with a reason naming the closure. Staff can still book if the scope says so.

---

### S67 · Packages — what you may reach, and when

**Why.** Decided 2026-08-01. Access is currently expressed three ways and none of them answers *"this person may use the workshop 24/7, that one only during opening hours."* Badges answer qualification, quotas answer volume, access passes answer exceptions — nothing answers the ordinary standing grant.

**Scope.** A `package`: a name, a description, and a set of grants. Each grant is *(equipment or category) × (24/7 · opening hours · custom window) × (may book, yes/no)*. Assigned to a member with a start date and an optional end date. Read by `BookingPolicyService` on booking and by the RFID authorization endpoint on machine start.

**No price. No billing cycle. No credits.** Decision 2 above.

⚠️ **The check is an AND with the badge, and the refusals must be distinguishable.** "You may not be here at this hour" and "you are not trained on this machine" are different problems with different fixes, and a member told the wrong one goes to the wrong person. This is S47's 403-vs-409 rule again: the wording has to offer the fix.

⚠️ **Refusal order matters, as it did in S47.** Package window before badge, or an untrained member booking at 3am is told to go and get trained for a slot they could never have had.

⚠️ **Decide access-passes-versus-packages before the migration, not after.** If a pass is a package with an end date, say so now and migrate the rows once. See the decision note above.

⚠️ **The RFID endpoint is the half everyone forgets, and it is the half that opens doors.** `ReservationService::book()` is not the only gate — `RfidMachineController` authorises a physical machine start. A package enforced only in the web booking flow is not enforced. ⚠️ **And that endpoint currently fails open** when `FABOS_RFID_API_TOKEN` is unset, which it still is on the live box (S48). **Fix that first or S67 ships a permission system anyone can bypass with curl.**

⚠️ **New ORM entities — migration first, then code.** Expand rule.

⚠️ **A member with no package.** Decide explicitly: does that mean no access, or unrestricted access? Getting it wrong in either direction is a live incident — one locks everyone out on deploy day, the other makes the whole feature decorative. **Recommend: the existing behaviour is preserved until an operator creates the first package,** so the migration is inert until used.

⚠️ **Feature-gate it.** A lending-only or events-only install has no use for equipment access windows; this is a registry feature like the rest.

**Verify.** A member with an opening-hours package is refused at 3am by the web form *and* by the RFID endpoint. A member with 24/7 and no laser badge is refused on the laser and told it is the training. A member with 24/7 and the badge is allowed. An install with no packages behaves exactly as it does today.

---

### S68 · Lock window and no-show release

**Why.** Decided 2026-08-01, copied from Fabman's booking settings. Two rules we have no equivalent of: a member cancelling a slot ten minutes before it starts wastes it, and a member who books and never turns up blocks a machine for everyone else.

**Scope.** Two settings, written as sentences (S60's form):

- *"Empêcher les membres d'annuler ou de modifier une réservation `[24]` heures avant son début."*
- *"Autoriser d'autres personnes à utiliser un équipement réservé si la personne qui l'a réservé ne s'est pas présentée depuis `[30]` minutes."*

Both live in `BookingPolicyService` beside the existing min-notice, horizon and quota rules.

✅ **COUNTED 2026-08-02: there is 1 `RFID_READER` row for 11 machines.** So the decision is settled — **the no-show release is per-resource and off by default**, offered only where a reader exists, and the lock window (which needs no hardware) ships independently of it. Do not re-open this on an assumption; re-count.

⚠️ **The no-show release needs a "showed up" signal, and we only have one on machines with a reader.** The signal is the RFID work-session start (`LogUtilisation` / `MachineUsageHistory`). **On a machine with no reader there is no signal at all, so the rule would release every single booking after 30 minutes.** Therefore: the setting is *per resource and only offered where a reader exists*, or it is globally off by default and the operator is told which machines it can apply to. **Check the reader coverage across the 11 machines before choosing** — do not assume it is total.

⚠️ **Releasing is not cancelling.** The original booking must stay visible and attributed, or the member is told they never booked and the lab loses the no-show record it wanted. Release the *slot*; keep the *row*, marked.

⚠️ **The lock window and staff are different questions.** Staff must still be able to move or cancel a locked booking — that is S62's override, and it must be audited (S63).

⚠️ **`Europe/Paris` on read and write.** Both rules are clock arithmetic near a boundary; this has bitten twice already.

⚠️ **No cancellation-fee logic.** That is billing, and billing is decided no.

**Verify.** A member cancelling inside the lock window is refused with a message naming the deadline; outside it, allowed. A booking on a reader-equipped machine with no session start releases after the configured minutes and the row survives, marked. A machine with no reader never releases anything. Staff can still move a locked booking, and it lands in the change log.

---

### S69 · Archive, not delete

**Why.** Decided 2026-08-01. Deleting a machine, a course or a member destroys history that has nothing to do with wanting the thing off the list. Fabman's answer is that **Archive is the button** and real deletion hides behind a caret.

**Scope.** An archived flag on the entities that are listed and retired rather than genuinely removed — machine, place, loanable item, material, course, badge, package, event. Archive as the primary action; delete demoted behind a disclosure and kept only where it is genuinely safe. An *"Afficher les archivés"* control on the affected lists.

⚠️ **Every list query needs the filter, and the one that is forgotten is the bug.** An archived machine reappearing on `/machines` is the obvious failure; an archived machine still counted in a dashboard statistic is the one nobody notices. **Grep the repositories, not the templates.**

⚠️ **Uniqueness still applies to archived rows.** Archiving `printer-01` does not free its `machineToken` — the next machine created with that name will collide. Decide per field whether archiving releases the constraint.

⚠️ **Archived must not mean invisible to the things that reference it.** A past reservation on an archived machine still has to render the machine's name, and a badge awarded by an archived course is still held. Archive hides it from *choosing*, never from *reading*.

⚠️ **This is a change to ~15 admin screens, which is why it comes after S58 and S59.** Done before them it is fifteen hand-edits; done after, it is one change to two includes.

⚠️ **Not the same thing as S57's account deletion.** A person is not an inventory item, and the erase-vs-anonymise decision there is per table and still unmade. Keep them apart.

**Verify.** An archived machine leaves every list, keeps rendering in existing reservations, and stops being offered anywhere new. Toggle the "show archived" control and it comes back. Statistics agree with the lists.

---

---

## Phase U (continued) — read against UTA FabApp

🔴 **PROPOSED. S70–S73 are not approved and not authorised to be built.**

*Added 2026-08-01 from **[UTA-FabLab/fabapp](https://github.com/UTA-FabLab/fabapp)** — the University of Texas at Arlington FabLab's own operational system, open source, PHP, v1.0 in production, 22 stars.*

**Operator context, and it explains the design: it is a full fablab like ours, but the bulk of the usage is the printers — and the operator has been there.** So this is not a narrow single-purpose tool whose ideas need translating; it is the same kind of place as ours, further along, with the parts that a heavy-traffic queue actually needs already built. ⚠️ **They also run physical hardware we do not: numbered storage boxes, and small thermal printers that print a paper queue ticket.** That is why `wait_ticket.php` exists, and it is a real dependency in S70 below — not a UI detail.

⚠️ **This is not a codebase to copy from and this section does not propose copying any of it.** Raw PHP with an `admin/sub/*.php` and `pages/sub/*.php` AJAX layer, no framework, no tests visible. **Our Symfony/Twig architecture is the better one and stays.** The value here is entirely the **domain model** — what a busy university makerspace actually turned out to need after running for real — and on that, they are ahead of both us and Fabman.

### The finding that matters more than anything in the Fabman section

**Booking is the wrong model for a 3D printer, and booking is all FabOS has.**

| | the interaction | the right model |
|---|---|---|
| Laser cutter, CNC, sewing machine, a room | you stand at it, or occupy it, for a known time | **book a slot** — ours and Fabman's model |
| 3D printer, and any long unattended job | you submit an 8-hour job and *leave*, then come back for the part | **join a queue** — UTA's model |

**Both are needed in the same lab, and UTA is a full fablab that needs both too.** The point is not that printers are a different kind of building; it is that **the interaction is different and only one of the two is modelled here.** `Reservation` is polymorphic across machines, places and people, and every single one of them is a time slot. **Nobody reserves a printer for 8 hours in a lab with three printers and forty students**, so what happens instead is a paper list, a whiteboard, or a queue in someone's head — and the app does not know about any of them.

UTA models it properly: `Wait_queue` with `Operator`, a **device *or* a device group** (queue for "any FDM printer", not printer #3), `estTime`, `last_contact`, and a `valid` flag; `calculateWaitTimes()` estimating from the jobs actually running; contact details captured **per ticket** (email, phone, carrier) so the member can leave the building; a `now_serving` screen for staff; and `transferFromWaitQueue()` when they are called.

⚠️ **And it does not stop at the queue — the loop only closes because of the next two pieces.** The job finishes while the member is elsewhere, so the part has to go *somewhere*: `StorageBox` / `ObjBox`, a numbered physical bin, with `pickup.php`. **Queue → print → bin → notify → pickup is one workflow, and building only the first third of it leaves parts on a shelf with names on masking tape.** That is the honest scope warning for S70.

### What they have that we do not

`class/` is the whole domain: `Accounts · AuthRecipients · Devices · IndividualsCertificates · Materials · Notifications · ObjBox · OfflineTrans · Purpose · Service · Status · StorageBox · TrainingModule · Transactions · Users · Wait_queue · wait`.

| Missing | What it does | Recommendation |
|---|---|---|
| **Wait queue** | Queue for a device *or a device group*, estimated wait, notify by email/SMS, staff "now serving" | 🟢 **Strongly recommend — S70.** The one genuine model gap in this app. ⚠️ Probably phase-sized, not session-sized. |
| **Finished-object storage** | Numbered bins, object → bin, pickup, notification | 🟢 **Recommend — S71**, and it is **not optional if S70 ships**; see above. |
| **Member-reported faults** | `Service` — a member opens a ticket on a device, with history and a log; plus offline tickets | 🟢 **Recommend — S72.** We have `MaintenanceTask`, authored by admins. **The person who finds a broken machine is the member standing at it**, and today they have no way to say so. |
| **Purpose on a usage session** | A short list — why is this person using this machine (a course, research, personal) | 🟢 **Recommend — S73.** Cheap, and it is the reporting axis a *university* lab is actually asked for: how much laser time went to which course. We have `Institution`; we have nothing per session. |
| **Certificate revocation + expiry** | `training_revoke.php`, `IndividualsCertificates`; Fabman independently has *"training is only valid for `[12]` months"* | 🟡 **Check first, then decide.** We award `UserBadge`/`UtilisateurBadge` — verify whether anything can take one back or expire it before scoping. **Two independent products both have this; that is a signal.** |
| **Live "in use" state** | `isInUse.php`, `getDot.php` — a dot showing whether a machine is running right now | 🟢 Fold into **S61**'s card face. We already have the signal (`LogUtilisation` / RFID work sessions) and do not show it. |
| **Onboarding / offboarding** | Explicit member lifecycle flows, not just create/delete | 🟡 Partly **S62** (invite, lock) and **S57** (deletion). Worth naming the two ends explicitly. |
| **Consumables: cart, stock decrement, sheet goods with variants** | `add_cart` / `Materials` / `inventory_quantity` / `sheet_goods` + `si_getVariants` | 🟡 **Split it from money.** ⚠️ Their version is wired to `Transactions` and `Accounts`, which is **billing, decided no**. But *"this member took 400mm of 3mm ply, decrement the stock"* is stock control, not accounting, and we already have `Material`. Recommend the stock half, explicitly without the ledger. |
| **`Transactions` / `Accounts` / `OfflineTrans`** | A member money ledger | ❌ **Billing. Decided no 2026-08-01.** Consistent with the Fabman answer. |
| **OctoPuppet / JuiceBox integrations** | OctoPrint control, physical device power | 🟡 Out of scope for Phase U, but note it: **their printers are software-controlled, ours are RFID-gated.** A queue that can *see the job finish* is a much better queue than one that cannot. |

---

### Direction taken 2026-08-01, second round — read before scoping S61, S67 or S70

Seven decisions from the operator, in their own framing. **Several change sessions that were already written; where they do, the session below says so.**

#### 0 · The principle: anything the member can do, the staff will not have to

*"User interactions are important… Anything the user can do the staff won't have to!"*

**This is now the tie-breaker for every affordance question in the phase.** S49's table asked "who edits what, where"; this answers the ambiguous rows in favour of the member. Concretely it decides: members report faults (S72), members claim their own place in a queue (below), members pick up their own parts (S71), and — see decision 6 — members may do maintenance.

⚠️ **It does not weaken any server-side check.** "The member can do it" means the affordance is theirs; the three permission layers still run. Self-service and unchecked are different words.

#### 1 · Categories become a first-class, app-wide concept — new session **S74**, and it is a prerequisite for S70

*"I do like their categories so you queue for a laser or a specific laser (ex: the only one big enough). It would be useful app wide this whole category thing."*

`MachineCategory` exists today and is used for **filtering a list, and nothing else**. It becomes the unit you can *act on*: queue for "a laser", browse by category, and — decision 2 — book one.

#### 2 · You book the category, not the machine, unless you need a specific one

*"If two machines only have limited space you book that instead of the specific machine."*

The common case is that any machine in the category will do; picking a specific one is the **exception**, for when only that one is big enough. So booking a category means booking *a* member of it, and the specific-machine path stays for the exception.

⚠️ **The hard question is when the pool resolves to a machine, and it must be answered before any code.** Two options, and they produce different calendars:

| | resolve **at booking** | resolve **at start** |
|---|---|---|
| The member is told | "you have printer #2 at 14:00" | "you have *a* printer at 14:00" |
| Utilisation | worse — a gap can't be refilled | better — whoever is free wins |
| The calendar shows | a booking on a machine row | a booking on a *category* row |
| Failure mode | fragmentation | the member arrives and is sent to a machine they didn't expect |

**Recommend resolve-at-booking with a re-assign on the day**, because it is the only one where the calendar stays readable and the member knows where to stand — but this is the operator's call and it is load-bearing.

⚠️ **`Reservation` currently points at a resource.** Pointing at a category is a schema change, and it is the same polymorphism question S8–S10 already answered once for machines/places/people. **Re-read those before adding a fourth shape.**

#### 3 · The calendar stays a calendar — this constrains S70

*"I do believe our calendar should only have that, the calendar with current infos and handles reservation as it does."*

`/calendrier` keeps its job: current information and reservations. **The queue does not go on the calendar grid.** A queue has no start time, so drawing it there would be inventing one.

⚠️ **This kills the most obvious S70 design** — "show the queue as pending blocks on the grid". Queue lives on the machine page, on a category page, and on a staff "now serving" screen. Good: it also means S70 does not have to touch the 589-line calendar or its three stylesheets.

#### 4 · The machines page leads with categories, and status is unmissable — this specifies S61 and S48's `machines` row

*"Machines could showcase the categories first, then the machines with their status obvious (free / used / you don't have the badge…)"*

So: category first as the primary grouping, machines within it, and **on every machine a single obvious state**. The three the operator named are three different kinds of thing and must not be one badge colour:

| state | kind | what the member does next |
|---|---|---|
| **Free** | availability | use it, book it |
| **In use** | availability | queue for it, or see when it frees |
| **You don't have the badge** | *permission* | go and get trained — a different verb entirely |

⚠️ **Add a fourth: "out of service"** (S62), and a fifth once packages land: **"not in your package / outside your hours"** (S67). ⚠️ **Permission states must not be rendered as availability states** — a member who reads "unavailable" when they mean "untrained" goes and waits instead of going and training.

⚠️ **This is the S47 query problem again, now larger.** Per-card live status across every machine, grouped by category, is the batched-query work parked in Phase H **S41**. The *layout* can ship first with status on the detail page only.

#### 5 · The badge swipe is the claim action — this joins S68 and S70

*"If someone swipes their card after the no-show limit, it moves to them in the queue."*

The RFID swipe is already the "I am physically here" signal. It becomes the arbitration between a released booking and the queue: the no-show window expires, the slot is released, and **the swipe is how the next person takes it** — no staff involvement, which is decision 0 exactly.

⚠️ **The operator's sentence has two readings and the difference matters**: does the machine go to *whoever swipes*, or to *the person at the head of the queue, once they swipe*? The second is fair; the first is a race won by whoever loiters nearest. **Recommend: only the head of the queue can claim, and the ticket is offered to them for a bounded time before it passes down.** Confirm before building.

⚠️ **This makes the RFID endpoint a decision point, not just a gate** — it now reads the queue and mutates it. That endpoint **still fails open** with `FABOS_RFID_API_TOKEN` unset (S38/S48). It was already true for S67; it is now true for S70 as well.

#### 6 · Packages grant *capabilities*, not just access — this extends S67

*"Maintenance in some labs would be done by members and no staff, but packages could handle this (ability to do maintenance becomes a thing part of the package or not)."*

So a package's grants are not only *(equipment × window)* but also named capabilities — the first being **may perform maintenance**.

⚠️ **This is the biggest architectural consequence in this whole message, and it needs deciding, not assuming.** It makes packages a **third authorisation source** beside roles and badges — and S49's whole mechanism, `can_reach()`, consults `security.access_map`, which **cannot see a package**. A maintenance action gated by a package is invisible to the one mechanism built to keep affordances honest.

Three ways out, and the operator or the next session must pick one:

1. **A package grants a role.** Reuses the firewall and `can_reach()` unchanged. Crude — roles are global, packages are scoped.
2. **A voter.** Correct in Symfony terms; means `can_reach()` gains a companion for capability questions, and S49's "one map, no second copy" property weakens.
3. **Capabilities stay out of packages** and member-maintenance is a role an admin grants. Simplest, and loses the operator's point.

⚠️ **Recommend 1 or 2, and recommend deciding it in S67 rather than S70** — but ⚠️ **do not let packages become a general-purpose RBAC.** A small closed list of capabilities, defined in code, is a feature; an operator-editable permission matrix is a second security system.

⚠️ **Member-performed maintenance touches safety.** "May log that I cleaned the bed" and "may sign off that the laser is safe again" are not the same permission. **Split them before building**, or the out-of-service flow (S62) can be cleared by whoever feels like it.

#### 7 · Ticket printer: yes, but a future release

*"We could add a printer if you know how to handle the print in a future release."*

**Yes, this is tractable.** Thermal receipt printers speak **ESC/POS**, and the clean path for us is a **network printer on the lab LAN, addressed from the server over raw TCP 9100** — the app renders the ticket as ESC/POS bytes and writes them to a socket. No driver, no browser print dialog, no kiosk PC. CT 210 already sits on `192.168.100.x` with the RFID readers, so this is the same shape of device we already talk to.

Fallback if the printer must be USB: a **small polling agent** next to it, exactly mirroring the existing RFID device pattern.

⚠️ **The printed ticket must never be the only record of a place in the queue.** Printers jam, run out of paper and get unplugged. The queue position lives in the database; **the paper is a receipt, not the ticket itself** — otherwise a paper jam loses someone's turn.
⚠️ **Print asynchronously.** A dead printer must not block or fail joining the queue; queue the print job the way mail is queued (`messenger`, and the `fabos-worker` unit already runs).
⚠️ **Deferred to a future release by decision.** Not in S70's scope; **S70 must work with mail-only** and treat paper as an enhancement.

---

### S74 · Categories you can act on

**Why.** Decisions 1 and 2. `MachineCategory` is a filter today; it becomes the thing a member queues for and books. It is a **prerequisite for S70** — queueing for "any FDM printer" is meaningless without it — and it is what the machines page leads with.

**Scope.** Promote the category: an operator-managed list (UTA and Fabman both have this tiny CRUD, we do not), category pages, category-first grouping on `/machines`, and the pool-booking model from decision 2 once its resolution timing is settled.

⚠️ **Resolve the at-booking-vs-at-start question first.** It is in decision 2 and everything else depends on it.
⚠️ **Categories are operator vocabulary (S31) and per portal**, like everything else nameable.
⚠️ **Not every category is a pool.** Two lasers of different bed sizes are one category and *not* interchangeable. **A category needs a "these are interchangeable" flag**, or pool-booking will hand someone a machine their job does not fit — which is the exact failure the operator named when they said "the only one big enough".
⚠️ **`places` are polymorphic with machines in `Reservation` already.** If categories become bookable, decide whether places get them too, or the two halves of the calendar diverge.

**Verify.** A member books a category and gets a usable machine. A member who needs the big laser can still pick it. A non-interchangeable category refuses to be pool-booked. Every machine shows one unambiguous state.

#### What the prototype settled, 2026-08-01

Built at `/proposition/machines` and iterated with the operator. **Approved by inspection; these are now S74's brief, not open questions.**

⚠️ **Categories are a quick-filter bar, NOT section headings — and this was learned the hard way.** The first build grouped the grid into one section per category. Eleven machines across six categories renders **one card and three empty cells, six times down the page**: *a group of one is a row of one*, and no amount of styling fills it. The operator's words were "lots of emptiness, I don't like it." Moving categories up into a persistent tile bar made them **more** prominent than a heading was — always on screen, carrying the icon and the free-count — while the grid below stays one continuous flow. ⚠️ **Do not re-propose category sections.**

⚠️ **The category tile carries three things and the third is the point**: icon, total, and **how many are free right now**. That last number is what decides whether the member walks over there.

⚠️ **The icon belongs to the category, not the machine.** `iconSlug` is a column on `Machine` today, so eleven rows each carry their own copy of "what a laser looks like". It moves to the category. **Repeat it on every card too** — in one continuous grid there is no heading saying what kind of thing you are looking at, and the icon lets the eye sort by shape without reading a word.

⚠️ **`MachineCategory` exists as an entity and `MACHINE_CATEGORY` does not exist in the database** — 1146, checked on the live box. Categories are denormalised `categorySlug` / `categoryLabel` strings on `Machine` and nothing else. **S74 cannot "just use `MachineCategory`"; it has to create the table.** This is the known entity-ahead-of-migrations drift, not a broken deploy — but it means S74 starts with a migration.

✅ **Categories live inside the `machines` feature and are operator-editable**, like every other piece of vocabulary (S31), and per portal.

🟡 **Shipping a default set of categories with the app was raised and explicitly deferred** — "we'll deal with that later." Worth revisiting: both Fabman and UTA ship one, and an empty category list is a bad first run.

⚠️ **`Machine.photo` is editable in the admin and rendered on the public list by nothing.** So the grid has never met a real photograph, and the normal case is a half-populated lab — three machines shot, eight not. **Both states need the same fixed aspect ratio** or the grid goes ragged the day someone uploads one picture. The prototype cover-crops a photo and falls back to the category icon on a tinted tile; neither reads as a missing image.

⚠️ **The machine icons hardcode `#9E1B56`** in `machines.html.twig`. The prototype switched them to `var(--color-primary)` so they follow the portal's own accent (S27) and dark mode. **A page full of hardcoded brand hexes is what turns a rebrand into a rewrite** — fix it in S58/S74, not later.

---

### S75 · Remove favourites

**Why.** Operator decision, 2026-08-01: *"We can remove favourites altogether, not sure the value it adds."* Taken while comparing the machines list against the prototype — with the star gone the card header rebalances, and the category icon takes the vacated side and does real work.

**Scope.** `MachineFavorite`, `MachineFavoriteRepository`, the favourite controller and its routes, the star on the card and the list row, the `data-favorite` attributes, the "Mes favoris" filter button, the favourites branch of the list JS, the sort-favourites-first pass in `SiteController::machines()`, the CSS, and the translation keys across five locales.

⚠️ **This closes an open audit item rather than leaving it.** The 2026-07-10 audit flagged **missing CSRF on the favourite add/remove endpoints**. Deleting the feature removes the vulnerability — but only if the *endpoints* go, not just the button. **Grep for the routes, not for the star.**

⚠️ **It is a table with member data in it.** Dropping it is a contract migration: code first, then the migration, and it belongs in the same conversation as S57's erase-vs-anonymise decisions.

⚠️ **S54's rule applies to the CSS.** Two of the rules it deleted were selector *lists* mixing dead and live selectors — prune selectors, never delete whole rules — and removing a cell from a grid row means fixing `grid-template-columns` too. The machine list row is a nine-column grid; check it.

⚠️ **Locale parity is verified at 627 keys × 5.** Every `machines.fav_*` key goes, in all five, and the parity check runs afterwards.

**Verify.** `grep -ri favorite src/ templates/ public/css/ translations/` returns nothing. The nine-column list row still lines up. The endpoints 404. Locale parity holds.

---

### S70 · A queue for the machines nobody can book

**Why.** See above. This is the model gap, not a UI gap.

**Scope.** A queue ticket: person, a device **or a device group**, joined-at, an estimated wait computed from what is actually running, a contact channel, and a state. A member joins from the machine page; staff see a "now serving" screen; being called transfers the ticket into a real usage session.

⚠️ **Per resource, not per lab.** A laser cutter is booked and a printer is queued **in the same lab, on the same day**, and both models must coexist. The choice belongs on the resource — a flag on `Machine`, decided by the operator. **Do not replace booking; add a second mode beside it.**

⚠️ **"Any FDM printer" is the whole point.** Queueing for a *specific* printer recreates the problem — the member waits behind a 9-hour job while an identical machine sits idle. This needs device *groups*, and we have `MachineCategory` already, unused for anything but filtering.

⚠️ **An estimate that is wrong is worse than no estimate.** It has to come from real running-session data (`LogUtilisation` / `MachineUsageHistory`), it has to say it is an estimate, and it must degrade honestly to "we can't tell yet" on a machine with no history.

⚠️ **Notification is the feature.** A queue whose member has to stand and watch the screen is a whiteboard with extra steps. Mail exists (S13/S15, the worker and the reminder timer). **SMS does not, and UTA carries `phone` + `carrier` per ticket for exactly that reason** — decide whether we need it, and if so it is a new external dependency and a cost, not a session detail.

⚠️ **UTA prints a paper ticket, and we have no printer.** `wait_ticket.php` drives a small thermal printer: the member takes a physical ticket and walks away, which is what makes the deli-counter model work for someone with no phone to hand. **Decide the equivalent before designing the flow** — a printed ticket, a QR the member scans to hold their place, or mail-only. Each gives a different screen. ⚠️ **Do not assume the hardware**: if a ticket printer is wanted it is a purchase and a driver, and the kiosk screens (`kiosk-*`) are the nearest thing we have to that medium today.

⚠️ **Feature-gate it** like everything else in the registry, and ⚠️ **it interacts with quotas**: is a queue ticket subject to the booking quota? Decide before building, or a member holds two bookings and six queue tickets.

⚠️ **This is very likely a phase, not a session** — an entity, a member surface, a staff surface, notification, and the storage loop below. **Scope it honestly before approving it.**

**Verify.** Two machines in one lab, one bookable and one queued, both working. A queue for a group hands the member the first free machine of that group. The estimate is derived from real sessions and says so. Leaving the queue works and does not notify the wrong person.

---

### S71 · Where the finished part waits

**Why.** S70 only makes sense if it closes. The member left the building; the print finished; the part exists and needs an address.

**Scope.** Numbered storage bins, an object assigned to a bin when a job completes, a notification to its owner, and a pickup action that frees the bin. UTA has `StorageBox`, `ObjBox`, a bin creator and a pickup page.

⚠️ **Do not build this before S70 is approved, and do not ship S70 without it.** Separately they are both half a workflow.

⚠️ **Bins are physical and finite.** The interesting states are "all bins full" and "this part has been here three weeks" — an unclaimed-object policy is an operator decision, and it is the one that decides whether the shelf works. Ask before designing.

⚠️ **UTA has the boxes; we would be buying shelving as well as writing software.** The operator has seen theirs. **Confirm the physical setup exists or is planned before this is scheduled** — a bin-numbering screen for bins that do not exist is the purest form of the dead affordance S54 spent a session deleting.

---

### S72 · The member who found the broken machine

**Why.** `MaintenanceTask` is authored by admins on a maintenance screen. The person who discovers a jammed extruder is a member standing in front of it with a phone. UTA gives them `Service` — open a ticket against a device, with a history and a log.

**Scope.** A fault report from the machine page: what, optionally a photo, and it lands where maintenance already lives. Status back to the reporter. Staff triage it into the existing `MaintenanceTask` rather than into a parallel system.

⚠️ **One concept, not two.** A ticket that does not become a maintenance task is a second inbox nobody reads. **Reuse `MaintenanceTask`**; a member report is an origin, not a new entity.

⚠️ **Reporting a fault should be able to take the machine out of service** — that is S62's out-of-service action, triggered by staff, not by the reporter. **A member must not be able to disable a machine by filing a ticket.**

⚠️ **Photos are uploads**, and this repo already has an upload path with its own history (`creation-upload`, the branch name). Reuse it; do not add a second.

---

### S73 · Why is this person using this machine

**Why.** `Purpose` is a two-column table — id and title — attached to usage. For a university lab it is the reporting axis: how much of the laser's time was a course, research, or personal work. We have `Institution` on a person and nothing at all on a session.

**Scope.** An operator-editable list of purposes, an optional purpose on a usage session and/or a booking, and a breakdown in the existing statistics.

⚠️ **Optional, and it must stay optional.** A required dropdown between a member and a machine is a tax on every single use, and it will be answered at random within a week — which is worse than no data, because it looks like data.

⚠️ **It is operator vocabulary (S31) and it is per portal.** A course list is not the same in two deployments.

⚠️ **"Which course" is about a person, and aggregate reporting on people has a privacy question** the leaderboard already raised and never settled (see the 2026-07-10 audit). Answer it here or inherit it.

---

### S76 · Access modes — how you get on a machine

🔴 **Proposed 2026-08-01.** Written at the operator's request, out of the prototype: *"we should have machine statuses, maintenance, first come first served, by appointment, by reservation (appointment means a staff must be there)."*

**Why.** Every machine in this app is booked, because `Reservation` is the only way in. Real labs have at least three different answers to *"how do I get on this thing"*, and today the app can express one.

⚠️ **The operator's list mixes two different concepts, and separating them is the whole session.**

| | | set by | changes |
|---|---|---|---|
| **Mode** | *libre-service* · *sur réservation* · *sur rendez-vous* | the operator, on the machine | rarely |
| **State** | *libre* · *occupée* · *complet* · *hors service* · *labo fermé* | computed | minute to minute |

**`maintenance` is a state, not a mode.** Putting it in the mode list means an operator sets it once and it never clears itself — a machine stuck out of service until someone remembers the dropdown. Out-of-service is S62's action, with a reason and an expected return.

**Scope.** A mode on the resource, and the three consequences it has:

| Mode | The card's action | What the booking layer must do |
|---|---|---|
| **Libre-service** | none — "allez-y" | refuse a booking attempt; there is nothing to book |
| **Sur réservation** | *Réserver* | today's behaviour, unchanged |
| **Sur rendez-vous** | *Demander un créneau* | refuse unless a staff member is present for that slot |

⚠️ **"Sur rendez-vous" is the expensive one and it should be scoped separately or dropped.** It means the app has to know **when staff are on site**, and there is no staffing calendar — `UserAvailability` exists for bookable *people* (the `person_booking` feature), which is a related but not identical idea. **Decide whether appointment mode reuses `person_booking` or needs a staffing rota, before committing to it.** The other two modes are cheap; this one is a feature.

⚠️ **Mode changes what "available" means, so it changes the list card.** A libre-service machine is never "complet" — it has no bookings — and showing it a next-free-slot is nonsense. The four-state display built in the prototype assumes reservation mode throughout; **each mode needs its own state vocabulary**, or the card lies about two thirds of the lab.

⚠️ **It interacts with S70's queue, and the pairing is the interesting part.** A libre-service machine with a queue is exactly UTA's model — walk up, take a ticket, wait your turn. **Mode and queue are orthogonal**: mode says how you get on, queue says what happens when it is taken. Do not fold them into one enum.

⚠️ **`ReservationService::book()` must enforce the mode**, not just the UI. A libre-service machine that still accepts a POST to the booking endpoint is a machine with two conflicting truths.

⚠️ **Per resource, and it applies to places too** — a meeting room is far more likely to be *sur réservation* than a soldering iron. `Reservation` is already polymorphic; the mode column should be too, or the two halves of the calendar diverge.

⚠️ **Default must be today's behaviour.** Every existing machine becomes *sur réservation* on migration, or the deploy silently changes how the whole lab works.

**Verify.** All three modes on three machines in one lab. A libre-service machine offers no booking affordance *and* refuses a hand-made POST. An appointment machine refuses a slot with no staff. Existing machines behave exactly as they did before the migration.

---

### Where these go in the order

```
already shipped: S45 tokens ✅ · S46 public layout ✅ · S50 admin nav ✅ · S54 dead buttons ✅ · S55 single-feature ✅

    S58 detail pattern ─┬─> S62 staff surfaces ──> needs S63 first (audit)
    S59 list pattern ───┤                          needs a staff-only account
    S60 sentence forms ─┘
                        └─> everything Phase D builds, checked against the contract

    S63 notes + changelog ──> unblocks S62's override
    S61 three-click booking ──> card face blocked on Phase H S41 (the batched query)
    S64 guided chains · S65 member area · S66 holidays — independent, any time

approved 2026-08-01, still unscheduled:
    S67 packages ──> BLOCKED on S38/S48: the RFID endpoint fails open today,
                     so a permission model enforced only in the web flow is not enforced
    S68 lock + no-show ──> needs the reader-coverage count across the 11 machines
    S69 archive ──> after S58/S59, or it is fifteen hand-edits instead of two

from UTA, proposed only — and these are MODEL work, not UI work:
    S70 queue ──┬─> S71 storage + pickup   ⚠️ one workflow; neither ships alone
                └─> ⚠️ probably a PHASE, not a session. Scope before approving.
    S72 member-reported faults ──> folds into the existing MaintenanceTask
    S73 purpose on a session ──> independent, cheap, any time
```

⚠️ **S70–S73 are the odd ones out in Phase U and it is worth saying so.** Phase U is a design-system phase; these are **domain model** changes that happen to have screens. If they are approved they should probably become their own phase rather than inflating this one — the queue in particular. **They are written here because that is where the comparison landed, not because that is where they belong.**

**If only three of these are ever approved, take S58, S59 and S63** — the two shape sessions before Phase D adds ten screens to the pile, and the audit trail that S62 and every future override depend on.

⚠️ **All nine still sit behind the same open decision as the rest of Phase U: `importmap()` is called in zero templates, so Turbo and Stimulus run nowhere.** Every session above is written to be buildable *without* them — native `<details>`, `<select>`, full page loads. **If AssetMapper is turned on first, S58–S61 get materially better and materially cheaper** (inline edit, drawers, a real combobox, a confirm step without a page load). That decision is worth making before S58, not after.

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

---

### S77 · Cancelling and changing a booking + S78 · one code, many uses (2026-08-02)

**Shipped and deployed in one long operator-in-the-loop session.** 15 commits. Verified on CT 210 by `lint:twig` (171), `lint:yaml` (5 locales, 913 keys at parity), a 147-route sweep, and — new this session — **browser geometry measurement**, for the reason below.

#### S77 — four verbs, one decision point

`BookingVerbService` answers *may this, and if not why not, in words*. `ReservationService` gained `cancel/endNow/reschedule/restore`; `POST /api/reservations/{id}/…` carries them. The page decides nothing; it reads verdicts.

- **Terminer maintenant** shrinks `dateFin`. Never blocked by the lock window, no mail, no confirmation — the member is standing at the machine having just finished.
- **Déplacer** re-runs the *entire* booking sequence with `ignoreId` set, then applies. Nothing is written unless every rule passes, so a failed move leaves the old slot standing. Needed `?int $ignoreId` on `countActiveUpcomingForUser` + `countForUserStartingBetween`, or a booking blocks its own move and anyone on their cap can never move anything.
- **Undo** (S47's leftover) re-validates the whole path rather than flipping a status — the slot really was released.
- `BookingPolicyService::changeDeadlineFor()` is the **S68 seam**: returns null, does no query, one line to fill in.
- **A page per booking, `GET /reservations/{id}`** — added on operator feedback because the card's "Voir" led to the *machine*. The card now carries one *Gérer* button; the page holds every verb. ⚠️ Kind-agnostic and verified so (booking 31 is a *Space*). ⚠️ **404, not 403**, for a booking that is not yours — a 403 confirms it exists and the id is a small integer.
- **Your own calendar slots link to it.** Only your own: `url` is emitted solely when `mine`, so other members' booking ids never reach the browser — the same line S38 drew replacing `user_id` with `mine`.
- **`VerbContext::Member|Staff`.** Cancelling a *past* booking is records management, not a member changing their mind. Role alone put that control on a personal page; `Staff` is reachable only through `POST /api/staff/reservations/{id}/cancel` (`IsGranted('ROLE_ADMIN')`), which `/admin/reservations` posts to. ⚠️ Still unaudited — attribution is S63.

🔴 **Every `booking vs now` comparison was skewed by the lab's UTC offset.** S38b classified the two storage conventions and fixed *display*; it never looked at *comparison*. A `Reservation` is stored as lab wall-clock and hydrated with no zone, so comparing it to a real instant is out by the offset — **always permissively**, which is why a finished booking sat in *À venir* for two more hours and stayed cancellable. New `LabClock` (`now()` / `instantOf()` / `storedFormOf()`). ⚠️ **`Event`, `OpeningHours` and access passes have the same skew — recorded, not swept.**

🔴 **The cancel redirect handed a client-supplied `Referer` to `redirect()`** — an open redirect, now reduced to path + query.

#### S78 — the shared shells

Measured case: **54 of 126 templates carry their own `<!DOCTYPE>` and `<head>`**, 82 have inline `<style>`, **70 contained a literal hex (each a dark-mode hole)**, 25 tables across **13 class names**, and the admin side had **no translations at all**.

Shipped: `_breadcrumb` (12 callers, 0 hand-rolled left) · nav active state + header search as a real form · `_data_table` + `_admin_delete_form` + `confirm_controller` (**7 of 25 tables**) · `form/admin_theme.html.twig` (**116 field triplets** → `form_row`) · `_admin_meta_grid` (**all 6 dark-mode holes closed**) · `adm.*` in five locales.

**The signal vocabulary.** Callers now name the *meaning* and the card picks the colour: `go` green · `wait` blue · `caution` amber · `stop` red · `muted` grey, in both halves. ⚠️ Amber is never "busy" and blue is never "warning". `_state_chip.html.twig` extracted so the booking page and the card share one mapping. Behaviour-neutral, verified by comparing emitted classes before and after.

#### 🔴 The one that matters: the card clipped its own controls

`.ml-card` is a grid item with `overflow: hidden`; `.ml-card-link` took `height: 100%`, so the `actions` block — a **sibling after the link** — fell past the card's height and was cut off. Measured: button bottom **892** vs card bottom **849**, 43 px past the clip.

**Every control in that block had been invisible for its entire life** — the cancel form shipped the day before, then S77's three new verbs. The page looked like it had no controls because on screen it had none. It also sent the S77 brief's own diagnosis astray, which blamed "cancel only renders on future bookings".

⚠️ **It survived two sessions because verification rendered the page and grepped the markup, which was correct every time.** `app:render` + grep proves the HTML exists; it never proves a member can see it. The operator found it in one screenshot.

#### Other faults found by looking rather than grepping

- `/admin/settings` was **500ing before the session started** — CT 210 carried an `AdminController.php` two lines ahead of the Mac calling a `SiteSettingService` method nobody wrote. ⚠️ **The container's filesystem can be ahead of the Mac in ways no local grep shows.**
- `/formations` rendered `<img src="/laser">` on every card — **`Formation.image` holds icon slugs, not paths.** The column is misnamed.
- `person-booking` rendered *"Accueil / / Léa"* for anyone neither trainer nor staff.
- `.navbar-link.active` was styled twice, light and dark, and **nothing ever emitted the class** — the menu had no active state on any page.
- The header search was a bare input with **no `<form>`**, working only via JS with a hardcoded `/search`.

#### Postmortem — three 500s shipped to `/formations` in a row

Two Twig comments placed **between two keys of an argument hash** (a syntax error), then a comment containing the literal comment-close sequence in its prose, which **ended the comment early and printed the rest onto the page** as visible text. All three reached the live site because the service was restarted without reading the lint output. **Lint now runs before `cache:clear` and the restart, and its output is read.**

### S79 — events look like events, dates speak French, and the front page answers the two questions (2026-08-02)

Five operator-asked deliverables, each committed and deployed on its own, plus S78 step 6.

**The artwork decides the page shape.** An organiser uploads one of two different things: a **poster**, already a finished design with its title and date printed inside it, or a **photo**, which carries no words. The old page treated both the same — cropped to a 21:8 band with our own title written over it, which for a poster cuts the words off and says everything twice. A poster is portrait and a photo is landscape, everywhere, on every wall — so `EventArtwork` **measures the file** and the page picks: poster shown whole on a blurred wash of itself with the words underneath, banner with the words laid over. No new column, no migration, nothing an operator can set wrong; uploading the file already said which it is.

**🔴 Two defects found only by looking at the file, not the markup:**

1. **The catalogue was pulling 46 MB to fill two 464 px cards.** The live posters are 23 MB PNGs behind a single-threaded `php -S`; the cards stayed blank for seconds and read as broken. `EventArtwork` now caches a 1600 px JPEG beside the original — **23 MB → 323 KB, 72×** — with every failure path falling back to the original, because a missing thumbnail must cost a slow card and never a 500.
2. **Those PNGs carry an `eXIf` chunk making them portrait.** The browser honours it; `getimagesize()` does not, and `exif_read_data()` **does not read PNG at all**. So a portrait poster was measured as a landscape banner and cropped to a letterbox, *and* the generated thumbnail came out lying on its side beside an original that looked fine. The chunk is located and its TIFF header parsed by hand (~30 lines, no dependency), the orientation decides poster-vs-banner, and it is baked into the JPEG GD writes. ⚠️ **Any image feature here must ask orientation before it asks dimensions.**

**Dates stopped speaking English.** Twig's `|date()` is PHP's `date()`, which has exactly one language, so `|date('M')` printed `AUG` to a French reader on a French page of a site with five catalogues — and nothing in the translation layer could have fixed it, because the string never went through the translator. `|loc_date()` formats through `IntlDateFormatter` with the *request's* locale, on ICU patterns (`MMM`, `EEEE`), the same vocabulary `twig/intl-extra` uses. ⚠️ It deliberately does **not** convert timezones: everything it formats is convention B, and shifting it would move a 14:00 workshop to 16:00. The mirror bug in JS — the two calendars passed `'fr-FR'` in hard, in a dozen functions, so a German reader got French — is one `window.FABOS_LOCALE` read off `<html lang>`.

**`/events` joined the catalogue.** It was the seventh list doing the same job its own way. It renders through `_catalogue.html.twig` now, two per row, with the shell's search and an à venir / passés / tous filter. **Density is one number**: `card_min` writes `--ml-card-min` on `.ml-grid`, so no page declares a grid rule of its own; `media_ratio` is the same idea for the picture box. ⚠️ `filter_all_value` was added because /events' bare URL is *not* "everything" — its default is itself a filter, and without it the all-tile lit up while showing a subset.

⚠️ **`x|default(true)` fires on empty as well as undefined**, so `show_tile_icons: false` came back as `true` and did nothing. Any boolean shell parameter needs `is defined ?`, not `|default`.

**The front page answers "are you open?" and "what's on?" in the hero.** Both were full-width sections a scroll and a half down. They are a two-panel deck across the bottom of the hero now, overlapping the image by 68 px so they read as part of it. The events rail is a **native `overflow-x` scroller with snap points**; Stimulus adds auto-advance and dots and does not build the carousel, so with the script gone you still scroll it by finger. On a narrow screen the **hours come first** — "are you open?" beats "what's on?" on a phone, and the events panel is the taller of the two. ⚠️ Both panels obey the **existing** `opening_hours` / `upcoming_events` visibility rows: moving a block up the page must not take it out of the operator's hands. They are removed from the *personalisation* modal only, via `HomepageVisibilityService::DECK_SECTIONS` — the deck's arrangement is fixed, and a handle offering to reorder them would be a control that does nothing. The body flow now opens on "comment ça fonctionne".

**S78 step 6** — 89 of the 108 form triplets became `form_row()` across 16 templates; the 19 left are three shapes the theme does not reproduce, each listed in `UI-CONSISTENCY.md`.

⚠️ **The route sweep was checking less than it claimed.** It probes `{id}` as `2`; institutions start at 5 and lab-pages at 3, so every `*/2/edit` line was a 404 being read as swept — including four pages this session changed. Render edit pages by an id pulled out of the list page.

⚠️ **A Twig comment between two keys of an argument hash is still a parse error**, and it shipped a 500 to `/events` for about a minute before the lint output was read. Third time in two sessions. Comments go *above* the tag.

---

### S86 · A working brief instead of rereading the archive — ✅ shipped 2026-08-08

`WORKING_BRIEF.md` is the short entry point for a new person or agent: the durable working rules, architectural seams, deployment routine, current priority and the historical traps that recur most often. It is deliberately not a second roadmap or a compressed copy of every session. The current plan stays in `ROADMAP.md`, the deep handover in `PROJECT_STATE.md`, and the reasoning for a touched subsystem stays in this history.

It is rendered at `/roadmap/brief`, alongside the roadmap and history. The page reads the file through `MarkdownDocService`, so the repository source and operator-visible page cannot drift apart.

### S87 · Institutions belong with badges — ✅ shipped 2026-08-08

Institutions are the external bodies that recognise badges, not a venue setting. The shared admin navigation now places *Institutions* in the *Badges* section beside the badge list, rather than in *Le lieu*. Because it now belongs to the badges feature section, it also follows that feature's visibility gate.

### S88 · One admin-list language — ✅ shipped 2026-08-08

The sixteen normal admin listings now use the same shared list tools: an immediate search, compact one-click category chips when the list has a real category, responsive tables that wrap rather than require horizontal scrolling, and the existing restrained row motion. The filter is a progressive enhancement: every row is still rendered and usable without JavaScript; with it, searching and switching a category happens in place without a reload.

`_admin_list.html.twig` owns the control shape, `admin_list_filter_controller.js` owns its behaviour, and `admin.css` is the single responsive-width authority. The translations for the shared state chips live in the five locale catalogues; entity categories remain data rather than a duplicated label set. The unusually wide machines, formations, reservations and members lists were reduced to the five decision-making columns during the earlier steps of this session.

### S89 · The lists use the maquette, not a lookalike — ✅ shipped 2026-08-08

The shared admin-list shell now emits the exact components shown on `/admin/design`: `.ml-cats` / `.ml-tile` for category chips and `.ml-filters` / `.ml-field` / `.ml-btn` for search. `admin.css` imports the existing `machines-list.css`, so there is one visual source for catalogue and admin list controls instead of a near-match maintained in parallel. The list-specific CSS cache version was advanced with the change.

### S90 · Delete an espace from its record, not its row — ✅ shipped 2026-08-08

The *Espaces* listing now keeps only its primary *Modifier* row verb, as the design maquette prescribes. Deletion moved to the edit page, in a shared danger zone explaining that upcoming reservations are cancelled while history remains. The existing CSRF and confirmation protections are unchanged. The five translation catalogues’ duplicate legacy `adm` namespace was also renamed, restoring valid YAML and keeping the active `adm.*` labels intact.

### S91 · One row, one action — ✅ shipped 2026-08-08

Every standard management row now exposes one primary navigation verb, *Modifier*, rather than competing *Voir* and *Modifier* controls. Destructive actions moved into the shared record-page danger zone for creations, events, institutions, Lab pages, loanable items, materials, spaces, maintenance tasks and portals. Machines now derive their one-click status tiles from the complete machine collection, so the chips remain correct while the search filters visible rows.

### S92 · Shared verbs resolve in every language — ✅ shipped 2026-08-08

The shared danger zone revealed that the legacy `adm` namespace had carried the common action vocabulary. Those values now live under `common.*` in all five locale catalogues, so the shared delete control renders its translated label rather than the literal translation key.

### S93 · Admin consistency audit — ✅ shipped 2026-08-08

The audit found and fixed the remaining missing badge deletion: it now lives in the badge edit page’s shared danger zone and relies on the database’s existing cascading/set-null relations. Events, materials and loanable items gained their missing one-click category filters. RFID readers no longer offer deletion from their list. Creations and usage logs were reduced to five useful columns so they remain legible without horizontal scrolling. The user list now follows the same *Modifier* verb as every other standard management list.

### S94 · A row is the target — ✅ shipped 2026-08-08

Rows with one destination now navigate from the whole row by click or keyboard, while retaining the visible *Modifier* link as an affordance. The behavior is owned by the shared admin-list controller and applies to every ordinary management list; operational rows with independent state-changing controls remain deliberately non-clickable. The design page documents the rule beside its list maquette.

### S95 · Shared access-pass desk and development workspace — ✅ shipped 2026-08-08

`/staff/acces-exceptionnels` was not failing a permission check: it was the last operational admin screen on an obsolete public-page scaffold, with copied CSS and no `admin.css`. It now uses the shared admin list shell, responsive form fields, common status chips and the standard confirmed destructive action. The pass remains a quota exception only; it cannot waive a safety badge or required training.

The site settings now own an off-by-default **development mode**. When enabled on Artemis it adds a clearly separated Development section to the admin navigation for the Design maquette and diagnostics. It changes presentation only: every route still requires `ROLE_ADMIN`, and no authentication bypass or hidden request variable exists. **Production release gate:** turn this setting off and remove the development section if the installation is promoted beyond its development environment.

### S96 · One machine category source, in both catalogues — ✅ shipped 2026-08-08

Machine category remains the existing canonical `categorySlug` / `categoryLabel` pair — no second category list or duplicate display mapping was introduced. The public Machines catalogue already used that source and keeps its category tiles unchanged. The admin Machines list now exposes the same category tiles (with the design-system category icons) plus a separate one-click Status row; both filters combine in the URL and survive a search. The shared list shell owns optional secondary tile groups, so another list can adopt the same pattern without copying markup or CSS.

### S97 · Usage Rights packages, without changing current access by surprise — ✅ shipped 2026-08-08

The former sidebar shortcut labelled “Accès exceptionnels” is replaced by **Droits d’usage**: an admin workspace for reusable packages, one-click active/inactive filtering, a full-access shortcut, selected feature grants, and fast member assignment/revocation on the same package record. It uses the shared admin list shell and the design-system card/form primitives; a row has one destination, and no package operation needs a separate selection page. The existing member detail page now carries a compact read-only rights summary linked directly to the owning package, so an operator can answer “what can this member use?” without hunting through the package list.

This is deliberately not a rename of `ACCESS_PASS`. Existing passes remain **quota overrides** with a use cap and a resource scope; silently treating one as broad entitlement would alter its meaning. The operational desk is still available for staff, but its revoke language now calls it what it is: a quota override.

Packages use the existing site-feature registry as their catalogue. A feature switched off in site configuration cannot be added to a package and is refused even if an old grant exists. The **full access** action means all currently enabled site features — never admin/staff roles, never badges or training. Feature enforcement is an explicit Site settings switch and is **off by default**, so the install preserves its current behaviour until an administrator has created and assigned packages. When enabled, the shared `UsageRightsService` checks web reservations before certification and checks authenticated event registrations centrally; administrators retain operational recovery, and guest event registration keeps following the event’s own guest rule.

The requested physical-card/RFID work is intentionally out of this session. Packages do not authorize reader starts yet and the UI says so plainly. That integration must be a later, security-reviewed change rather than a half-enforced rule.

### S98 · One effective Usage Right answer across the site — ✅ shipped 2026-08-09

Packages no longer stop at the admin editor. A closed `UsageCapabilityRegistry` now lists only actions with a real enforcement chokepoint: machine booking, space booking, person booking and member-only event registration. `UsageRightsService` returns one structured verdict (allowed, reason and granting packages) that both enforcement and presentation consume; Twig never guesses access from package rows. The same shared state/summary/matrix components appear on `/admin/design`, public catalogues and detail pages, both calendars, the member profile and the admin member record. The physical-card/RFID path remains deliberately excluded.

The package gate now evaluates the actual use interval: a package that expires before a future reservation or event no longer authorizes it merely because it is valid when the button is clicked. Certification, opening hours, quotas and feature availability remain independent AND conditions. Guest-open events remain genuinely public; packages govern member-only registration rather than creating a sign-out loophole.

“Full access” is now a durable package mode, not a snapshot of today's four checkboxes, so future audited capabilities join it automatically. Temporarily disabling a site feature suspends its grant without silently deleting it during an unrelated package edit. The activation setting is portal-local, carries a live coverage preview, refuses activation without an active assigned package, and requires an explicit impact confirmation. This prevents a global inherited switch from locking an override portal whose package table is empty.

Presentation is one system: `_usage_right_state`, `_usage_rights_summary` and `_usage_rights_matrix` own the recurring markup; `components.css` owns public/admin states, accessible touch targets, responsive wrapping and reduced-motion behaviour. All new language is present in the five catalogues. Pure policy and registry tests cover decision precedence and the honest capability list.

### S99 · Usage Rights list tiles translate through the shared shell — ✅ shipped 2026-08-09

The three package-list category tiles supplied translation keys but did not flag them as keys to `_admin_list`; the shared shell therefore correctly rendered their literal identifiers. They now use its existing `label_is_key` contract, restoring translated *Tous / Actifs / Inactifs* labels in every locale without a second template or translation path.

### S100 · One reviewed vision before expanding Usage Rights — ✅ shipped 2026-08-09

The next package phase is now explicit rather than hidden in a growing editor. `USAGE_RIGHTS_VISION.md` separates five concepts: site availability, audience, capability grant, physical/resource scope and quota profile. Package audiences reuse direct assignments and the existing role table, with distinct authenticated and genuinely anonymous guest audiences; administrator recovery remains a system rule. Physical Venue, reservable Place and publication Portal are deliberately different models. Equipment categories become an authorization scope only after a canonical category migration.

Booking quotas are planned as named, complete profiles. When several packages apply, FabOS evaluates their full policies as alternatives and never constructs a policy by taking unrelated fields from each. The plan also records two repairs that precede schema work: quota counts must respect the reservable type, and exceptional quota passes must not bypass hard slot-alignment or turnaround constraints.

Development mode now exposes two read-only, multilingual prototype pages: *Droits & quotas* composes audiences, the real audited capability matrix, venue/category scopes, a whole quota profile and a deterministic decision summary; *Structure* maps the installation's real organisation, single physical venue, shared portals, reservable places and current machine category data. Both use the shared admin design system, save nothing and link to the rendered vision document. The live package schema and enforcement state are unchanged.

### S101 · Groups connect people; they do not become permissions — ✅ shipped 2026-08-09

The *Accès & responsabilités* prototype now explains the model through one concrete Volunteers example instead of an inert package wizard. User and Global admin are the only system-account concepts. A user group organizes people and grants nothing by itself; it may assign an independently-audited administrative responsibility set and usage package. The example gives volunteers routine Maintenance visibility/management plus machine booking 24/7 without soft quotas, while explicitly withholding safety recommissioning and preserving badges, training, capacity and technical shutdowns.

### S103 · Feature Workspace registry and corrected Development maquettes — ✅ shipped 2026-08-09

`FeatureWorkspaceRegistry` is the target-only metadata source for the thirteen FabOS workspaces: sections, Use/Manage rights, scopes, filters, representative live routes and atomic capabilities. It drives the Development Workspaces matrix and central Themes contract without changing a voter, service or live grant. The Rights and Structure maquettes now reflect the approved model: no Report right, no portal-shaped target, global Formation catalogue with physical sessions scoped to a sub-location, Admin recovery respecting qualification and shutdowns, and a 24-hour IdP outage grace for existing sessions.

### S104 · Quota correctness — ✅ shipped 2026-08-09

Quota counters now receive `ReservableType`, preventing bookings of one resource kind from consuming another kind's active/day/week limits. Access passes are evaluated after slot alignment and the resource buffer, so they lift soft quotas only. Targeted tests cover both non-bypassable constraints and type propagation.

### S105 · Portal freeze and consolidation inventory — ✅ shipped 2026-08-09

Portal mutation is frozen. `/admin/portals/consolidation` reports hostname plus scoped settings, features and usage packages for every portal before any data consolidation; no row is migrated or deleted.

### S106 · Default sub-location and hours — ✅ shipped 2026-08-09

The migration creates `VENUE/default` from the prior physical identity settings and backfills every opening-hour row before making `venueId` mandatory. Opening-hours reads and admin writes now resolve the default venue, preserving the existing rendering while making the physical scope explicit.

### S107 · Physical resource venue scope — ✅ shipped 2026-08-09

`MACHINE`, `PLACE` and `LOANABLE_ITEM` are backfilled to `VENUE/default` before their `venueId` foreign keys become mandatory. New records use that default until a venue picker exists. On-site events use the default venue; off-site events retain no venue by design. Loans and RFID readers derive their venue through their item or machine, so no conflicting duplicate scope is stored. There is no persisted training-session model yet.

### S108 · Profile venue preference and display context — ✅ shipped 2026-08-09

`UTILISATEUR.preferredVenueId` stores an optional display preference. The shared `VenueContext` resolves a valid explicit `?location=` first, then the profile preference, then all active venues; invalid or inactive slugs fail explicitly. The common selector is used by the Machines, Spaces and Events lists. It is deliberately only a display filter and never an authorization grant.

### S109 · Protected group descriptors — ✅ shipped 2026-08-09

The existing `ROLE`/`UTILISATEUR_ROLE` security-membership mechanism remains active. `ROLE` now includes a stable group key, local presentation fields and protected metadata. The migration backfills Admin, Staff, User and Formateurs without renaming their security rows and adds Manager, Super user and Guest descriptors. User and Guest remain virtual audiences, so no membership or current route authorization was silently changed.

### S110 · Shadow Use/Manage grants — ✅ shipped 2026-08-09

`UsageGrantAction` permits only `use` and `manage`. Manage declares reporting/export coverage but cannot cover a Use action. `ShadowUsageGrant` evaluates declared scopes with AND and remains unpersisted/non-authoritative; it provides testable semantics for S111 without changing any live route or voter.

### S111 · Persisted package v2 shadow — ✅ shipped 2026-08-09

Package v2 grants persist feature, optional section, action and optional venue scope. Group assignments are dated and revocable beside individual assignments. Legacy feature grants are copied as unrestricted Use grants and package rows are consolidated to the FabOS-wide scope. The v2 reader unions direct/group paths but remains diagnostic only.

The same page compares a standard member, volunteer and global administrator, then shows a portal-scoped presentation manager. It states the current domain limit honestly: portals own visual/configuration overrides but not events or resources, so delegated portal content management needs real data ownership before an authorization label can promise it. The proposed migration turns current Staff/Trainer business roles into initial groups, adds a closed responsibility-capability registry and moves delegated routes from the blanket admin firewall to voter/service checks only after shadow comparison. Mobile visual QA also found and fixed the shared admin shell's missing one-column collapse, so every screen using that shell now remains usable below 900 px. The live roles, package enforcement and admin access are unchanged.

### Archived delivery notes moved out of the active roadmap — S80–S85

The active roadmap used to carry several pages of already-shipped implementation notes. They are history, not remaining work. S80 normalized every image-upload path around the shared orientation-aware `ImageNormalizer`; the real event posters went from roughly 23 MB to 6.3 MB while retaining their existing PNG database names. S81 put the single shared header on admin pages. S82 moved the admin sidebar into NavBuilder, introduced feature grouping, the shared admin-list shell and the two-level navigation. S83 promoted surface, elevation and soft-state tokens. S84 converted visual div grids to semantic shared tables. S85 standardized the list table rules, identity cells, pinned single action and reusable icon vocabulary. The remaining list/filter work is replanned under S111–S126 rather than pretending those presentation sessions are still pending.

### S102 · One FabOS, several sub-locations, no portals — ✅ roadmap decision 2026-08-09

The operator replaced the S100–S101 target before schema work began. Portals will be retired. One FabOS instance manages physical sub-locations; packages belong to the instance and their grants may be scoped by sub-location and domain filters. Packages are assignable to people or editable local groups and expose Use, Report and Manage, while Global-admin recovery stays protected outside revocable packages. Reservations, quotas and reporting move into feature workspaces over shared backend services. The navigation target becomes Equipment, Events, Loans, Spaces, Training, Badges, Project gallery, Custom pages, Users, Locations, Packages, FabOS Network and Configuration.

The same decision adds opt-in public member profiles, signed one-shot QR import, and a later trusted FabOS-to-FabOS network for consented members, badges, training and machine make/model data. Badge grants are global to an instance and append-only; corrections become audited revocations rather than deletion. `USAGE_RIGHTS_VISION.md` now lists every contradiction with the live schema and the operator choices still required. The previous S100–S101 proposal remains here as history; its portal assumptions are explicitly superseded. No runtime behavior changed in S102.

Configuration will also gain one **Themes** workspace instead of continuing to scatter the public identity across Site settings, Portal overrides, navigation code and the homepage editor. It owns the safe, versioned editing experience for colours, brand images, public name, menu labels/order and homepage blocks/content/order, with preview, publish and rollback. Hours remain under Locations; the public welcome/homepage presentation belongs to Themes.

The reason portals are being removed is now explicit. A department that wants only Events or Loans, its own branding and its own administrators should run its own configured FabOS instead of projecting a filtered slice of another installation. Shared LDAP/OIDC/SAML makes sign-in familiar, but authentication never transfers local groups, packages, quotas or Admin recovery. Sub-locations are reserved for physically separate parts of one shared organisation/data set and therefore aggregate by default. Cross-instance badges, training, machine models and consented member claims travel through the FabOS network; events and reservations keep one owning instance and deep-link there by default.

The operator then closed the Guest, built-in group, Institution, badge, QR and package-composition questions. Guest is the anonymous audience and public visibility is independent from registration/action. Admin, Manager, Staff, Super user, User, Guest and Formateurs are protected built-ins; labs add groups without deleting these stable keys. One Institution URL either remains descriptive or, after secure discovery and explicit trust, creates an internal FabOS peer. Export is the intersection of instance filters and member consent. Badge awards remain as revoked history rather than being deleted. QR imports general fields and acquired badges automatically after one summary confirmation. Package grants are default-deny and cumulative across the person and every group; complete quota paths are alternatives and their fields are never mixed. Materials are a shareable instance catalogue, while availability, location and future stock remain local to sub-locations. This records product intent only; no runtime behavior changed.

Sol's independent review made those choices executable rather than ambiguous. Guest settings are FabOS defaults with per-event tri-state overrides, and the migration first backfills current visibility/registration. User is the virtual audience of every active account, avoiding missing memberships. Institution URLs become canonical unique HTTPS origins; a changed origin suspends trust. Personal exports require both operator allowlisting and member consent, while non-personal catalogues follow publication plus peer trust. QR imports carry expiration, revocation and tombstones so a revoked remote badge can never reappear active.

After reviewing the complete contradiction register, the operator asked for the bulky table to leave the active roadmap. A first edit mistakenly treated that request as acceptance of every unanswered recommendation. The operator clarified that the questions themselves must remain visible. The active roadmap therefore keeps five questions in a short dedicated section — Admin safety, Manage→Report, Training session localization, existing public identity surfaces and IdP session lifetime — without restoring the obsolete contradiction matrix.

A future optional **Commerce** phase is also approved in principle. It can sell package assignments, materials and machine/person/training-time credits through one shared offer/order/payment/refund/delivery engine surfaced inside the relevant feature workspaces. Payment confirmation is only an idempotent trigger for a domain delivery: it is not authorization, does not auto-book and never bypasses qualification, badges, shutdowns, schedules, capacity or unrelated quotas. No runtime payment behavior was added.

Sol's review made the future transaction boundary explicit: browser returns never confirm payment; verified provider events are deduplicated and drive per-line fulfillment through an outbox. Refund money and domain compensation are separate. Package compensation touches only the assignment created by that order line, materials require an atomic stock hold or an explicit backorder, and time credits use an append-only hold/consume/release ledger rather than a mutable balance. This remains architecture only.

The operator also recorded a deliberately much-later Training communications module. Assigned trainers and enrolled learners will exchange canonical internal FabOS messages, with a separate asynchronous e-mail copy per recipient. Conversation membership comes from the future formation/session/cohort and enrollment model; e-mail is notification, not the source of truth, and a mail failure cannot lose the internal message. The roadmap defers this to S134–S136 after the core and Commerce, including permissions, unread state, moderation, retention, export and privacy. No runtime messaging behavior was added.

Sol's review separated cohort announcements, one-learner private threads and explicitly composed groups so privacy never changes implicitly. Pending e-mail is cancelled if enrollment, assignment, account or participation is revoked before worker delivery. The initial content contract is bounded, rate-limited plain text rendered escaped; rich text and attachments are not silently included. This remains future architecture only.

## How these sessions are sized

Each `S##` is **one self-contained, deployable session**: build, deploy to the live container, verify, commit. If a session cannot be verified end-to-end it is too big and should be split. Every session ends with the app running.

---

## Phase C — S112–S117 · shared workspaces — ✅ shipped 2026-08-09

The descriptive S103 registry now drives the real operator shell. Every migrated list gets the same feature tabs, one quick axis, server-side search, progressively disclosed advanced filters and at most six removable URL chips. The five translation catalogues carry the shell copy and `/admin/design` renders the actual components. Quotas and Reporting are deliberately not rendered before S118/S119.

Equipment gained category and model/brand catalogue tabs plus nullable manufacturer/model metadata; Materials, Maintenance and typed machine reservations stay in the same workspace. Events and Loans use their existing domain adapters. Spaces gained nullable category, responsible person and department facets without weakening the required venue relationship. Existing Gallery, Pages, Users, Locations, Packages, Network and Configuration lists were connected to the registry rather than copied.

Badges are now non-destructive: the delete route and danger zone are gone, archive/restore preserves the definition, and all award/equipment/institution badge FKs prevent cascading deletion. The existing local award path was already insert-only. Configuration gained `/admin/themes`, with a scoped JSON draft, validated logo/colour values, preview, explicit publish and discard back to the published version; publication feeds the existing organization, venue, logo and accent readers.

Deployment used three additive migrations: space facets, equipment manufacturer/model, and badge archival/FK hardening. The migration was preflighted against the actual Artemis FK names before execution.
## 2026-08-10 — Phase D: policies, reporting, workspace parity (S118–S120)

Booking policies now cover the complete shared path: notice, horizon, granularity, duration, active/day/week caps, buffer and cancellation/reschedule notice. The new field is read by the existing central booking-verb service, so UI and API actions receive the same verdict and physical availability remains separate.

Reporting is a tagged adapter registry rather than page-specific queries. Its first reservation adapter serves the Equipment and Spaces workspaces with venue/date scopes, aggregate metrics and identity-free CSV exports; the stable capability names are `analytics.view` and `analytics.export`. The screen composes the existing workspace tabs, venue selector, admin list shell, summary cards and data tables.

Historical unscoped reservation links now redirect to the Equipment workspace with their filters preserved. The duplicate global Reservations navigation group is removed after Quotas and Reporting reached their feature workspaces; packages and usage rights remain reachable from the shared venue administration area.
## 2026-08-10 — Phase E: identity, public profiles and FabOS Network (S121–S126)

FabOS now separates sign-in federation from data federation. OIDC uses authorization code with PKCE, configured HTTPS issuers and secrets read only from named environment variables. External accounts are keyed exclusively by `(issuer, subject)`; matching e-mail never merges accounts, external claims never create roles, and local suspension remains authoritative.

Member pages are private by default. A member explicitly enables `/m/{slug}` and separately selects name, avatar, bio, badges and completed training; e-mail, RFID, identifiers, reservations, groups and packages are not representable in the public-field allow-list. Public pages are non-indexable by default.

The Network workspace centralizes the instance Ed25519 identity, OIDC providers and trusted peers using the shared admin shell. The target schema includes bounded one-use exchanges, replay storage, audit, credentials with source/expiry/revocation provenance, and machine catalogue projections. Imported machine metadata is filtered so local tokens, RFID, state, safety, location, availability and stock can never be overwritten.

Because FabOS has not shipped V1 and has no real installations, Phase E implements the target schema directly. No historical compatibility layer or transition route was added.

## 2026-08-10 — Phase F: S127 portal retirement — ✅ shipped on Artemis

The single-instance retirement removes hostname portal resolution and per-portal configuration from the runtime. Artemis is a disposable development dataset, so its one portal, two scoped settings and thirteen feature overrides were intentionally purged; packages would have been folded into the single site. Legacy `/admin/portals*` routes transition permanently to the structure screen. The migration's rollback deliberately refuses to fabricate discarded portal data. Artemis verification: rebuilt production cache, 188 Twig templates valid, service active, public homepage 200, structure screen 200 and portal transition 301.

### S128 · audit transversal du socle — ✅ shipped 2026-08-10

The workspace contract was rechecked rather than inferred from the UI: the focused workspace/reporting tests and full suite pass (**30 tests, 208 assertions**); all **188 Twig** templates and **29 YAML** translation/configuration files validate; and the live kernel renders all fourteen canonical shared-workspace routes with HTTP 200. The audit resolved route paths from Symfony before rendering, avoiding false negatives from guessed URLs. The scope remains descriptive where designed to be descriptive; the shell still does not grant access, and route/services remain the enforcement boundary.

### S130b · one admin sub-navigation — ✅ shipped 2026-08-11

The admin had **two** sub-navigations and drew both on every page. `NavBuilder`
emitted a labelled strip from the feature sections; `FeatureWorkspaceRegistry`
emitted a second, unlabelled one from a `TABS` map that `_workspace_tabs.html.twig`
rendered inside the content column. Measured on CT 210 before the change: **20 of
20 admin pages carried both.**

They were not two views of the same list. Équipement had six entries on one strip
and eight on the other. Institutions sat under Badges in one and under Réseau in
the other. Five sections rendered a bar whose only link was the page already open —
twice over. And because only the tabs read the catalogues, an English account saw a
French strip stacked on an English one.

**Six live screens were reachable only from the tabs** — `app_admin_reservations`,
`app_admin_booking_policies`, `app_admin_reporting`, `app_admin_machine_categories`,
`app_admin_machine_models` and `app_admin_homepage`. Deleting the tabs without
folding them in would have orphaned all six, which is why the coverage table was
built before anything was removed.

`NavBuilder` is now the whole navigation. The registry keeps `all()` and
`themeContract()` for `/admin/design/workspaces` and lost `TABS`, `forRoute()`,
`markActiveTab()` and the `feature_workspace()` Twig function.

**Two things the merge forced, both of them real.** Réservations, Quotas and
Reporting are *one route each serving several sections*, told apart by
`reservableType` or `workspace`. Matching on the route alone lit all three at once
and sent an operator looking at space reservations into Équipement, so entries
carry `params` and `isCurrent()` compares them. Those same params also go to
`canReach()` — it answers by generating the URL, and `/admin/reporting/{workspace}`
cannot be generated without one, so the first deploy silently dropped Reporting
from both strips and rendered `/admin/reporting/equipment` with no navigation at
all. The catch that swallowed it exists to make dead routes vanish quietly; it
does the same to a live route asked about incorrectly.

**The labels moved to the catalogues.** `workspace.tab.*` was already translated in
all five locales and keyed by route, so those 27 values were re-homed as
`admin_nav.entry.<route>` rather than rewritten; 11 entries and 14 section names
that had lived as French literals in `NavBuilder.php` were added. That file was
invisible to `scan_hardcoded.py`, which reads Twig text nodes — the same blind spot
that hid the dashboard for two batches of S134c.

**Shipped in the same session, from the whole-site review:**

- The **footer moved into `base.html.twig`**. S129 had established "the footer
  belongs to the shell" for the 25 pages on the list shell; the same split existed
  one shell up, where 82 templates extend a base that emitted no footer at all.
  Fifteen templates were compensating with a hand-written include. ⚠️ **The review
  said 12 pages lacked a footer; measuring the running pages found 27.**
- The users list said **Modifier** and opened a read-only detail page. Now *Voir*.
- A loan's object is a link to that object.
- Lecteurs RFID drew its pairing and create buttons **twice, three lines apart**;
  the panel copy is gone. ⚠️ The review said both RFID screens did this. Only one
  did.

**Tests.** The two `FeatureWorkspaceRegistryTest` cases that asserted the tab map
were removed — they described deleted machinery — and replaced by
`AdminNavCatalogueTest`, which fails if a navigation label is a literal, if an
entry key does not name its route, or if any key is missing from any of the five
catalogues. **31 tests / 780 assertions**, up from 30 / 167.

**Verified on CT 210, not inferred.** 34 admin pages re-rendered: every one HTTP
200, `workspace-tabs` count **0**, exactly one strip, exactly one lit entry, and
the parameterised routes lighting their own section. 50 pages checked for the
footer: **0 without**. Public and static pages checked for a doubled footer: all
exactly 1. Twig 191/191 and YAML 5/5 valid. Hash comparison over all 94 changed
files: identical. ⚠️ Twig lint refused this session's work **twice**, both times
over an explanatory comment and not the change — once for a comment between two
keys of an argument hash, once for spelling the comment-closing sequence inside a
comment. Both are the traps `ARTEMIS_DEPLOYMENT.md` already names, and both were
caught before the restart because the lint runs first.

### S130c · the sub-venue filter becomes a one-click filter — ✅ shipped 2026-08-11

Every other filter on an admin list is one click on a labelled chip row:
categories, status, the tile groups. The sub-venue filter — the one that scopes
what all the others operate on — was a `<label>`, a `<select>` and a
right-aligned sentence in a full-width bar of its own, sitting *above* the panel
that holds the rest. It cost two clicks and a menu, and its placement said it
belonged to a different screen.

It is now the first group inside that panel, in the same
`ml-filter-group-label` / `ml-cats` / `ml-tile` markup as everything else. Read
top to bottom the list narrows from *where*, to *what kind*, to *which words*.

**It renders nothing on a single-venue install**, which is the operator's actual
request and was already true on one page and false on forty:
`admin-opening-hours` had wrapped its include in `venues|length > 1` since S131
and `_admin_list` never did. The test moved into the partial so neither call site
can forget it, and the redundant guard came out of the hours page.

Two details the tiles carry that the select did not: `page` is dropped from every
link — narrowing to a sub-venue with fewer rows while staying on page 4 shows an
empty list that reads as "this sub-venue is empty" — and the "does not change
your permissions" line appears only while a sub-venue is actually selected,
rather than permanently reassuring about a filter that is not filtering.

🔴 **`allow_all|default(true)` was always true.** Twig's `default` fires on any
*empty* value, not only an undefined one, and `false` is empty. `/admin/horaires`
passes `allow_all: false` precisely because opening hours are stored per venue,
and it had been offering "all sub-venues" since S131 — the one option its own
include comment says must not exist there. Choosing it does not error:
`VenueContext::single()` reads `all` as "not specified" and falls back to the
default venue, so the operator edits the default venue's week while the URL says
`location=all`. That is the silent-wrong-row failure S131 was written to prevent,
surviving inside the fix for it. `allow_all is not defined or allow_all` is the
test that distinguishes undefined from false.

⚠️ **`admin.css` line 1 is `@import url("machines-list.css")`,** which is how
`.ml-tile` reaches ~41 admin pages that never name that file. Two consequences.
Grepping rendered HTML for `machines-list.css` finds nothing and does not mean it
is absent — a same-origin mirror of the four linked stylesheets, built to inspect
the cascade, omitted the imported file and rendered the tiles unstyled, which
briefly looked like a discovery and was an artifact of the mirror. And the
imported file is fetched at a URL the page's `?v=` never touches, so editing it
requires bumping **both** the import's own version and `admin.css`'s, or a cached
`admin.css` keeps importing the old URL.

**Verified on CT 210.** 13 pages rendered: the old `<select>` absent on all of
them; three tiles with exactly one lit on machines, places, events and loanable
items; `?location=fabshop` lighting FabShop and the counts below it dropping to
that venue's single machine; `/admin/horaires` down to two tiles with no "all";
and no picker at all on the pages that carry no venue context. The install has
two active venues, so this is the positive case measured directly rather than a
count that happens to match. 31 tests / 780 assertions; 34 admin pages still 200;
50 pages still footered; hash comparison over 75 files identical.

### S130d · Réseau FabOS becomes a Configuration tab — ✅ shipped 2026-08-11

Operator's call, and the right one. Réseau FabOS was a top-level admin section
holding exactly one screen. That cost it a whole sidebar row — level with
Équipement's eleven screens and Formations' catalogue — to say one thing, and
under S130b's rule a one-entry section draws no strip, so `/admin/network` had no
sub-navigation at all.

What that screen configures is the instance's own Ed25519 identity, its OIDC
providers and its trusted peers. That is configuration, not a workspace an
operator works inside. It now sits in the Configuration section between E-mails
and Configuration initiale — which stays last, because it is the one entry you
use once and never again.

Two consequences worth noting. `/admin/network` gained a sub-navigation it never
had. And the sidebar lost a row, which is the point: the sidebar lists workspaces,
and a single settings screen was never one.

Institutions did **not** move with it. `Institution` is referenced by exactly one
entity — `Badge`, many-to-many — so it is a badge issuer, not a federated peer,
and it stays under Badges. The retired workspace registry had claimed it for
Réseau; that disagreement is what S130b resolved.

`admin_nav.section.network` was removed from all five catalogues with the section.
The entry key `admin_nav.entry.app_admin_network` stays and is now read from
Configuration.

**Verified on CT 210.** Ten pages rendered: no Réseau row in the sidebar; every
Configuration screen showing the same seven-entry strip with exactly one lit;
`/admin/network` lit inside it; Badges and Institutions untouched. 31 tests / 769
assertions. 34 admin pages still 200, 50 still footered. ⚠️ The sub-venue sweep
returned four tiles instead of three mid-session — not a regression: a third
sub-venue, "Batiment D", was created on the install while this was being built,
and the tile row picked it up on its own. That is the data-driven behaviour S130c
was after, confirmed by accident.

### S130e · the list format, decided — ✅ 2026-08-11

Four rounds with the operator, in `/admin/design`. The proposals have been removed
from that page; only the retained format remains. What follows is the reasoning, so
removing them costs nothing.

**Round 1 — three panels.** The diagnosis mattered more than any of them: the
panel stacked **three different kinds of control as if they were peers** — a scope
(sub-venue), facets (category, status) and a query (search) — and two of the three
grow on their own, one with the organisation and one with the catalogue. Measured:
394 px of controls before the first row, 446 px projected.

**Round 2 — the page shape.** The operator took A and changed it: title left,
sub-venue **right** in the hero. That forces the create action out of the hero, and
it landed in the list header as a green `+`.

**Round 3 — search and the button.** Search moved out of the filter panel into the
list header, after the count: it finds *one* row, so it belongs to the list. The
add button went back into the hero, tested as an icon-only green circle and as a
named green pill. The pill won — an icon beside a sub-venue menu can read as "add
a sub-venue", and a `title` does not fix a first visit.

**Round 4 — the final shape.** Sub-venue left the hero entirely and became the
**first dropdown under "Affiner"**: it is a filter like the others, and it earned
neither its own bar nor a place in the hero. The hero keeps the title and the one
green named button. **124 px**, down from 394.

**What it cost to build, and is worth knowing.**

🔴 **`style.css` line 3191 repaints every `<span>` inside `.admin-panel`.**
`html[data-theme="dark"] body :is(… .admin-panel …) :is(p, span, small, li, …) {
color: var(--color-text-light) !important }`. The design page *is* an
`.admin-panel`, so the white `+` on the green pill rendered grey-on-green —
measured at `rgb(212, 200, 210)`. No specificity beats an `!important`; the rare
`!important` in that section exist for this. Invisible to lint, invisible in the
Twig, visible only by interrogating the cascade.

⚠️ **Twice in one session I sliced the file into head/tail, then edited the
original, then reassembled from the stale head — discarding the edit.** Both times
the symptom was a mockup that rendered subtly wrong rather than an error. When
splitting a file to swap a block, re-slice *after* every other edit, or apply the
edits to the reassembled file.

⚠️ **The first version of the comparison table asserted "3 rangées" for one
proposal from reasoning.** On a page whose stated purpose is measuring rather than
asserting, that was the wrong kind of claim. Every projection is now produced by
cloning the real panel, adding the missing tiles and measuring the clone.

**Next:** Phase G3 (S134h–S134j) applies the format to all 41 lists, defines a
column vocabulary and deletes ~590 lines of local list CSS. ⚠️ "Sous-lieu" is a bad
word and is to be renamed in one catalogue pass during S134i — `Venue`/`VENUE` stay.

## S134h–S134j — Phase G3, les listes (2026-08-11)

Le format arrêté en S130e était une maquette `dzf-` qui ne touchait aucun écran.
Cette session en fait le composant et y met toutes les listes d'administration.

**S134h — un composant, pas 41 copies.** `_admin_filters.html.twig` : une seule
rangée de tuiles pour la facette qui définit la page, puis « Affiner » en listes
déroulantes qui s'appliquent au changement, **le sous-lieu en premier**, le groupe
entier absent sur une installation à un seul sous-lieu. La recherche descend dans
l'en-tête de liste, à côté du compte, qui devient « 3 sur 11 ». Le bouton d'ajout
monte dans le bandeau, vert et nommé.

⚠️ Le vert : `--color-ok` est une couleur de **texte** — `#86efac` en sombre, pour
que « disponible » se lise sur un panneau foncé. Rempli dans un bouton, ça donne
une dalle fluo portant du blanc à 1,7:1, et c'est ce que l'opérateur a signalé.
`--color-ok-fill` / `--color-ok-on-fill` sont la paire pour le vert **surface** :
5,01:1 dans les deux thèmes, et volontairement identique d'un thème à l'autre
puisqu'il n'est jamais peint que sur le bandeau `#9E1B56`.

**S134i — six types de cellules**, chacun une classe et un partial, démontrés dans
`/admin/design#colonnes` avec les vraies machines de l'installation. `_cell_date`
**exige** sa convention (`machine` | `wall`) et n'en devine aucune : se tromper est
silencieux, plausible, et faux du décalage du lab. Sans convention, la cellule
affiche un marqueur visible plutôt qu'une date.

**S134j — douze listes remappées.** Cinq copies privées de la pastille d'état
(`.status-badge`, `.ml-state`, `.loan-status`, `.m-status`, `.pill`), deux copies
identiques de `.mat-thumb`, douze copies de la règle responsive du bandeau, deux
littéraux `#6b7280` dans des cellules.

🔴 **La prémisse chiffrée de S134j était fausse** : les « ~590 lignes de `<style>`
local » étaient les totaux de lignes des fichiers, moins quatre. Corrigé dans
`ROADMAP.md`.

**Ce qui a été trouvé en chemin, tout en production avant la session :** `.sr-only`
défini dans une feuille que deux pages chargent, donc son texte imprimé sur chaque
pastille de filtre actif de l'admin ; toutes les pastilles d'état grises en thème
sombre ; 4,09:1 et 4,08:1 en thème clair ; les comptes de tuiles calculés sur la
collection filtrée sur quatre listes ; `categoryTiles()` sans `active` ;
`machineListQuery` portant deux filtres sur cinq ; `niveau` proposé deux fois sur
les formations ; et le mot stocké interpolé en nom de classe sur les utilisateurs.

**Vérifications :** `lint:twig` (199 fichiers), `lint:yaml` (5), les 165 routes
rendues (six échecs, tous préexistants et tous des routes POST/OIDC), les hachages
des 103 fichiers comparés un à un sur CT 210, et le contraste des onze états plus
le bouton mesuré dans le navigateur : **rien sous 4,5:1 dans aucun des deux thèmes.**

## S135 — le même objet partout (2026-08-11)

Phase G3 avait mis le format et le vocabulaire en place sur douze listes. S135
les met sur **toutes**, et l'audit qui va avec a trouvé ce que « chaque page a
écrit sa propre version » coûtait vraiment.

**Sept familles de pastilles d'état** existaient en parallèle : `.status-badge`,
`.ml-state`, `.loan-status`, `.m-status`, `.status-pill`, `.pill` (deux fois,
sur deux pages, avec des valeurs différentes), `.badge-yes`/`.badge-no` (deux
fois, identiques) et `.physical-validation-status`. Toutes disent la même chose ;
aucune ne le disait pareil. Une seule reste, `_cell_state`, qui prend un
**signal** et jamais une valeur stockée.

**Six pages hors coquille** y sont entrées — les deux écrans RFID, les
inscriptions d'événement, les pages introuvables, et les lieux et
accès exceptionnels qui n'avaient jamais reçu ni panneau, ni recherche, ni
compte. `/admin/access-rfid-logs` passe de dix colonnes à cinq et
`/admin/utilisateurs/{id}` de huit à cinq : rien n'est perdu, les paires
deviennent titre + sous-titre, ce que la liste des utilisateurs faisait déjà.

🔴 **Deux bugs de cascade, dont un créé puis rattrapé dans la même session.**
`_cell_state` a été posé sur `/machines/{id}/calendrier`, une page **publique**,
qui ne charge pas `admin.css` : la pastille sortait sans aucune règle. C'est la
faute de `.sr-only` à l'identique. Le vocabulaire vit dans `components.css`
depuis, que `base.html.twig` émet partout. Et une fois stylée, elle mesurait
`rgb(212, 200, 210)` — `calendar-modern.css` porte
`html[data-theme="dark"] .calendar-stat-card span` en (0,2,1) contre (0,2,0) pour
la pastille : **une feuille de page repeignait un composant partagé.**
`:not(.status-badge)` sur ce balayage, comme dans `style.css`. Mesuré après :
7,32:1, le même objet et la même couleur qu'en administration.

🔴 **Et `machine.statut` était encore imprimé brut** sur le calendrier machine —
le mot de la base, donc `idle` en anglais sur une page française, avec la classe
tirée de ce même mot. La faute exacte que S84 avait corrigée sur
`/admin/machines`, survivante côté public.

**Vérifications :** `lint:twig` 199 fichiers, les 165 routes rendues (six échecs,
tous préexistants et tous POST/OIDC), et les hachages comparés fichier par
fichier sur CT 210.

## S134c2 / S134g / S137 — le produit arrête de mentir, et le compte appartient au membre (2026-08-12)

**S134c2 — FabOS n'invente plus le contenu d'une formation.** Une formation aux
champs vides servait à un membre un programme en quatre points, **trois sessions
à venir** rattachées à aucune donnée et impossibles à réserver, trois objectifs,
deux prérequis, trois éléments de matériel — et, trouvés en chemin, une
**description** et une **catégorie**. ⚠️ Les listes étaient *traduites*, ce qui
aggravait la chose : l'invention était courante en cinq langues. Vide veut dire
vide ; le bloc n'est plus dessiné. Les exemples restent dans
`FormationPageContentService::EXAMPLES` pour l'éditeur admin — une suggestion à
un opérateur, jamais un fait servi à un membre.

**S134g — le mot de passe oublié, de bout en bout.** 🔴 `/forgot-password` était
une **page morte** : GET seulement, aucun formulaire, aucune route POST, et la
page de connexion y renvoyait. Un membre bloqué dehors arrivait sur un écran
incapable de réinitialiser quoi que ce soit.

⚠️ **Jeton signé plutôt qu'une table**, et c'est ce qui a permis de livrer sans
migration — une migration doit être lancée à la main par l'opérateur, donc un
design qui en exige une ne peut pas partir avec les écrans qui s'en servent.
Haché (HMAC-SHA256 sur APP_SECRET), expirant (l'échéance est *dans* la charge
signée), à usage unique (la charge s'engage sur une empreinte du hash de mot de
passe courant : s'en servir change le hash, ce qui tue ce jeton **et tous les
autres** en cours pour ce compte).

Sept propriétés exercées sur CT 210 contre un compte réel : valide, correspond au
compte, expiré refusé, MAC altéré refusé, ordures refusées, mauvais secret
refusé, mort après changement de mot de passe. **Toutes PASS.**

⚠️ La réponse est la même que l'adresse existe ou non — sinon le formulaire
devient un oracle d'appartenance. Le mail part en `transactional`, pour qu'un
membre désabonné des annonces ne soit pas enfermé dehors par sa propre
préférence. Et la page « lien expiré » ne dit pas *pourquoi*.

**S137 — quatre propositions d'en-tête pour les grilles publiques**, préfixées
`dzp-`, dans `/admin/design#catalogue`. Mesurées avant la première carte :
A (aujourd'hui) 168 px, B (la bande admin) 201 px, C (une barre) 150 px,
D (recherche d'abord) 252 px. Une seule variable change d'une à l'autre.

## S139 — la recherche cherche dans tout le produit (2026-08-16)

Cinq rapports de l'opérateur, tous le même défaut : `usb` (objet prêtable),
`valentin` (projet), `D251` (espace réservable), `anniversaire` (événement
passé), les matériaux, le corps des pages personnalisées — rien ne remontait.
`/recherche` regardait **quatre** types d'objets sur dix et rendait un « aucun
résultat » propre pour les six autres. ⚠️ **Une recherche qui ne cherche pas est
indiscernable d'une recherche qui ne trouve pas** : c'est pour ça qu'elle a
tenu si longtemps, et c'est pour ça que le test qui la garde lit les
déclarations plutôt que des résultats.

**S139a — la couverture.** `SiteSearch` remplace les 75 lignes inline du
contrôleur : dix groupes au lieu de quatre, chacun derrière `allowsSurface()`.

- 🔴 **Chaque badge pointait vers `app_admin_badges`**, donc un membre qui
  cliquait tombait sur le mur de connexion. `/badges/{id}` existait depuis
  toujours.
- 🔴 **`getStatut()` était imprimé brut** dans la ligne meta d'une machine —
  `idle` en anglais sur une page française, exactement la faute que S84 puis
  S135 avaient retirée ailleurs. `getStatusKey()` désormais.
- ⚠️ Les événements passent par `findAll`, pas `findUpcoming` : « À venir » est
  une façon de **naviguer**, pas une règle de **recherche**.
- ⚠️ Les créations passent par `findPublishedForGallery` : une création non
  publiée est invisible en galerie et doit le rester en recherche.
- Les titres de groupe étaient des littéraux français servant de **clés de
  tableau** puis imprimés tels quels sur une page en cinq langues. Ce sont des
  clés `nav.*` : un résultat est classé sous l'entrée de menu qui y mène.

**S139b — les destinations.** L'opérateur a ensuite cherché `horaires`, puis
`heures`. Aucune couverture ne pouvait répondre : **aucune ligne ne s'appelle
comme ça.** Les horaires sont sept `OpeningHour` rendus dans une carte du deck
d'accueil, et personne ne tape « lundi 09:00–18:00 ». Il manquait un second
type de résultat — huit surfaces du produit, chacune avec sa liste de synonymes
traduite dans les cinq catalogues. La carte des horaires a reçu `id="horaires"`
pour qu'un résultat puisse y atterrir.

⚠️ **Correspondance par préfixe sur des synonymes entiers**, pas `str_contains`
dans les deux sens : `heure` doit atteindre `heures`, mais `re` ne doit pas
atteindre les huit destinations — une requête de deux lettres qui matche tout
enterre les vrais résultats en dessous. Un test refuse tout synonyme de moins de
trois caractères, dans les cinq langues.

🔴 **`isEnabled('bookings')` répondait toujours `true`** — le test l'a trouvé.
`bookings` n'est pas une clé du registre (les réservations sont polymorphes
depuis S8–S10 : le critère est « au moins une couche réservable »), et
`isEnabled()` **échoue en ouvert** sur une clé inconnue. La règle vivait en
privé dans `NavBuilder::featureAllows()` ; elle est maintenant
`SiteFeatureService::allowsSurface()`, que la navigation **et** la recherche
lisent. Le commentaire de `NavBuilder` disait déjà « un mot, un sens, un
endroit » — il est enfin vrai.

🔴 **Thème sombre : le diagnostic écrit en août était faux dans sa cause.** La
todo pointait vers le balayage général de `style.css`. C'était
`background: white`, écrit deux fois en clair dans le `<style>` local de
`search.html.twig` — cartes de résultat et cartes de conseil restaient des
plaques blanches sur le fond sombre. Réparé avec les tokens de surface (S83),
et **les deux thèmes vérifiés en pixels** dans le navigateur. Les trois icônes
de conseil portaient aussi le hex littéral de l'accent ; elles l'héritent.

**Renumérotation.** Phase H (commerce) S135–S139 → **S150–S154**, Phase I
(messagerie) S140–S142 → **S155–S157**. Les quatre numéros S135–S138 avaient
déjà été livrés en interface les 11 et 12 août pendant que la table les
réservait au commerce. Les numéros livrés ne bougent pas ; c'est le travail non
commencé qui se déplace. Prochain numéro libre : **S140**.

**Vérifications :** `lint:twig` 201 fichiers, `lint:yaml` 5, **38 tests / 1 607
assertions** (35/904 avant, 31/780 en S135), les 163 routes balayées (trois
échecs, tous préexistants : `/.well-known/fabos` et les deux routes legacy
`/machine/new` et `/machines/new`, signalées à l'opérateur), les hachages des 12
fichiers comparés un à un sur CT 210, et les requêtes de l'opérateur rejouées
sur le site en fr/en/de.

⚠️ **Reste de S139 :** S139c — la page porte encore 23 règles CSS locales pour
15 classes à elle. Trois de ses cinq formes ont déjà un équivalent livré
(`_cell_title`, `_cell_state`, le `frame: 'full'` de `_catalogue`) ; seules les
deux formes de « conseils » sont réellement nouvelles et méritent d'entrer dans
`/admin/design`. Documenter les cinq bénirait une copie.

## S139c / S139d — la recherche compose, les routes legacy disparaissent, un événement passé se voit (2026-08-16)

**S139c — ce qui existait déjà a été rendu, pas redessiné.** `search.html.twig`
portait 23 règles pour 15 classes à elle. Le titre et sa seconde ligne sont
`_cell_title`, la pastille de catégorie `_cell_state` en signal `muted`. Trois
règles supprimées — ⚠️ **le compte n'est pas l'intérêt** : ces trois formes ne
peuvent plus diverger du reste de FabOS.

🔴 **Le piège de cascade que ça a révélé, et qui aurait été livré sans mesure.**
`_cell_title` est un partial `_cell_*`, donc appelable de partout — mais ses
règles (`.admin-cell-user`, `.admin-cell-stack`, `.is-strong`, `.is-meta`)
vivaient dans `admin.css`, **qu'une page publique ne charge jamais**. C'est mot
pour mot la faute de S135 avec `_cell_state` sur `/machines/{id}/calendrier`.
Les quatre règles sont dans `components.css`.

🔴 **Et une deuxième, trouvée en regardant les pixels et pas le balisage.** Une
fois le composant stylé, le **titre sortait à 1,16:1** en thème sombre — presque
noir sur la carte sombre — pendant que le sous-titre à côté était correct.
`.is-meta` nomme sa couleur ; `.is-strong` la laissait à l'héritage, et ce
codebase a plusieurs balayages qui repeignent un `<span>` nu en `!important`.
**Un composant partagé ne doit pas dépendre de gagner cette course** : il nomme
sa propre couleur. Mesuré après : **13,65:1 sombre, 17,4:1 clair**.

⚠️ `getComputedStyle` a menti pendant le diagnostic — il a rendu la chaîne
entière, `body` compris, en `rgb(26,26,26)`, ce que la capture d'écran
contredisait (les titres de section et le pied étaient blancs). La capture avait
raison. Diagnostiquer sur les pixels, confirmer sur une mesure, jamais l'inverse.

Ce qui reste est documenté dans `/admin/design#recherche` : un panneau et une
carte de conseil, que rien de partagé n'exprime. ⚠️ La section dit aussi
pourquoi les trois autres formes **n'y sont pas** — un guide qui accueille
chaque variante locale devient un catalogue de dettes.

**S139c bis — toutes les routes legacy supprimées** (opérateur : « pre V1 dev
site, no need to retain legacy anything » — donc supprimer, pas rediriger).
**44 chemins**, 163 routes → 119 : tous les `.html`, tous les `_legacy`, le
`/machine` singulier, le `/calendar` anglais, les six redirections
d'administration et `/search`, doublon anglais de `/recherche`. Le formulaire de
l'en-tête pointait sur `app_search` : repointé sur `app_recherche`.
`LegacyAdminController` entier est parti, ainsi que six méthodes devenues
orphelines et trois alias morts dans `NavBuilder`.

⚠️ Déclencheur : `/machine/new` et `/machines/new` rendaient **500** — deux
routes **publiques sans attribut de sécurité** qui rendaient
`admin-machines.html.twig`, gabarit passé sur la coquille de liste en S134h/S135
et qui attend des variables qu'elles ne passaient pas. Elles n'existent plus.

**S139d — un événement passé se voit.** L'opérateur : « la mention est petite et
les inscriptions ont toujours l'air actuelles ». Mesuré avant de toucher : une
carte passée et une carte à venir étaient le **même** `<article class="ml-card">`
— affiche pleine couleur, même taille, même action — séparées par une seule
pastille grise, qu'on lit en dernier.

- `_catalogue_card` apprend `spent` : l'affiche se désature et le titre
  s'éteint. ⚠️ Pas de `filter` sur la carte entière — `annulé` doit rester rouge
  sur un événement passé, c'est un autre fait. ⚠️ La carte garde bordure, ombre
  et survol : elle reste une destination réelle ; l'éteindre dirait « cassé »
  plutôt que « fini ».
- Le pied disparaît sur une carte passée. Il disait « inscriptions fermées » —
  du **vocabulaire d'inscription**, et c'est précisément ce qui la faisait lire
  comme courante : une inscription fermée est une chose qui vient d'être
  ouverte. Un événement passé n'a pas d'histoire d'inscription. Annulé garde sa
  ligne, parce que c'est une nouvelle.
- 🔴 **Deux horloges sur la même carte.** `EventRepository` sélectionne en heure
  murale (`nowInStoredForm()`), mais le gabarit classait via
  `Event::isRegistrationOpen()`, qui compare à `new \DateTimeImmutable()` —
  l'instant **serveur**. Près de minuit, le filtre et la pastille divergeaient
  du décalage du lab. `storedNow()` est exposé, le contrôleur calcule `past`,
  le gabarit ne calcule plus rien.
- 🔴 **« 0 / 3 disponibles » au-dessus de trois cartes visibles** sur
  `?when=all`. La paire retombait sur `common.available_short` faute
  d'`available_word` — et « disponible » n'est pas un état d'événement. La clé
  `events.headline_upcoming` existait déjà et disait « à venir ».

⚠️ **Le piège Twig, une troisième fois, dans le fichier qui le documente.** Un
commentaire placé **entre deux clés d'un hash d'arguments** est une erreur de
syntaxe : « Unclosed block ». `events.html.twig` porte l'avertissement depuis
deux sessions ; la note posée à côté de `foot:` a quand même cassé la page.
`lint:twig` l'a attrapée **avant** le `cache:clear` et le restart — c'est
exactement pourquoi il passe en premier. L'avertissement couvre maintenant aussi
le hash du `{% embed %}`.

**Vérifications :** `lint:twig` 201, `lint:yaml` 5, 38 tests / 1 607 assertions,
**les 119 routes balayées — un seul non-2xx/3xx, `/.well-known/fabos` qui rend
`503 {"status":"unconfigured"}` volontairement**, les quatre chemins supprimés
confirmés en 404, contrastes mesurés dans les deux thèmes, et les hachages
comparés fichier par fichier sur CT 210.

## S139e — le fil d'Ariane n'avait aucune règle, et un événement passé le dit maintenant fort (2026-08-16)

🔴 **« Le fil d'Ariane a l'air d'être du texte brut. »** Il l'était.
`_breadcrumb.html.twig` est un partial partagé, mais ses règles vivaient dans
`details.css`, que **36 gabarits sur 201** chargent — et `/events/{id}` n'en
fait pas partie : cette page n'émet que `style.css` et `components.css`. Le
composant sortait donc **sans une seule règle**.

⚠️ **Troisième fois dans la même session, même faute :** `_cell_title` dans
`admin.css` (S139c), puis sa couleur laissée à l'héritage, puis ceci. La règle
est écrite dans `/admin/design#fil-ariane` : **les règles d'un partial partagé
vivent dans `components.css`**, la feuille que `base.html.twig` émet partout.
Une feuille que seules certaines pages chargent ne peut pas styler un composant
commun.

🔴 **Et le fil avait deux formes.** En thème sombre il portait une **pilule** —
fond, bordure, rayon, padding — que le clair n'avait pas. Un thème change des
couleurs, pas la forme d'un composant. Pilule retirée ; sombre ne redéfinit plus
que la couleur, par token au lieu du littéral `#d7c9d4`.

**Un événement passé, sur sa propre page.** S139d n'avait traité que les cartes
du catalogue ; l'opérateur regardait la **fiche**, où rien n'avait bougé. Ce
qu'elle affichait encore pour un événement terminé : un panneau « Inscription »
avec un nombre de **places restantes**, une **jauge animée**, un compteur de
liste d'attente, et un bouton d'inscription. Tout le vocabulaire de quelque
chose qu'on peut encore rejoindre.

- Le panneau devient « Participation » : le nombre de **participants**, à plat,
  sans jauge. Ce qu'une fiche d'événement passé doit au lecteur, c'est combien
  sont venus, pas combien pourraient encore venir.
- La pastille d'état de son inscription reste — c'est le fait qu'il avait une
  place — mais **le bouton « Annuler » disparaît** : on ne se retire pas d'une
  chose qui a déjà eu lieu, et un bouton qui ne peut qu'échouer est pire que pas
  de bouton.
- 🔴 **Troisième horloge.** `'registrationOpen' => $event->isRegistrationOpen()`
  comparait encore à l'instant serveur. La liste, la carte et la fiche doivent
  s'accorder sur ce qui est passé, sinon une carte dit « Terminé » et sa propre
  page propose une place.
- ⚠️ **« Événement passé » était trop petit** (opérateur). La bannière `.ev-cancelled`
  existait déjà pour les annulations : elle devient `.ev-notice` **plus un ton** —
  les noms que les pastilles de la même page utilisent déjà. **Un élément,
  plusieurs tons**, plutôt qu'une seconde classe de bannière.
- ⚠️ **Puis : « le manque de couleur la rend quelconque. »** Le ton gris était
  sémantiquement juste et visuellement muet. 🔴 **On n'a pas emprunté un feu
  tricolore pour autant** : un événement fini n'est ni une erreur (rouge), ni un
  avertissement (ambre), ni une disponibilité (vert), et se servir d'un signal
  pour attirer l'œil est exactement comment un vocabulaire de signaux cesse de
  vouloir dire quelque chose. Le système en a un quatrième qui n'est pas un
  signal : **l'accent**. `is-accent` utilise `--tone-primary-soft` (S83), donc
  une installation qui change de couleur emporte la bannière avec elle au lieu
  de garder le magenta FabOS. Mesuré : **5,51:1 sombre et 6,23:1 clair** sur le
  titre, 13:1 et 14,18:1 sur le texte.
- ⚠️ Et la phrase « cet événement a eu lieu » a quitté le panneau latéral : la
  bannière la dit déjà, et sur mobile les deux s'empilaient en une répétition.
- L'affiche est désaturée comme sur la carte (`.ev-page.is-spent`), donc un
  événement fini a la même tête dans la liste et sur sa fiche.

**Vérifications :** `lint:twig` 201, `lint:yaml` 5, 38 tests / 1 607 assertions,
rendu vérifié en pixels sur `/events/8`, et `?v=` porté à `20260816-s139e` sur
les quatre feuilles touchées — ⚠️ y compris l'`@import` de `machines-list.css`
dans `admin.css`, qu'un `?v=` sur le `<link>` n'atteint pas.

## S134g moitié 2 — le compte appartient au membre, jusqu'à sa disparition (2026-08-16)

**Décision opérateur, qui a débloqué la session :** « stats should stay,
bookings and all. If user Pierre got deleted, we should still see his activity
in stats, projects untouched, leaderboard as well. Maybe just his name gets
changed? » Donc **anonymisation, jamais suppression**. Chaque ligne survit ; la
personne s'en va.

⚠️ **Ce n'est pas un contournement du RGPD, c'est sa lecture.** L'article 17
donne un droit à l'effacement des **données personnelles** ; le considérant 26
place les informations anonymes hors du règlement. Effacer les identifiants et
garder les lignes honore la demande *et* conserve l'histoire du lab — qui n'est
plus la donnée personnelle de personne dès lors que nul ne peut dire de qui il
s'agissait.

🔴 **Tout repose donc sur l'IRRÉVERSIBILITÉ.** S'il subsiste où que ce soit une
correspondance vers la personne, c'est de la *pseudonymisation* : les lignes
restent des données personnelles et l'effacement n'a pas eu lieu. D'où :

- aucune table « comptes supprimés », aucune copie d'archive, aucune ligne
  d'audit qui les nomme ;
- l'adresse est **écrasée et non hachée** — le hachage d'une adresse connue se
  ré-identifie en testant des candidats ;
- l'avatar et la bannière sont **supprimés du disque** : une ligne qui cesse de
  nommer le fichier n'efface pas le visage qui est dedans ;
- les lignes `EXTERNAL_IDENTITY` partent, sinon la prochaine connexion OIDC
  reconstruit le compte depuis les claims du fournisseur et défait tout.

**Les satellites, chacun pour une raison qui mérite d'être dite.** `EMAIL_LOG`
garde l'adresse, le nom affiché **et** un contexte qui nomme la machine et les
horaires réservés. Une inscription faite **en invité** porte un nom et une
adresse saisis dans le formulaire, hors du compte. Et `AccessRfidLog.badgeUid`
est le numéro de la carte physique : un identifiant aussi personnel qu'une
adresse, qui survit au compte parce que le journal est une piste d'audit. Le
scan reste — c'est la statistique — le numéro non.

**Ce qui est gardé** : réservations, passages machine, emprunts, badges, points,
temps de présence, progression, votes, créations. Tout pointe vers le même id,
devenu un simple numéro de ligne qui ne désigne personne.

⚠️ **Deux entrées, un seul service.** La page du membre et l'écran d'admin
appellent le même `AccountAnonymiser`. Une seconde implémentation côté admin
serait une seconde définition d'« effacé », et celle qui dérive est celle qui
laisse des données derrière.

⚠️ **La confirmation est l'IDENTIFIANT tapé, pas le mot de passe.** Un mot de
passe exclurait quiconque se connecte par fournisseur d'identité et n'a donc pas
de mot de passe local utilisable — précisément les membres les plus susceptibles
de vouloir effacer leur copie locale. Taper son propre nom est une friction que
tout le monde peut franchir et que personne ne franchit par accident.

🔴 **L'invariant de verrouillage, testé pour de vrai.** `AccountGuard` refuse le
dernier administrateur actif, et deux cas subtils sont l'intérêt du test : un
admin **déjà anonymisé** ne compte pas comme le second (la ligne existe, un
comptage naïf dit « c'est bon », mais ce compte ne pourra plus jamais se
connecter), et un admin **suspendu** non plus. Compter des lignes plutôt que des
administrateurs utilisables *est* le verrouillage. L'effacement n'a pas d'annulation :
un opérateur qui se trompe ici n'a plus personne à qui demander.

⚠️ **Aucune migration.** Le marqueur est le domaine réservé `.invalid`
(RFC 2606), qui ne peut jamais résoudre vers une vraie boîte. Une migration doit
être lancée à la main par l'opérateur, donc un design qui en exige une ne peut
pas partir avec les écrans qui s'en servent — même raisonnement qu'au jeton
signé de la moitié 1.

⚠️ **Le nom stocké est `Anonyme #<id>`**, et c'est le seul endroit où la règle
« traduire l'interface, jamais le contenu » plie : `getDisplayName()` a **83
sites d'appel** et une entité n'a pas à porter un traducteur. Le `#id` garde deux
membres effacés distinguables dans un classement sans rien dire ni de l'un ni de
l'autre. À revoir le jour où l'affichage passera par une clé.

**Vérifications :** `lint:twig` 202, `lint:yaml` 5, **53 tests / 1 822
assertions** (46/1 813 puis 38/1 607 avant), `/profil/supprimer` rendu 200
derrière l'authentification avec ses deux listes et son champ de confirmation,
le panneau d'admin rendu sur quatre fiches, et les hachages comparés sur CT 210.

### S134g — le 500 en production, et ce qu'il a révélé (2026-08-16, même jour)

L'opérateur a essayé d'anonymiser un compte et a eu un **500**.

🔴 **`Utilisateur::setPublicFields()` prend `array`, on lui passait `null`.** La
vérification avait contrôlé que chaque setter **existait**, jamais qu'il
acceptait ce qu'on lui donnait — et les tests de contrat affirment qu'un setter
est *appelé*, ce qu'un `TypeError` traverse sans les déranger.

🔴 **La moitié la plus grave était l'ordre.** Les nettoyages en SQL brut
tournaient **avant** le point de plantage, et DBAL valide immédiatement. Une
exception à mi-parcours laissait donc `EXTERNAL_IDENTITY` supprimé et
`EMAIL_LOG` nettoyé **pendant que le compte gardait son nom** — une érasure
*partielle*, dont chaque partie est irréversible. Le compte visé n'avait aucune
ligne dans ces deux tables : rien n'a été perdu, mais c'est de la chance, pas du
design. Tout le scrub est désormais dans une transaction : **ou la personne est
effacée, ou rien ne s'est passé.**

⚠️ Un commentaire écrit le matin même défendait qu'un passage à moitié fait
était « plus facile à raisonner ». C'était justifier un ordre, pas empêcher un
problème. Il est supprimé.

⚠️ **Trouvé en corrigeant :** déplacer l'`unlink` après la transaction ne
supprime **rien**, parce que la ligne ne connaît plus le nom des fichiers. Ils
sont capturés avant, et l'`unlink` reste hors transaction — il n'a pas de
rollback, et perdre la ligne est la moins mauvaise des deux pannes.

**Trois tests ajoutés, le premier écrit après le bug qu'il aurait attrapé :**
chaque `setX(null)` de l'anonymiseur est confronté par réflexion à la vraie
signature, l'érasure doit être atomique, et les noms de fichiers doivent être lus
avant d'être effacés.

⚠️ **Vérifié autrement qu'en faisant passer des tests.** Une commande jetable a
exécuté le **vrai** scrub contre le compte réel dans une transaction, puis a
annulé : `ulpzugfv@immenseignite.info` → `anonymised-8@anonymised.invalid` /
« Anonyme #8 », sans erreur, compte intact après rollback. Commande supprimée du
conteneur ensuite. 56 tests / 1 844 assertions.

**La leçon, plus large que ce bug :** vérifier qu'une méthode existe n'est pas
vérifier qu'on peut l'appeler. Quand un service pilote des dizaines de setters
d'entité, c'est la **signature** qu'il faut confronter, et un test par réflexion
le fait sans base de données.

---

## S141 — la carte fusionnée devient LE format de liste (2026-08-16)

**Le format avait été validé sur une page et restait un paramètre.** S140 avait
fusionné les trois cartes d'`/admin/machines` en une seule, l'opérateur l'avait
retenue, et le fichier gardait quand même trois formes : le grand bandeau
pleine largeur d'origine (20 pages), `hero: 'compact'` (10 pages) et
`hero: 'merged'` (1 page). S141 supprime le drapeau, les deux autres formes et
la classe de variante `.is-merged`. Les règles s'attachent au shell lui-même,
`.admin-list-card`.

### Le titre est le nom de l'entrée de menu

Décision opérateur du 2026-08-16 : « quotas » plutôt que « gestion des
quotas ». Le mot est déjà écrit dans la barre de sous-navigation ; une page qui
le réécrit tient une deuxième copie, traduite en cinq langues et libre de
diverger. `NavBuilder::adminCurrentTitle()` renvoie
`admin_nav.entry.<route>` et le shell le lit. **`/admin/machines` est la seule
exception** — il porte le libellé de SECTION (« Équipement ») parce que c'est la
page d'atterrissage du groupe et que c'est le rendu validé à l'écran ; l'exception
est un `titleLabel` sur l'entrée, pas un `if` sur la route.

Conséquence : **41 clés mortes supprimées des cinq catalogues**, 205 lignes.
Trente et une étaient les `*.title` et `*.description` par page ; dix étaient des
`*.subtitle` publiques mortes depuis le passage aux grilles de cartes.

🔴 **`page_title`, et pas `title`.** Un `{% embed %}` sans `only` fusionne le
contexte parent, donc tant que le paramètre s'appelait `title`, n'importe quelle
variable de contrôleur du même nom devenait le titre de la page.
`/admin/machines/categories` en passe une : elle a affiché la clé brute
`machine_taxonomy.categories_title` dans le bandeau au premier essai.

### Trois affordances mortes, trouvées en regardant les pages

1. `{% block header_extra %}` n'était imprimé que dans l'ancien en-tête. Les dix
   pages passées en `compact` le perdaient en silence :
   `/admin/rfid-readers` n'affichait plus son bouton « Comment appairer un Pi ? »
   et les instructions d'appairage étaient inatteignables.
2. `sidebar_variant: 'rfid'` (puis `'user'`) rendait un `<aside>` sans aucune
   règle pour ses liens : la sidebar sortait en texte inline replié sur quatre
   pages. Les deux variantes sont supprimées ; `'edit'` reste pour S132.
3. `/admin/quotas-reservation` sans `reservableType` n'allumait aucune entrée et
   sortait donc un `<h1>` vide. Le contrôleur canonicalise l'URL.

### La revue de contenu — la moitié intéressante

🔴 **Cinq tableaux avaient plus d'en-têtes que de cellules.**
`/staff/acces-exceptionnels` (7 pour 6), `/admin/loans` (6 pour 5),
`/admin/utilisateurs/{id}` (5 pour 4), `/admin/homepage` (colonne conditionnelle
déclarée sans condition) et `/admin/access-rfid-logs`. Le motif est le même à
chaque fois : une valeur repliée dans le sous-titre de la cellule de titre, et
l'en-tête laissé derrière. **Rien ne plantait.** Le tableau dessinait chaque
colonne suivante sous le mauvais nom — les dates sous « Portée », un bouton de
révocation sous « État » — et l'en-tête en trop prenait un filet de largeur à
droite, ce qui faisait rendre « Reason » une lettre par ligne.

**`_cell_chip`** — le cas nommé par l'opérateur. `/admin/formations` imprimait le
nom du badge deux fois par ligne, dont une dans une colonne `is-tight` où il
cassait sur trois lignes. Un jeton court cliquable (`#7`), le nom complet en
`aria-label` et en infobulle ; les lignes passent de doubles à 61 px chacune.
Ses règles vont dans `components.css`, jamais `admin.css` — troisième fois que
ce piège est écrit.

🔴 **`overflow-wrap: anywhere` contre `width: 1%`.** `anywhere` laisse une
coupure douce compter dans la largeur *min-content* et `th.is-tight` vaut
`width: 1%`, donc les colonnes serrées se réduisaient à quelques caractères puis
fracassaient leurs valeurs : « 23/07/2 026 », « Salle Impres sion 3D »,
« Impri mante 3D test ». `break-word` corrige les trois d'un mot.

🔴 **`/admin/access-rfid-logs` imprimait `REQUIRED_BADGE_MISSING`**, et deux fois
— `MachineAccessService` écrit la même valeur dans `status` et dans `reason`.
`_rfid_result` porte maintenant le vocabulaire pour les deux pages qui listent
des scans, avec un repli qui **humanise** une valeur inconnue plutôt que
d'imprimer une clé : deux générations de vocabulaire cohabitent dans cette table.

### Les six squelettes d'administration deviennent un

Cinq écrans portaient un tableau sans passer par `_admin_list`, chacun avec son
propre squelette. Ils cachaient trois flashes privés à couleurs littérales, une
page entière repeinte en clair (`.admin-user-page { background: #f6f7fb; color:
#1f2937; font-family: Arial }`), deux boutons publics redéfinis localement dont
un `background: white` sur panneau sombre, et quatre `colspan` comptés à la main
et inatteignables. Le guide de style lui-même documentait un shell qu'il
n'utilisait pas ; **52 règles de maquette** y ont été supprimées avec les
propositions qu'elles imitaient.

🔴 **La mesure que `/admin/design` citait ne mesurait rien.** `paint()` cherchait
une classe que le spécimen avait cessé d'émettre, sortait tout de suite, et les
deux chiffres annoncés « mesurés dans le navigateur » affichaient un tiret
cadratin en production. Ils mesurent maintenant le haut de la carte à la première
ligne : **268 px**, et **346 px** projetés à douze catégories.

### 🔴 Onze clés supprimées par accident, et la classe entière fermée

Une regex censée retirer deux clés de `rfid_logs` a été écrite sans ancre —
`^ {4}col_status: .*$` — et a emporté **tous** les `col_status` du fichier, dans
les cinq langues. Huit listes d'administration et trois pages membres ont
affiché `admin_machines.col_status` en en-tête, entre deux exécutions vertes de
la suite de tests.

Trouvé en balayant les 139 pages rendues à la recherche d'identifiants pointés.
**Le même balayage a trouvé trois clés manquantes qui n'étaient pas de moi** :
`admin_emails.col_status` et `login.email` sur le formulaire **public** de mot de
passe oublié.

`TranslationKeyTest` ferme la classe : toute clé littérale `'x.y'|trans` d'un
gabarit doit exister dans les cinq catalogues. ⚠️ Elle ne regarde que les clés
**littérales** — une clé concaténée ne se vérifie pas sans exécuter le gabarit,
et prétendre le contraire est exactement ce qui supprime treize clés vivantes
(cf. les treize `usage_rights.verdict.*` et `notification.category.*`).

### Mesuré, pas déduit

- **31 pages du shell rendues et regardées**, clair et sombre : un bandeau, une
  carte, zéro `admin-page-header`, bords gauches alignés (carte 323, `<h1>` 348,
  facette 348), aucun débordement horizontal à 1440 px.
- `/admin/machines` : **268 px** du haut de la carte à la première ligne, contre
  430 px pour les trois cartes de S134h et 394 px pour les en-têtes à la main.
- **21 listes sondées** : zéro colonne au-delà de cinq, zéro cellule fracassée,
  zéro fait imprimé deux fois dans une même ligne.
- **139 chemins rendus** : zéro identifiant pointé hors des pages qui citent des
  clés exprès, et pour seuls non-2xx `/.well-known/fabos` 503 et
  `/desabonnement` 400, tous deux voulus.
- `?v=20260816-s141` vérifié par ce que les pages **émettent** : 106 pour
  `style.css`, 106 pour `components.css`, 61 pour `admin.css`.
- **64 tests / 2 096 assertions.**

⚠️ **Reste ouvert :** `/admin/homepage` porte six colonnes (bloc + quatre
audiences + ordre). C'est une matrice d'audiences, pas une liste
d'enregistrements, et le plafond de cinq ne lui répond pas ; le test ne le voit
pas non plus, ses colonnes venant d'une variable. À trancher si la question
revient.
