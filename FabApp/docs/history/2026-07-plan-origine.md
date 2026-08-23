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

